#include "imu.h"
#include <math.h>
#include <string.h>

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} IMU_AxisRaw_t;

// Static state 
static float tilt_angle   = 0.0f;
static float gyro_offset  = 0.0f;
static float acc_offset   = 0.0f;

// CS helpers 
static inline void CS_Low(void)  { HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET); }
static inline void CS_High(void) { HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);   }

// Read multiple gyro registers in one SPI transaction.
static HAL_StatusTypeDef Gyro_ReadBurst(SPI_HandleTypeDef *hspi, uint8_t start_reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[1 + 6] = {0};
    uint8_t rx[1 + 6] = {0};

    if (len > 6U)
    {
        return HAL_ERROR;
    }

    tx[0] = (uint8_t)(0xC0U | start_reg);  // read + auto-increment
    CS_Low();
        HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx, rx, (uint16_t)(len + 1U), IMU_SPI_TIMEOUT_MS);
    CS_High();

    if (st == HAL_OK)
    {
        memcpy(data, &rx[1], len);
    }
    return st;
}

static HAL_StatusTypeDef Gyro_ReadRaw(IMU_AxisRaw_t *raw, SPI_HandleTypeDef *hspi)
{
    uint8_t buf[6] = {0};
    if (Gyro_ReadBurst(hspi, GYRO_OUT_X_L, buf, 6U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    raw->x = (int16_t)((buf[1] << 8) | buf[0]);
    raw->y = (int16_t)((buf[3] << 8) | buf[2]);
    raw->z = (int16_t)((buf[5] << 8) | buf[4]);
    return HAL_OK;
}

static HAL_StatusTypeDef Acc_ReadRaw(IMU_AxisRaw_t *raw, I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[6] = {0};
    if (HAL_I2C_Mem_Read(hi2c, ACC_ADDR_READ, ACC_OUT_X_L_A | 0x80,
                         I2C_MEMADD_SIZE_8BIT, buf, 6, IMU_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    raw->x = (int16_t)((buf[1] << 8) | buf[0]);
    raw->y = (int16_t)((buf[3] << 8) | buf[2]);
    raw->z = (int16_t)((buf[5] << 8) | buf[4]);
    return HAL_OK;
}

static void Gyro_WriteReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    CS_Low();
        HAL_SPI_Transmit(hspi, tx, 2, IMU_INIT_TIMEOUT_MS);
    CS_High();
}

// Initialise gyroscope
static void Gyro_Init(SPI_HandleTypeDef *hspi)
{
    CS_High();  // ensure CS starts de-asserted
    Gyro_WriteReg(hspi, GYRO_CTRL_REG1, GYRO_CTRL_REG1_VAL);
}

//Initialise accelerometer
static void Acc_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t val;
    val = ACC_CTRL_REG1_VAL;
        HAL_I2C_Mem_Write(hi2c, ACC_ADDR_WRITE, ACC_CTRL_REG1_A, I2C_MEMADD_SIZE_8BIT, &val, 1, IMU_INIT_TIMEOUT_MS);
    val = ACC_CTRL_REG4_VAL;
        HAL_I2C_Mem_Write(hi2c, ACC_ADDR_WRITE, ACC_CTRL_REG4_A, I2C_MEMADD_SIZE_8BIT, &val, 1, IMU_INIT_TIMEOUT_MS);
}

// Public init
void IMU_Init(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c)
{
    Gyro_Init(hspi);
    Acc_Init(hi2c);
}

//Read gyro X axis → degrees/sec
float IMU_GetGyroY(SPI_HandleTypeDef *hspi)
{
    IMU_AxisRaw_t raw = {0};
    if (Gyro_ReadRaw(&raw, hspi) != HAL_OK)
    {
        return -gyro_offset;
    }
    return (raw.y * GYRO_SENSITIVITY) - gyro_offset;
}

// Read accelerometer X/Z axes → tilt angle in degrees
float IMU_GetAccAngleX(I2C_HandleTypeDef *hi2c)
{
    IMU_AxisRaw_t raw = {0};
    if (Acc_ReadRaw(&raw, hi2c) != HAL_OK)
    {
        return -acc_offset;
    }

    float ax = (raw.x * ACC_SENSITIVITY) / 1000.0f;  // convert mg -> g
    float az = (raw.z * ACC_SENSITIVITY) / 1000.0f;

    // Tilt from gravity components on X/Z plane.
    float angle = atan2f(ax, az) * RAD_TO_DEG;
    return angle - acc_offset;
}

// Offset calibration (call once while robot is stationary & upright)
void IMU_OffsetCalibrate(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c)
{
    const int SAMPLES = 300;
    float g_sum = 0.0f, a_sum = 0.0f;

    for (int i = 0; i < SAMPLES; i++)
    {
        // Temporarily zero offsets so raw values are read
        gyro_offset = 0.0f;
        acc_offset  = 0.0f;

        g_sum += IMU_GetGyroY(hspi);
        a_sum += IMU_GetAccAngleX(hi2c);
        HAL_Delay(10);
    }
    gyro_offset = g_sum / SAMPLES;
    acc_offset  = a_sum / SAMPLES;
}

// Complementary filter update (call inside timer ISR at 100 Hz)
struct imu_output IMU_UpdateAngle(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c)
{
    float gyro_x = IMU_GetGyroY(hspi);
    float acc_x  = IMU_GetAccAngleX(hi2c);

    // Complementary filter: favour gyro short-term, acc long-term
    tilt_angle = CF_ALPHA * (tilt_angle + gyro_x * DT) + (1.0f - CF_ALPHA) * acc_x;
    struct imu_output output = { tilt_angle, gyro_x, acc_x };
    return output;
}

