#ifndef IMU_H
#define IMU_H

#include "stm32f3xx_hal.h"

// Gyroscope (I3G4250D via SPI)
#define GYRO_CS_PORT     GPIOE
#define GYRO_CS_PIN      GPIO_PIN_3

#define GYRO_CTRL_REG1   0x20
#define GYRO_CTRL_REG1_VAL 0b10001111  // Power on, ODR=200Hz, X/Y/Z enabled
#define GYRO_OUT_X_L     0x28
#define GYRO_OUT_Y_L     0x2A
#define GYRO_SENSITIVITY 0.00875f      // dps/LSB at ±245 dps

//Accelerometer (LSM303AGR via I2C)
#define ACC_ADDR_WRITE   0x32          // 7-bit address left-shifted
#define ACC_ADDR_READ    0x33
#define ACC_CTRL_REG1_A  0x20
#define ACC_CTRL_REG1_VAL 0x67         // 200 Hz ODR, normal mode, X/Y/Z on
#define ACC_CTRL_REG4_A  0x23
#define ACC_CTRL_REG4_VAL 0x00         // Normal mode
#define ACC_OUT_X_L_A    0x28
#define ACC_SENSITIVITY  3.9f          // mg/LSB in normal mode -> divide by 1000 for g
#define RAD_TO_DEG       57.2958f

// Bounded peripheral timeouts (ms) to avoid blocking the control loop.
#define IMU_SPI_TIMEOUT_MS   2U
#define IMU_I2C_TIMEOUT_MS   2U
#define IMU_INIT_TIMEOUT_MS 20U

struct imu_output {
   float tilt_angle;
   float gyro_x;
   float acc_x;
};

//Complementary filter
#define DT 0.01f         // 100 Hz → 10 ms
#define CF_ALPHA 0.985f  // 1.5% accelerometer, 98.5% gyro integration

void IMU_Init(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c);
void IMU_OffsetCalibrate(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c);
float IMU_GetGyroY(SPI_HandleTypeDef *hspi);
float IMU_GetAccAngleX(I2C_HandleTypeDef *hi2c);
struct imu_output IMU_UpdateAngle(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c);

#endif