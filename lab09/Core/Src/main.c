/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f303xc.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
#define LSM303AGR_ADDR 0x32
#define RAD_TO_DEG 57.2958f
#define GYRO_WHO_AM_I_REG 0x0F
#define GYRO_CTRL_REG1    0x20
#define GYRO_CTRL_REG4    0x23
#define GYRO_OUT_X_L      0x28
#define GYRO_SPI_READ     0x80
#define GYRO_SPI_MULTI    0x40

typedef struct {
    int16_t raw_x, raw_y, raw_z;
    float scaled_x, scaled_y, scaled_z;
  float offset_x, offset_y, offset_z;
} LSM303AGR_Data;

typedef struct {
    int16_t raw_x, raw_y, raw_z;
    float scaled_x, scaled_y, scaled_z;
    float offset_x, offset_y, offset_z;
} Gyro_Data;

LSM303AGR_Data acc_data = {0};
Gyro_Data gyro_data = {0};
uint8_t gyro_whoami = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myPrintf (const char *fmt , ...);
void Print_LSM(const LSM303AGR_Data *acc, const Gyro_Data *gyro);

static uint8_t Gyro_SPI_ReadReg(uint8_t reg)
{
  uint8_t tx[2] = {(uint8_t)(GYRO_SPI_READ | (reg & 0x3F)), 0x00};
  uint8_t rx[2] = {0};

  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);

  return rx[1];
}

static void Gyro_SPI_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {(uint8_t)(reg & 0x3F), value};

  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

static void Gyro_SPI_ReadMulti(uint8_t startReg, uint8_t *buffer, uint16_t len)
{
  uint8_t cmd = (uint8_t)(GYRO_SPI_READ | GYRO_SPI_MULTI | (startReg & 0x3F));

  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, buffer, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

void Init_LSM(void) {
    // Initialize LSM303AGR accelerometer
    uint8_t ctrl_reg1 = 0x67;
    uint8_t ctrl_reg4 = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ADDR, 0x20, I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ADDR, 0x23, I2C_MEMADD_SIZE_8BIT, &ctrl_reg4, 1, HAL_MAX_DELAY);
}

void Init_Sensor(void) {

  uint8_t ctrl_reg1 = 0x0F; // normal mode, XYZ enabled
  uint8_t ctrl_reg4 = 0x80; // BDU enabled, +/- 250 dps

  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
  HAL_Delay(5);

  Gyro_SPI_WriteReg(GYRO_CTRL_REG1, ctrl_reg1);
  Gyro_SPI_WriteReg(GYRO_CTRL_REG4, ctrl_reg4);

  gyro_whoami = Gyro_SPI_ReadReg(GYRO_WHO_AM_I_REG);
  myPrintf("GYRO WHO_AM_I (SPI): 0x%02X\r\n", gyro_whoami);
}
void Read_LSM(LSM303AGR_Data *data) {
    uint8_t raw_data[6];
    
    // Read high and low bytes for X, Y, Z
    HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_ADDR, 0x28 | 0x80, I2C_MEMADD_SIZE_8BIT, raw_data, 6, HAL_MAX_DELAY);
    
    // In normal mode (10-bit), output is left-aligned in 16-bit registers, so shift by 6.
    data->raw_x = ((int16_t)((raw_data[1] << 8) | raw_data[0])) >> 6;
    data->raw_y = ((int16_t)((raw_data[3] << 8) | raw_data[2])) >> 6;
    data->raw_z = ((int16_t)((raw_data[5] << 8) | raw_data[4])) >> 6;
    
    // Scale to physical units (g) - multiply by 0.0039 (3.9 mg/LSB sensitivity)
    data->scaled_x = (data->raw_x * 0.0039f)-data->offset_x;
    data->scaled_y = (data->raw_y * 0.0039f)-data->offset_y;
    data->scaled_z = (data->raw_z * 0.0039f)-data->offset_z;
    
}

void Read_Sensor(Gyro_Data *data) {
    uint8_t raw_data[6];

  Gyro_SPI_ReadMulti(GYRO_OUT_X_L, raw_data, 6);
    
    // OUT_* order is LOW then HIGH byte.
    data->raw_x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    data->raw_y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    data->raw_z = (int16_t)((raw_data[5] << 8) | raw_data[4]);
    
    // Scale to angular velocity (deg/s) - I3G4250D at 250 dps: 8.75 mdps/LSB
    data->scaled_x = data->raw_x *0.00875 - data->offset_x;
    data->scaled_y = data->raw_y *0.00875 - data->offset_y;
    data->scaled_z = data->raw_z *0.00875 - data->offset_z;
    
}
void Offset_LSM(LSM303AGR_Data *data) {
  int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint8_t raw_data[6];
    
    for(int i = 0; i < 20; i++) {
        // Read 6 bytes starting from OUT_X_L_A (0x28). 
        // Bit 7 is set (0x80) to automatically increment the register address.
        HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_ADDR, 0x28 | 0x80, I2C_MEMADD_SIZE_8BIT, raw_data, 6, HAL_MAX_DELAY);
        
        sum_x += ((int16_t)((raw_data[1] << 8) | raw_data[0])) >> 6;
        sum_y += ((int16_t)((raw_data[3] << 8) | raw_data[2])) >> 6;
        sum_z += ((int16_t)((raw_data[5] << 8) | raw_data[4])) >> 6;
        HAL_Delay(10);
    }
    
    // Calculate average and convert to g units (3.9 mg/LSB)
    data->offset_x = (sum_x / 20.0f) * 0.0039f;
    data->offset_y = (sum_y / 20.0f) * 0.0039f;
    data->offset_z = (sum_z / 20.0f) * 0.0039f;
}

void Offset_Sensor(Gyro_Data *data) {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint8_t raw_data[6];
    
    
    for(int i = 0; i < 20; i++) {
      Gyro_SPI_ReadMulti(GYRO_OUT_X_L, raw_data, 6);
        
        sum_x += (int16_t)((raw_data[1] << 8) | raw_data[0]);
        sum_y += (int16_t)((raw_data[3] << 8) | raw_data[2]);
        sum_z += (int16_t)((raw_data[5] << 8) | raw_data[4]);
        HAL_Delay(10);
    }
    
    // Calculate average offsets in deg/s
    data->offset_x = (sum_x / 20.0f) * 0.00875;
    data->offset_y = (sum_y / 20.0f) * 0.00875;
    data->offset_z = (sum_z / 20.0f) * 0.00875;
}

  void Print_LSM(const LSM303AGR_Data *acc, const Gyro_Data *gyro)
  {
    float angle_x_deg = acc->scaled_x * RAD_TO_DEG;

    myPrintf("%.3f,%.3f,%.3f\r\n",
         acc->scaled_x, gyro->scaled_x, angle_x_deg);
  }

void myPrintf (const char *fmt , ...){
      char buffer[256];
      va_list args;
      va_start (args, fmt);
      int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
      va_end (args);
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
  }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2C_Init(&hi2c1);
  Init_LSM();
  Init_Sensor();
  HAL_Delay(200);
  
  // Calibrate sensors (call only once at startup)
  Offset_LSM(&acc_data);
  Offset_Sensor(&gyro_data);
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Read_LSM(&acc_data);
    Read_Sensor(&gyro_data);
    Print_LSM(&acc_data, &gyro_data);
    HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
