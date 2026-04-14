/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DRDY_Pin GPIO_PIN_2
#define DRDY_GPIO_Port GPIOE
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define MEMS_INT3_Pin GPIO_PIN_4
#define MEMS_INT3_GPIO_Port GPIOE
#define MEMS_INT4_Pin GPIO_PIN_5
#define MEMS_INT4_GPIO_Port GPIOE
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MISOA7_Pin GPIO_PIN_7
#define SPI1_MISOA7_GPIO_Port GPIOA
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOE
#define LD5_Pin GPIO_PIN_10
#define LD5_GPIO_Port GPIOE
#define LD7_Pin GPIO_PIN_11
#define LD7_GPIO_Port GPIOE
#define LD9_Pin GPIO_PIN_12
#define LD9_GPIO_Port GPIOE
#define LD10_Pin GPIO_PIN_13
#define LD10_GPIO_Port GPIOE
#define LD8_Pin GPIO_PIN_14
#define LD8_GPIO_Port GPIOE
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOE
#define DM_Pin GPIO_PIN_11
#define DM_GPIO_Port GPIOA
#define DP_Pin GPIO_PIN_12
#define DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define MEMS_INT1_Pin GPIO_PIN_0
#define MEMS_INT1_GPIO_Port GPIOE
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

// Motor shield mapping used in this project.
// Right motor direction: D8/D12, PWM: D10
// Left motor direction: D7/D6,  PWM: D9
#define RIGHT_MOTOR_IN1_Pin        GPIO_PIN_0   // Shield D8
#define RIGHT_MOTOR_IN1_GPIO_Port  GPIOC
#define RIGHT_MOTOR_IN2_Pin        GPIO_PIN_0   // Shield D12
#define RIGHT_MOTOR_IN2_GPIO_Port  GPIOD

#define LEFT_MOTOR_IN1_Pin         GPIO_PIN_1   // Shield D7
#define LEFT_MOTOR_IN1_GPIO_Port   GPIOC
#define LEFT_MOTOR_IN2_Pin         GPIO_PIN_1   // Shield D6
#define LEFT_MOTOR_IN2_GPIO_Port   GPIOD

#define RIGHT_MOTOR_PWM_Pin        GPIO_PIN_9   // Shield D10, TIM15_CH1
#define RIGHT_MOTOR_PWM_GPIO_Port  GPIOF
#define LEFT_MOTOR_PWM_Pin         GPIO_PIN_10  // Shield D9, TIM15_CH2
#define LEFT_MOTOR_PWM_GPIO_Port   GPIOF

// Quadrature encoder mapping.
// Right wheel: D4 (phase A) + D3 (phase B)
// Left wheel:  D5 (phase A) + D2 (phase B)
#define RIGHT_ENCODER_A_Pin        GPIO_PIN_6   // Shield D4, TIM3_CH1 on PC6
#define RIGHT_ENCODER_A_GPIO_Port  GPIOC
#define RIGHT_ENCODER_B_Pin        GPIO_PIN_7   // Shield D3, TIM3_CH2 on PC7
#define RIGHT_ENCODER_B_GPIO_Port  GPIOC

#define LEFT_ENCODER_A_Pin         GPIO_PIN_12  // Shield D5, TIM4_CH1 on PD12
#define LEFT_ENCODER_A_GPIO_Port   GPIOD
#define LEFT_ENCODER_B_Pin         GPIO_PIN_13  // Shield D2, TIM4_CH2 on PD13
#define LEFT_ENCODER_B_GPIO_Port   GPIOD

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
