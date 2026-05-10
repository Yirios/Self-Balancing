/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY2_Pin GPIO_PIN_13
#define KEY2_GPIO_Port GPIOC
#define oled_SCL_Pin GPIO_PIN_14
#define oled_SCL_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOC
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define PS2_CS_Pin GPIO_PIN_4
#define PS2_CS_GPIO_Port GPIOC
#define IIC_SCL_Pin GPIO_PIN_14
#define IIC_SCL_GPIO_Port GPIOB
#define IIC_SDA_Pin GPIO_PIN_15
#define IIC_SDA_GPIO_Port GPIOB
#define PS2_CLK_Pin GPIO_PIN_8
#define PS2_CLK_GPIO_Port GPIOC
#define PS2_DO_Pin GPIO_PIN_9
#define PS2_DO_GPIO_Port GPIOC
#define BEEP_Pin GPIO_PIN_15
#define BEEP_GPIO_Port GPIOA
#define oled_DC_Pin GPIO_PIN_3
#define oled_DC_GPIO_Port GPIOB
#define oled_RST_Pin GPIO_PIN_4
#define oled_RST_GPIO_Port GPIOB
#define oled_SDA_Pin GPIO_PIN_5
#define oled_SDA_GPIO_Port GPIOB
#define PS2_DI_Pin GPIO_PIN_8
#define PS2_DI_GPIO_Port GPIOB
#define mpu6050_exit_Pin GPIO_PIN_9
#define mpu6050_exit_GPIO_Port GPIOB
#define mpu6050_exit_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
