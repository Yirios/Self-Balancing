/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "sys.h"
/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN Private defines */
#define    ELE_ADC_L_CHANNEL					 ADC_Channel_4
#define    ELE_ADC_M_CHANNEL					 ADC_Channel_5
#define    ELE_ADC_R_CHANNEL					 ADC_Channel_15
#define    CCD_ADC_CHANNEL					 	 ADC_Channel_15
#define Battery_Ch 11
#define Angle_Ch 15
/* USER CODE END Private defines */

void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */
u16 Get_Adc_Average(u8 ch,u8 times);
u16 Get_Adc(u8 ch);
int Get_battery_volt(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

