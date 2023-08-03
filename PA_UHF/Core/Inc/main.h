/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AGC_LEVEL_Pin GPIO_PIN_0
#define AGC_LEVEL_GPIO_Port GPIOA
#define CURR_Pin GPIO_PIN_1
#define CURR_GPIO_Port GPIOA
#define V_IN_SAMPLE_Pin GPIO_PIN_2
#define V_IN_SAMPLE_GPIO_Port GPIOA
#define PA_HAB_Pin GPIO_PIN_3
#define PA_HAB_GPIO_Port GPIOA
#define SWR_Pin GPIO_PIN_4
#define SWR_GPIO_Port GPIOA
#define P_OUT_Pin GPIO_PIN_5
#define P_OUT_GPIO_Port GPIOA
#define P_IN_Pin GPIO_PIN_6
#define P_IN_GPIO_Port GPIOA
#define LE_ATT_Pin GPIO_PIN_7
#define LE_ATT_GPIO_Port GPIOA
#define CLK_ATT_Pin GPIO_PIN_0
#define CLK_ATT_GPIO_Port GPIOB
#define DATA_ATT_Pin GPIO_PIN_1
#define DATA_ATT_GPIO_Port GPIOB
#define T_SN_IN_Pin GPIO_PIN_2
#define T_SN_IN_GPIO_Port GPIOB
#define TEMP_HIGH_Pin GPIO_PIN_8
#define TEMP_HIGH_GPIO_Port GPIOA
#define TEMP_OK_Pin GPIO_PIN_9
#define TEMP_OK_GPIO_Port GPIOA
#define KA_Pin GPIO_PIN_6
#define KA_GPIO_Port GPIOC
#define CURR_H_Pin GPIO_PIN_10
#define CURR_H_GPIO_Port GPIOA
#define CURR_N_Pin GPIO_PIN_11
#define CURR_N_GPIO_Port GPIOA
#define CURR_L_Pin GPIO_PIN_12
#define CURR_L_GPIO_Port GPIOA
#define DE_485_Pin GPIO_PIN_5
#define DE_485_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
