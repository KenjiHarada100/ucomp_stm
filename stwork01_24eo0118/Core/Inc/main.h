/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define LCD_DB5_Pin GPIO_PIN_13
#define LCD_DB5_GPIO_Port GPIOC
#define LCD_DB6_Pin GPIO_PIN_14
#define LCD_DB6_GPIO_Port GPIOC
#define LCD_DB7_Pin GPIO_PIN_15
#define LCD_DB7_GPIO_Port GPIOC
#define Buzzer_Pin GPIO_PIN_4
#define Buzzer_GPIO_Port GPIOC
#define LED9_CLK_Pin GPIO_PIN_5
#define LED9_CLK_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_0
#define SW1_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_1
#define SW2_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_2
#define SW3_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_11
#define LED4_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_12
#define LED5_GPIO_Port GPIOB
#define LED6_Pin GPIO_PIN_13
#define LED6_GPIO_Port GPIOB
#define LED7_Pin GPIO_PIN_14
#define LED7_GPIO_Port GPIOB
#define LED8_Pin GPIO_PIN_15
#define LED8_GPIO_Port GPIOB
#define LED9_DATA_Pin GPIO_PIN_6
#define LED9_DATA_GPIO_Port GPIOC
#define LED9_SCAN_Pin GPIO_PIN_7
#define LED9_SCAN_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_8
#define LCD_RS_GPIO_Port GPIOC
#define LCD_RW_Pin GPIO_PIN_9
#define LCD_RW_GPIO_Port GPIOC
#define LCD_E_Pin GPIO_PIN_10
#define LCD_E_GPIO_Port GPIOC
#define LCD_DB4_Pin GPIO_PIN_12
#define LCD_DB4_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOD
#define SW4_Pin GPIO_PIN_3
#define SW4_GPIO_Port GPIOB
#define SW5_Pin GPIO_PIN_4
#define SW5_GPIO_Port GPIOB
#define SW6_Pin GPIO_PIN_5
#define SW6_GPIO_Port GPIOB
#define SW7_Pin GPIO_PIN_6
#define SW7_GPIO_Port GPIOB
#define SW8_Pin GPIO_PIN_7
#define SW8_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
