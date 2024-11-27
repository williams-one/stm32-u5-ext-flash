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
#include "stm32u5xx_hal.h"

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
#define LCD_ON_Pin GPIO_PIN_4
#define LCD_ON_GPIO_Port GPIOE
#define TP_IRQ_Pin GPIO_PIN_5
#define TP_IRQ_GPIO_Port GPIOE
#define TP_IRQ_EXTI_IRQn EXTI5_IRQn
#define BL_CTRL_Pin GPIO_PIN_6
#define BL_CTRL_GPIO_Port GPIOE
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define USB_DETECT_Pin GPIO_PIN_1
#define USB_DETECT_GPIO_Port GPIOA
#define USB_DETECT_EXTI_IRQn EXTI1_IRQn
#define USB_FAULT_Pin GPIO_PIN_4
#define USB_FAULT_GPIO_Port GPIOA
#define USB_FAULT_EXTI_IRQn EXTI4_IRQn
#define RED_LED_Pin GPIO_PIN_2
#define RED_LED_GPIO_Port GPIOD
#define GREEN_LED_Pin GPIO_PIN_4
#define GREEN_LED_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
