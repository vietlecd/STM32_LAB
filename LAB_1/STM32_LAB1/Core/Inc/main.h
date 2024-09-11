/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define led_red_vert_Pin GPIO_PIN_5
#define led_red_vert_GPIO_Port GPIOA
#define led_yellow_vert_Pin GPIO_PIN_6
#define led_yellow_vert_GPIO_Port GPIOA
#define led_green_vert_Pin GPIO_PIN_7
#define led_green_vert_GPIO_Port GPIOA
#define a_v_seg_Pin GPIO_PIN_0
#define a_v_seg_GPIO_Port GPIOB
#define b_v_seg_Pin GPIO_PIN_1
#define b_v_seg_GPIO_Port GPIOB
#define c_v_seg_Pin GPIO_PIN_2
#define c_v_seg_GPIO_Port GPIOB
#define d_h_seg_Pin GPIO_PIN_10
#define d_h_seg_GPIO_Port GPIOB
#define e_h_seg_Pin GPIO_PIN_11
#define e_h_seg_GPIO_Port GPIOB
#define f_h_seg_Pin GPIO_PIN_12
#define f_h_seg_GPIO_Port GPIOB
#define g_h_seg_Pin GPIO_PIN_13
#define g_h_seg_GPIO_Port GPIOB
#define led_red_hori_Pin GPIO_PIN_8
#define led_red_hori_GPIO_Port GPIOA
#define led_yellow_hori_Pin GPIO_PIN_9
#define led_yellow_hori_GPIO_Port GPIOA
#define led_green_hori_Pin GPIO_PIN_10
#define led_green_hori_GPIO_Port GPIOA
#define d_v_seg_Pin GPIO_PIN_3
#define d_v_seg_GPIO_Port GPIOB
#define e_v_seg_Pin GPIO_PIN_4
#define e_v_seg_GPIO_Port GPIOB
#define f_v_seg_Pin GPIO_PIN_5
#define f_v_seg_GPIO_Port GPIOB
#define g_v_seg_Pin GPIO_PIN_6
#define g_v_seg_GPIO_Port GPIOB
#define a_h_seg_Pin GPIO_PIN_7
#define a_h_seg_GPIO_Port GPIOB
#define b_h_seg_Pin GPIO_PIN_8
#define b_h_seg_GPIO_Port GPIOB
#define c_h_seg_Pin GPIO_PIN_9
#define c_h_seg_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
