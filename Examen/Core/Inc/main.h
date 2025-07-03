/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define alimentacion0_Pin GPIO_PIN_13
#define alimentacion0_GPIO_Port GPIOC
#define blinky_Pin GPIO_PIN_1
#define blinky_GPIO_Port GPIOH
#define disminuirTasaRefresco_Pin GPIO_PIN_2
#define disminuirTasaRefresco_GPIO_Port GPIOC
#define disminuirTasaRefresco_EXTI_IRQn EXTI2_IRQn
#define CLK_Pin GPIO_PIN_3
#define CLK_GPIO_Port GPIOC
#define VRY_Pin GPIO_PIN_0
#define VRY_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define VRX_Pin GPIO_PIN_4
#define VRX_GPIO_Port GPIOA
#define aumentarTasaRefresco_Pin GPIO_PIN_5
#define aumentarTasaRefresco_GPIO_Port GPIOA
#define aumentarTasaRefresco_EXTI_IRQn EXTI9_5_IRQn
#define ledRojo_Pin GPIO_PIN_6
#define ledRojo_GPIO_Port GPIOA
#define ledVerde_Pin GPIO_PIN_7
#define ledVerde_GPIO_Port GPIOA
#define alimentacion1_Pin GPIO_PIN_5
#define alimentacion1_GPIO_Port GPIOC
#define segmento11_Pin GPIO_PIN_12
#define segmento11_GPIO_Port GPIOB
#define alimentacion2_Pin GPIO_PIN_6
#define alimentacion2_GPIO_Port GPIOC
#define segmento7_Pin GPIO_PIN_8
#define segmento7_GPIO_Port GPIOC
#define DT_Pin GPIO_PIN_9
#define DT_GPIO_Port GPIOC
#define DT_EXTI_IRQn EXTI9_5_IRQn
#define alimentacion3_Pin GPIO_PIN_11
#define alimentacion3_GPIO_Port GPIOA
#define segmento10_Pin GPIO_PIN_12
#define segmento10_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define segmento2_Pin GPIO_PIN_10
#define segmento2_GPIO_Port GPIOC
#define segmento3_Pin GPIO_PIN_11
#define segmento3_GPIO_Port GPIOC
#define segmento4_Pin GPIO_PIN_12
#define segmento4_GPIO_Port GPIOC
#define segmento1_Pin GPIO_PIN_2
#define segmento1_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define segmento5_Pin GPIO_PIN_7
#define segmento5_GPIO_Port GPIOB
#define SW_Pin GPIO_PIN_8
#define SW_GPIO_Port GPIOB
#define SW_EXTI_IRQn EXTI9_5_IRQn
#define ledAzul_Pin GPIO_PIN_9
#define ledAzul_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
