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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//Enumeracion y estructura de los estados de la maquina
typedef enum{
	audio1,
	audio2,
	audio3,
	capitulo1,
	capitulo2,
	capitulo3,
	mensaje,
	Blinky,
	IDLE
}posiblesEstados;

typedef struct
{
	posiblesEstados estado;
}estadoActual;

//ENumeracion y estructura de los posibles comandos
typedef enum{
	cap1, //Cambia el valor del Duty
	cap2, //Muestra los ciclos utilizados por el MCU para mostrar el digito
	cap3, //Muestra los ciclos utilizados por el MCU para mostrar el mensaje
	help,			// Imprimir el menu de configuraciones
	comandoDesconocido //El comando que se ingresó no es válido
}comandoID_t;

//Definimos la estructura donde se hace llamada a los posibles estados de la
//enumeracion
typedef struct{
	const char* comando_str;
	comandoID_t comando_id;
}comando_t;
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
#define enable_Pin GPIO_PIN_13
#define enable_GPIO_Port GPIOC
#define blinky_Pin GPIO_PIN_1
#define blinky_GPIO_Port GPIOH
#define DIR_Pin GPIO_PIN_2
#define DIR_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define finalIzquierdo_Pin GPIO_PIN_5
#define finalIzquierdo_GPIO_Port GPIOA
#define finalIzquierdo_EXTI_IRQn EXTI9_5_IRQn
#define finalDerecho_Pin GPIO_PIN_6
#define finalDerecho_GPIO_Port GPIOA
#define finalDerecho_EXTI_IRQn EXTI9_5_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define STEP_Pin GPIO_PIN_7
#define STEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
