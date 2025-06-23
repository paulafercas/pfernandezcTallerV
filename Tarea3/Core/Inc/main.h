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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//Definimos los estados que puede tener la maquina de estados
typedef enum{
	frecBlinky, //Cambiar la frecuencia del Bliny
	ledRGB,		//Encender o apagar un led del RGB
	tiempoMuestreo, //Configurar el tiempo de muestreo
	tamanoFFT,		//Configurar el tamaño de la FFT
	imprimirADC,	//Imprimir señal ADC
	imprimirConf,	//Imprimir configuracion del equipo
	imprimirFFT, 	//Imprimir el espectro FFT
	importantesFFT, // Imprimir solo los valores mas importantes
	help,			// Imprimir el menu de configuraciones
	comandoDesconocido //El comando que se ingresó no es válido
}comandoID_t;

//Definimos la estructura donde se hace llamada a los posibles estados de la
//enumeracion
typedef struct{
	const char* comando_str;
	comandoID_t comando_id;
}comando_t;

//D


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
#define blinky_Pin GPIO_PIN_1
#define blinky_GPIO_Port GPIOH
#define UsartTX_Pin GPIO_PIN_2
#define UsartTX_GPIO_Port GPIOA
#define UsartRX_Pin GPIO_PIN_3
#define UsartRX_GPIO_Port GPIOA
#define LedRojo_Pin GPIO_PIN_6
#define LedRojo_GPIO_Port GPIOA
#define LedVerde_Pin GPIO_PIN_7
#define LedVerde_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Ledazul_Pin GPIO_PIN_9
#define Ledazul_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
