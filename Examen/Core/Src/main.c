/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Definimos una enumeracion con los diferentes estados que puede tener la maquina
typedef enum{
	refrescar,
	cambiar_numero,
	aumentar_tasa_refresco,
	disminuir_tasa_refresco,
	resetear,
	Blinky,
	IDLE
}posiblesEstados;

typedef struct
{
	posiblesEstados estado;
}estadoActual;


//Enumeramos las posibles partes que tenemos en numeroDisplay
typedef enum{
	unidad1 =0,
	decena1,
	centena1,
	milUnidad1,
}parteNumero;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Inicializamos la variable a la cual vamos a gurdarle el estado que
//tiene la maquina
estadoActual fsm ={0};

//Creamos la variable donde vamos a guardar el numero que se va a mostrar en el display
uint16_t numeroDisplay =0;

//Inicializamos las variables que van a permitir separar el numeroDisplay en 4 partes
uint8_t unidad = 0;
uint8_t decena = 0;
uint8_t centena = 0;
uint8_t milUnidad = 0;


//Inicializamos la variable donde guardamos que digito queremos encender (0,1,2,3)
uint8_t digito = 0;
//Inicializamos la variable que va a modificar la tasa de refresco
//Comenzamos con un periodo de 20ms
uint16_t tasa_refresco = 20;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/*
 * Declaramos la funciones que aparecerán próximamente
 */
//Funcion que define qué se va a hacer según el estado en el que se encuentre la maquina
void maquinaEstados(uint16_t numeroLocal,uint8_t digito);
//Funcion que separa el numeroDisplay en 4 partes (unidad, decena, centena, milUnidad)
uint16_t separacion_parte (parteNumero parte, uint16_t numeroDisplay);
//Funcion que nos permite encender el numero que queremos en el digito que queremos
void digito_encendido(uint8_t digito, uint8_t unidad,uint8_t decena, uint8_t centena, uint8_t milUnidad);
//Funcion que se encuentra dentro de "digito_encendido" para definir que numero
//quiero ver en cada digito
void definir_numero (uint8_t numero);
//Funcion para configurar los pines del 7 segmentos
void configurar7Segmentos();
//Funcion para configurar el pin del Blinky
void configurarTimers ();
//Funcion que configura los pines para el EXTI
void configurarExti ();
//Funcion para cambiar numero según el Encoder
uint16_t cambioNumero (uint16_t numeroDisplay);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  fsm.estado = refrescar;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //Activamos los timers y sus interrupciones
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  //Inicializamos el Timer1 y 4 (sin interrupciones)
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim4);
  //Inicializamos los PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(fsm.estado != IDLE){
		  maquinaEstados(numeroDisplay,digito);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  return 0;
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 80;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 250-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, alimentacion0_Pin|alimentacion1_Pin|alimentacion2_Pin|segmento7_Pin
                          |segmento2_Pin|segmento3_Pin|segmento4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(blinky_GPIO_Port, blinky_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, segmento11_Pin|segmento5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, alimentacion3_Pin|segmento10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(segmento1_GPIO_Port, segmento1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : alimentacion0_Pin */
  GPIO_InitStruct.Pin = alimentacion0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(alimentacion0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : blinky_Pin */
  GPIO_InitStruct.Pin = blinky_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(blinky_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : disminuirTasaRefresco_Pin DT_Pin */
  GPIO_InitStruct.Pin = disminuirTasaRefresco_Pin|DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_Pin */
  GPIO_InitStruct.Pin = CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : aumentarTasaRefresco_Pin */
  GPIO_InitStruct.Pin = aumentarTasaRefresco_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(aumentarTasaRefresco_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : alimentacion1_Pin alimentacion2_Pin segmento7_Pin segmento2_Pin
                           segmento3_Pin segmento4_Pin */
  GPIO_InitStruct.Pin = alimentacion1_Pin|alimentacion2_Pin|segmento7_Pin|segmento2_Pin
                          |segmento3_Pin|segmento4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : segmento11_Pin segmento5_Pin */
  GPIO_InitStruct.Pin = segmento11_Pin|segmento5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : alimentacion3_Pin segmento10_Pin */
  GPIO_InitStruct.Pin = alimentacion3_Pin|segmento10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : segmento1_Pin */
  GPIO_InitStruct.Pin = segmento1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(segmento1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint16_t separacion_parte (parteNumero parte, uint16_t numeroDisplay){
	switch (parte){
	case unidad1:{
		uint8_t unidad = 0;
		unidad = numeroDisplay%10;
		return unidad;
		break;
	}
	case decena1: {
		uint8_t unidad = 0;
		uint8_t decena = 0;
		unidad = numeroDisplay%10;
		decena = ((numeroDisplay-unidad)/10)%10;
		return decena;
		break;
	}
	case centena1:{
		uint8_t residuoCentena = 0;
		uint8_t centena = 0;
		residuoCentena = numeroDisplay%100;
		centena = ((numeroDisplay - residuoCentena)/100)%10;
		return centena;
		break;
	}
	case milUnidad1: {
		uint8_t residuoMil = 0;
		residuoMil = numeroDisplay%1000;
		uint8_t milUnidad = 0;
		milUnidad = (numeroDisplay-residuoMil)/1000;
		return milUnidad;
		break;
	}
	default:{
		return 0;
	}
	}//Fin del Switch case
}

/*
 * Creamos la funcion que le indica a la maquina de estados que debe hacer para cada caso
 */
void maquinaEstados(uint16_t numeroLocal, uint8_t digito){
	switch (fsm.estado){
	case refrescar:{
		//Apagamos todos los digitos
		HAL_GPIO_WritePin(alimentacion0_GPIO_Port, alimentacion0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(alimentacion1_GPIO_Port, alimentacion1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(alimentacion2_GPIO_Port, alimentacion2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(alimentacion3_GPIO_Port, alimentacion3_Pin, GPIO_PIN_SET);

		// Vamos a dividir el numeroDisplay en unidades, decenas, centenas y unidades de mil.
		unidad =  separacion_parte (unidad1, numeroDisplay);
		decena = separacion_parte (decena1, numeroDisplay);
		centena = separacion_parte (centena1, numeroDisplay);
		milUnidad = separacion_parte (milUnidad1, numeroDisplay);
		//Llamamos a la funcion que nos indica qué pines deben estar encendidos en el digito
		//que deseo mostrar
		digito_encendido(digito,unidad, decena, centena, milUnidad);
		break;
	}
	case cambiar_numero: {
		//Llamamos a la funcion encargada de cambiar el numero
		//debido a la interrupcion
		numeroDisplay = cambioNumero (numeroLocal);
		//Volvemos al estado refrescar
		fsm.estado = refrescar;
		break;
	}
	case aumentar_tasa_refresco:{
		// Apagamos el Timer.
		HAL_TIM_Base_Stop_IT(&htim3);
		//Nos aseguramos de que la tasa de refresco no vaya a sea menor a 1ms
		if (tasa_refresco<=1){
			tasa_refresco = 20;
		}
		//Disminuimos a la mitad ms la tasa de refresco
		else {
			tasa_refresco/=2;
		}

		//Configuramos el refresh Timer
		htim3.Init.Period = tasa_refresco;
		//Guardamos las nuevas configuraciones
		HAL_TIM_Base_Init(&htim3);
		// Encendemos nuevamente el Timer
		HAL_TIM_Base_Start_IT(&htim3);
		fsm.estado = refrescar;
		break;
	}
	case disminuir_tasa_refresco:{
		// Apagamos el Timer.
		HAL_TIM_Base_Stop_IT(&htim3);
		//Nos aseguramos de que la tasa de refresco no vaya a ser mayor de 1000 ms
		if (tasa_refresco>=1000){
			//Llevamos a la tasa de refresco a su estado inicial
			tasa_refresco =15;
		}
		else{
			//Aumentamos la tasa de refresco multiplicando por 2
			tasa_refresco*=2;
		}
		//Configuramos el refresh Timer
		htim3.Init.Period = tasa_refresco;
		//Guardamos las nuevas configuraciones
		HAL_TIM_Base_Init(&htim3);
		// Encendemos nuevamente el Timer
		HAL_TIM_Base_Start_IT(&htim3);
		//Volvemos al estado refrescar
		fsm.estado = refrescar;
		break;
	}
	case resetear:{
		//Con el SWITCH vamos a reiniciar nuestro contador
		numeroDisplay = 0;
		//Volvemos al estado refrescar
		fsm.estado = refrescar;
	}
	case Blinky:{
		//Encendemos o apagamos el led
		HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);
		//Volvemos al estado refrescar
		fsm.estado = refrescar;
	}
	case IDLE:{
		//En esta funcion no se hace absolutamente nada
		break;
	}
	default:{
		__NOP();
		break;
	}
	}//Fin del Switch case
}
/*
 * Funcion que determina qué digito tenemos encendido
 */
void digito_encendido(uint8_t digito, uint8_t unidad,uint8_t decena, uint8_t centena, uint8_t milUnidad){
	switch (digito){
	//Caso en el cual queremos ver el numero en el digito de las unidades
	case 0: {
		//Encendemos los pines correspondientes al numero que se desea
		definir_numero (unidad);
		//Encedemos el digito (1) donde se va a mostrar el numero seleccionado anteriormente
		HAL_GPIO_WritePin(alimentacion0_GPIO_Port, alimentacion0_Pin, GPIO_PIN_RESET);
		break;
	}
	case 1: {
		//Encendemos los pines correspondientes al numero que se desea
		definir_numero (decena);
		//Encedemos el digito (1) donde se va a mostrar el numero seleccionado anteriormente
		HAL_GPIO_WritePin(alimentacion1_GPIO_Port, alimentacion1_Pin, GPIO_PIN_RESET);
		break;
	}
	case 2:{
		//Encendemos los pines correspondientes al numero que se desea
		definir_numero (centena);
		//Encedemos el digito (1) donde se va a mostrar el numero seleccionado anteriormente
		HAL_GPIO_WritePin(alimentacion2_GPIO_Port, alimentacion2_Pin, GPIO_PIN_RESET);
		break;
	}
	case 3:{
		//Encendemos los pines correspondientes al numero que se desea
		definir_numero (milUnidad);
		//Encedemos el digito (1) donde se va a mostrar el numero seleccionado anteriormente
		HAL_GPIO_WritePin(alimentacion3_GPIO_Port, alimentacion3_Pin, GPIO_PIN_RESET);
		break;
	}
	default:{
		__NOP();
		break;
	}
	}// Fin del Switch-case
}
/*
 * Funcion que determina que numero vamos a mostrar
 */
void definir_numero (uint8_t numero){
	switch (numero){
	case 0: {
		HAL_GPIO_WritePin(segmento1_GPIO_Port, segmento1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento2_GPIO_Port, segmento2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento3_GPIO_Port, segmento3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento4_GPIO_Port, segmento4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento5_GPIO_Port, segmento5_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento7_GPIO_Port, segmento7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento10_GPIO_Port, segmento10_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento11_GPIO_Port, segmento11_Pin, GPIO_PIN_RESET);
		break;
	}
	case 1: {
		HAL_GPIO_WritePin(segmento1_GPIO_Port, segmento1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento2_GPIO_Port, segmento2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento3_GPIO_Port, segmento3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento4_GPIO_Port, segmento4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento5_GPIO_Port, segmento5_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento7_GPIO_Port, segmento7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento10_GPIO_Port, segmento10_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento11_GPIO_Port, segmento11_Pin, GPIO_PIN_SET);
		break;
	}
	case 2: {
		HAL_GPIO_WritePin(segmento1_GPIO_Port, segmento1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento2_GPIO_Port, segmento2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento3_GPIO_Port, segmento3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento4_GPIO_Port, segmento4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento5_GPIO_Port, segmento5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento7_GPIO_Port, segmento7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento10_GPIO_Port, segmento10_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento11_GPIO_Port, segmento11_Pin, GPIO_PIN_RESET);
		break;
	}
	case 3: {
		HAL_GPIO_WritePin(segmento1_GPIO_Port, segmento1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento2_GPIO_Port, segmento2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento3_GPIO_Port, segmento3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento4_GPIO_Port, segmento4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento5_GPIO_Port, segmento5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento7_GPIO_Port, segmento7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento10_GPIO_Port, segmento10_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento11_GPIO_Port, segmento11_Pin, GPIO_PIN_RESET);
		break;
	}
	case 4: {
		HAL_GPIO_WritePin(segmento1_GPIO_Port, segmento1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento2_GPIO_Port, segmento2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento3_GPIO_Port, segmento3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento4_GPIO_Port, segmento4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento5_GPIO_Port, segmento5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento7_GPIO_Port, segmento7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento10_GPIO_Port, segmento10_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento11_GPIO_Port, segmento11_Pin, GPIO_PIN_SET);
		break;
	}
	case 5: {
		HAL_GPIO_WritePin(segmento1_GPIO_Port, segmento1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento2_GPIO_Port, segmento2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento3_GPIO_Port, segmento3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento4_GPIO_Port, segmento4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento5_GPIO_Port, segmento5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento7_GPIO_Port, segmento7_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento10_GPIO_Port, segmento10_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento11_GPIO_Port, segmento11_Pin, GPIO_PIN_RESET);
		break;
	}
	case 6: {
		HAL_GPIO_WritePin(segmento1_GPIO_Port, segmento1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento2_GPIO_Port, segmento2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento3_GPIO_Port, segmento3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento4_GPIO_Port, segmento4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento5_GPIO_Port, segmento5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento7_GPIO_Port, segmento7_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento10_GPIO_Port, segmento10_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento11_GPIO_Port, segmento11_Pin, GPIO_PIN_RESET);
		break;
	}
	case 7: {
		HAL_GPIO_WritePin(segmento1_GPIO_Port, segmento1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento2_GPIO_Port, segmento2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento3_GPIO_Port, segmento3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento4_GPIO_Port, segmento4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento5_GPIO_Port, segmento5_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento7_GPIO_Port, segmento7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento10_GPIO_Port, segmento10_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento11_GPIO_Port, segmento11_Pin, GPIO_PIN_RESET);
		break;
	}
	case 8: {
		HAL_GPIO_WritePin(segmento1_GPIO_Port, segmento1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento2_GPIO_Port, segmento2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento3_GPIO_Port, segmento3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento4_GPIO_Port, segmento4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento5_GPIO_Port, segmento5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento7_GPIO_Port, segmento7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento10_GPIO_Port, segmento10_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento11_GPIO_Port, segmento11_Pin, GPIO_PIN_RESET);
		break;
	}
	case 9: {
		HAL_GPIO_WritePin(segmento1_GPIO_Port, segmento1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento2_GPIO_Port, segmento2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento3_GPIO_Port, segmento3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(segmento4_GPIO_Port, segmento4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento5_GPIO_Port, segmento5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento7_GPIO_Port, segmento7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento10_GPIO_Port, segmento10_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(segmento11_GPIO_Port, segmento11_Pin, GPIO_PIN_RESET);
		break;
	}
	default: {
		__NOP();
		break;
	}
	}// Fin del switch-case

}
uint16_t cambioNumero (uint16_t numeroLocal){
	//Creamos una variable en la cual guardamos el valor del pin
	//donde está el DT
	uint32_t valor_DT = 0;
	//Cargamos el valor del pin en la variable
	valor_DT = HAL_GPIO_ReadPin(CLK_GPIO_Port, CLK_Pin);
	//Comparamos las posibles opciones
	switch (valor_DT){
	//Cuando el pin DT está en o
	case 0: {
		//Nos aseguramos de que el numeroDisplay no vaya a ser mayor ue 4095
		if (numeroLocal>=4095){
			//Devolvemos el valor de numeroDisplay a 0
			numeroLocal=0;
			//Retornamos el valor numeroLocal
			return numeroLocal;
		}
		else {
			//Le sumamos una unidad a la variable numeroDisplay
			numeroLocal ++;
			//Retornamos el valor numeroLocal
			return numeroLocal;
		}
		break;
	}

	case 1: {
		//Nos aseguramos de que el numeroDisplay no vaya a ser menor que cero
		if (numeroLocal==0){
			//Lo devolvemos a 4095
			numeroLocal =4095;
			//Retornamos el valor numeroLocal
			return numeroLocal;
		}
		else {
			//Le restamos a numeroDisplay una unidad
			numeroLocal --;
			//Retornamos el valor numeroLocal
			return numeroLocal;
		}
		break;

	}
	default:{
		//Retornamos el valor numeroLocal
		return numeroLocal;
		break;
	}
	}
}
//Callback del timer2 para la funcion del blinky
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance ==TIM2){
		fsm.estado = Blinky;
	}
	else if (htim->Instance == TIM3){
		digito +=1;
		//Debemos cersiorarnos de que el digito que queremos encender no tenga un
		//valor mayor a 3 (ya que solo tenemos 4 digitos)
		if (digito >= 4){
			digito=0;
		}
	}

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == GPIO_PIN_9){
		//Cambiamos el estado a "cambiar numero"
		fsm.estado = cambiar_numero;
	}
	else if (GPIO_Pin == GPIO_PIN_5){
		//cambiamos el estado a "aumentar tasa de refresco"
		fsm.estado = aumentar_tasa_refresco;
	}
	else if (GPIO_Pin == GPIO_PIN_2){
		//Cambiamos el estado a "disminuir tasa de refresco"
		fsm.estado = disminuir_tasa_refresco;
	}
	else if (GPIO_Pin == GPIO_PIN_8){
		//Cambiamos el estado a resetear
		fsm.estado = resetear;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
