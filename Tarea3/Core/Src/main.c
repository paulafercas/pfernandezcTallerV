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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Definimos la variable que nos indica el tamaño de los buffer recibidos
#define UART_RX_BUFFER_SIZE 64
//Definimos el tamaño del buffer donde se va a almacenar el menu
#define MENU_BUFFER_SIZE 1150
//Definimos el tamaño del buffer donde almacenaremos los valores de la señal
#define ADC_BUFFER_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
//Inicializamos la variable que almacena el estado actual de la maquina
estadoActual fsm ={0};
//Definimos una tabla con los comandos y funciones posibles
const comando_t tablaComandos[]={
		{"blinky", 	frecBlinky},
		{"led", 			ledRGB},
		{"muestreo", tiempoMuestreo},
		{"tamanoFFT",		tamanoFFT},
		{"señalADC", imprimirADC},
		{"equipo", imprimirConf},
		{"espectroFFT", imprimirFFT},
		{"help", help},
};
//Guardamos el numero de comandos en una variable
const int numeroComandos = sizeof (tablaComandos)/sizeof (comando_t);

//Creamos las variables donde se van a guardar los buffer utilizando
//el metodo ping pong
typedef enum{
	bufferA,
	bufferB
}bufferActivo;

//Creamos los buffer a y b
uint8_t rx_buffer_a [UART_RX_BUFFER_SIZE]={0};
uint8_t rx_buffer_b [UART_RX_BUFFER_SIZE]={0};

//Creamos una estructura donde se almacena los datos correspondientes al
//buffer activo
typedef struct{
	uint8_t* buffer;
	uint16_t size;
}DataPacket;

//Inicializamos la estructura DataPacket con el buffer vacío y el tamaño
//igual a 0
volatile DataPacket data_ready_packet ={.buffer= NULL, .size=0};
//Inicializamos la variable del buffer activo con el buffer a
volatile bufferActivo dma_buffer_activo = bufferA;

//Creamos la variable donde almacenaremos el menu inicial
char menu_display_buffer[MENU_BUFFER_SIZE];

//Inicializamos la variable que guarda el periodo del blinky
int periodo_ms =250;
//Inicializamos la variable donde almacenaremos el periodo del Timer3
int periodoTimer3 = 362;
//Inicializamos la variable donde guardaremos el valor de la frecuencia de muestreo
float frecMuestreo =44.1;
//Inicializamos la variable donde almacenaremos el tamaño de la FFT
int puntosFFT = 1024;
//Creamos el arreglo donde se van a almacenar los valores de la señal
volatile uint16_t adc_dma_buffer [ADC_BUFFER_SIZE];

//Bandera para avisar que el buffer de ADC está listo
uint8_t adc_data_ready = 0;

//UART/DMA externos
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

//Extern ADC y Timer
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
//Funcion que analiza el comando recibido
void analizarComando (uint8_t* buffer, uint16_t size);
//Funcion que despacha el comando
void despacharComando (comandoID_t id, char* comando, char* params);
//Funcion para encontrar el comando que el usuario envió
comandoID_t encontrarComandoid (const char* comando_str);
//Funcion para imprimir el menu de opciones
void menuComandos (char* params);

//Funciones dentro del despacho de comandos
//Funcion para cambiar el periodo del blinky
void periodoBlinky (char* params);
//Funcion para encender o apagar un led del RGB
void estadoRGB (char* params);
//Funcion para configurar el tiempo de muestreo
void configurarMuestreo(char* params);
//Funcion para configurar el tamaño de la FFT
void configurarTamanoFFT (char* params);
//Funcion para imprimir la señal ADC
void printADC(void);
//Funcion para imprimir las configuraciones del equipo
void printConf(void);
//Funcion para imprimir la FFT
void printFFT(void);
//Funcion para imprimir los valores mas importantes de la FFT
void printImportantes(void);

//Funcion maquina de estados
void maquinaEstados (void);

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
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //Inicializamos el timer 2 y sus interrupciones
  HAL_TIM_Base_Start_IT(&htim2);
  //Inicializamos el Timer3 (sin interrupciones)
  HAL_TIM_Base_Start(&htim3);
  //Inicializamos el PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  //Imprimimos el menu de comandos
  menuComandos(menu_display_buffer);
  //Inicializamos el buffer activo
  dma_buffer_activo= bufferA;
  //Comenzamos la recepcion del comando
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer_a, UART_RX_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(fsm.estado != IDLE){
		  maquinaEstados();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 362;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(blinky_GPIO_Port, blinky_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LedRojo_Pin|LedVerde_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Ledazul_GPIO_Port, Ledazul_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : blinky_Pin */
  GPIO_InitStruct.Pin = blinky_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(blinky_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LedRojo_Pin LedVerde_Pin */
  GPIO_InitStruct.Pin = LedRojo_Pin|LedVerde_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Ledazul_Pin */
  GPIO_InitStruct.Pin = Ledazul_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(Ledazul_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Funcion de maquinaEstados
void maquinaEstados (void){
	switch (fsm.estado){
	case blinky:{
        // Hacemos un Toggle al blinky
        HAL_GPIO_TogglePin(blinky_GPIO_Port, blinky_Pin);
        //Volvemos al estado de recibir mensaje
        fsm.estado = mensaje;
		break;
	}
	case mensaje:{
		  //Nos seguramos de que el buffer tenga un tamaño distinto de 0
		  if (data_ready_packet.size >0){
			  //Creamos una variable auxiliar donde guardamos el valor del buffer
			  uint8_t* proc_buffer = 0;
			  //Creamos una variable auxiliar donde guardamos el valor del tamaño del buffer
			  uint16_t proc_size=0;
			  //Asignamos los valores correspondientes a cada variable
			  proc_buffer = data_ready_packet.buffer;
			  proc_size = data_ready_packet.size;

			  //Borramos lo guardado en las variables globales para que no se acumulen los datos recibidos
			  data_ready_packet.buffer = NULL;
			  data_ready_packet.size =0;

			  //Procesamos los datos usando estas copias locales de forma segura
			  analizarComando(proc_buffer, proc_size);
		  }
		break;
	}
	case IDLE:{
		break;
	}
	default:{
		__NOP();
		break;
	}
	}
}
//Funcion para imprimir el menu inicial
void menuComandos (char* params){
	//Limpiamos el buffer
	memset (menu_display_buffer, 0, MENU_BUFFER_SIZE);
	//Creamos el string para el menu
	int offset =0;
	// Header
	    offset += snprintf(menu_display_buffer + offset, MENU_BUFFER_SIZE - offset,
	                       "| %-15s| %-65s| %-24s|\r\n", "Comando", "Descripcion", "Formato");

	    // Comandos con sus respectivas descripciones
	    offset += snprintf(menu_display_buffer + offset, MENU_BUFFER_SIZE - offset,
	                       "| %-15s| %-65s| %-24s|\r\n", "led", "Controla los led RGB con la funcion Toggle", "led <color>");
	    offset += snprintf(menu_display_buffer + offset, MENU_BUFFER_SIZE - offset,
	                       "| %-15s| %-65s| %-24s|\r\n", "blinky", "Configura el periodo del blinky", "blinky <periodo en ms>");
	    offset += snprintf(menu_display_buffer + offset, MENU_BUFFER_SIZE - offset,
	                       "| %-15s| %-65s| %-24s|\r\n", "muestreo", "Configura el tiempo de muestreo (44.1kHz,48kHz, 96kHz, 128kHz)", "muestreo <valor>");
	    offset += snprintf(menu_display_buffer + offset, MENU_BUFFER_SIZE - offset,
	                       "| %-16s| %-66s| %-25s|\r\n", "tamañoFFT", "Configura el tamaño de la FFT (1024, 2048)", "tamanoFFT <tamaño>");
	    offset += snprintf(menu_display_buffer + offset, MENU_BUFFER_SIZE - offset,
	                       "| %-16s| %-66s| %-25s|\r\n", "señalADC", "Imprime la señal ADC muestreada", "señalADC");
	    offset += snprintf(menu_display_buffer + offset, MENU_BUFFER_SIZE - offset,
	                       "| %-15s| %-65s| %-24s|\r\n", "equipo", "Imprime la configuracion del equipo", "equipo");
	    offset += snprintf(menu_display_buffer + offset, MENU_BUFFER_SIZE - offset,
	                       "| %-15s| %-66s| %-24s|\r\n", "espectroFFT", "Imprime el espectro FFT de la señal", "espectroFFT");
	    offset += snprintf(menu_display_buffer + offset, MENU_BUFFER_SIZE - offset,
	                       "| %-15s| %-66s| %-24s|\r\n", "principalFFT", "Imprime los valores más importantes de la FFT", "principalFFT");
	    offset += snprintf(menu_display_buffer + offset, MENU_BUFFER_SIZE - offset,
	                       "| %-15s| %-65s| %-24s|\r\n", "help", "Imprime el menu de comandos", "help");


	    // Transmitimos el menu a traves de la DMA
	    //Nos aseguramos de que no estemos transmitiendo otro buffer a traves de la DMA
	    if (huart2.gState == HAL_UART_STATE_READY) {
	    	//Transmitimos el menu
	        HAL_UART_Transmit_DMA(&huart2, (uint8_t*)menu_display_buffer, strlen(menu_display_buffer));
	    } else {
	        //Creamos un char donde almacenamos el mensaje de buffer ocupado
	        char busy_msg[] = "UART ocupado. Intenta 'help'.\r\n";
	        HAL_UART_Transmit_DMA(&huart2, (uint8_t*)busy_msg, strlen(busy_msg));
	    }

}
//Funcion para analizar el comando recibido
void analizarComando (uint8_t* buffer, uint16_t size){
	//Nos preguntamos si el tamaño de la entrada es mayor al asignado inicialmente
	if (size >= UART_RX_BUFFER_SIZE){
		//Nos aseguramos de que el ultimo caracter sea el nulo
		buffer [UART_RX_BUFFER_SIZE-1]='\0';
	} else{
		//Nos aseguramos de que el ultimo caracter sea nulo
		buffer[size]='\0';
	}
	//Usamos strok para extraer el comando y los parametros
	//obtenemos el comando
	char* comando_str = strtok ((char*)buffer, " ");
	//Obtenemos el resto de strings como parametros
	char* params = strtok  (NULL, " ");
	//Preguntamos si tenemos parametros para aquellas entradas sin parametros
	if (params == NULL){
		//Limpiamos el comando_str en caso de que tenga \r y \n
		comando_str[strcspn(comando_str, "\r\n")] = 0;
	}

	//Preguntamos si el comando esta vacio no hacemos nada
	if (comando_str==NULL){
		return;
	}

	//Debemos identificar el comando que estamos utilizando
	comandoID_t id =encontrarComandoid (comando_str);
	//Enviamos el comando a la maquina de estados
	despacharComando(id, comando_str, params);
}

//Funcion para encontrar el comando recibido
comandoID_t encontrarComandoid (const char* comando_str){
	for (int i =0; i< numeroComandos; i++){
		//Utilizamos la funcion strcmp que compara bit a bit el comando
		//ingresado con cada uno de los disponibles en la tabla
		if (strcmp (comando_str, tablaComandos[i].comando_str)==0){
			return tablaComandos[i].comando_id;
		}
	}
	//Si no coincide con ninguno entonces se retorna el estado de
	//comandoDesconocido
	return comandoDesconocido;
}
//Funcion para enviar el comando
void despacharComando (comandoID_t id, char* comando, char* params){
	switch (id){
	case frecBlinky:{
		periodoBlinky(params);
		break;
	}
	case ledRGB:{
		estadoRGB (params);
		break;
	}
	case tiempoMuestreo:{
		configurarMuestreo(params);
		break;
	}
	case tamanoFFT:{
		configurarTamanoFFT (params);
		break;
	}
	case imprimirADC:{
		printADC();
		break;
	}
	case imprimirConf:{
		printConf();
		break;
	}
	case imprimirFFT:{
		printFFT();
		break;
	}
	case importantesFFT:{
		printImportantes();
		break;
	}
	case help:{
		menuComandos (NULL);
		break;
	}
	case comandoDesconocido:{
        //Creamos un char donde almacenamos el mensaje de comando incorrecto
        char incorrect_msg[] = "Comando desconocido \r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)incorrect_msg, strlen(incorrect_msg), 1000);
		break;
	}
	default:{
		__NOP();
		break;
	}
	}
}
//Funcion para configurar el periodo del blinky
void periodoBlinky (char* params){
	/*char debug_buffer[64];
	snprintf(debug_buffer, sizeof(debug_buffer), "DEBUG param: '%s'\r\n", params);
	HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);*/
	//Creamos un buffer auxiliar
	char tx_buffer[80]={0};
	//Preguntamos si los parametros estan nulos
	if (params ==NULL){
		//Guardamos en nuestro nuevo buffer el mensaje de que no hay periodo
		sprintf (tx_buffer,"No se encontró periodo para el blinky, Escribe 'blinky <periodo en ms>'.\r\n");
		//Transmitimos el mensaje
		HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 1000);
		return;
	}
	//Convertimos el periodo dado por el usuario en un numero entero
	periodo_ms = atoi (params);
	//Preguntamos si el valor del periodo es menor o igual que 0
	if (periodo_ms <=0){
		//Guardamos el mensaje de periodo invalido en el buffer auxiliar
		sprintf (tx_buffer, "Periodo inválido. Debe ser mayor que 0\r\n");
		HAL_UART_Transmit(&huart2,(uint8_t *) tx_buffer, strlen(tx_buffer), 1000);
		return;
	}
	//Apagamos del TIMER2
	HAL_TIM_Base_Stop_IT(&htim2);
	//Modificamos el periodo del Timer
	htim2.Init.Period = periodo_ms;
	//Guardamos las nuevas configuraciones
	HAL_TIM_Base_Init(&htim2);
	// Encendemos nuevamente el Timer
	HAL_TIM_Base_Start_IT(&htim2);

}
//Funcion para encender o apagar un led del RGB
void estadoRGB (char* params){
	//Creamos un buffer para transmitir los mensajes en caso de error
	char tx_buffer[65];
	//Preguntamos si params es nulo
	if (params ==NULL){
		//Mensaje para decirle al usuario que la forma de su comando está incorrecta
		sprintf(tx_buffer, "No se encontraron los parámetros. Escribe 'led <color>'");
		//Imprimimos el mensaje
		HAL_UART_Transmit(&huart2, (uint8_t*) tx_buffer, strlen(tx_buffer), 1000);
		return;
	}
	//Creamos una variable auxiliar donde almacenamos el color que escribio el usuario
	char* color_str = strtok ((char*)params, "\r\n");
	//Preguntamos si color_str es igual a alguno de los colores disponibles
	if (strcmp (color_str, "rojo")==0){
		//Cuando es igual a rojo entonces invertimos el estado del led rojo
		HAL_GPIO_TogglePin(LedRojo_GPIO_Port, LedRojo_Pin);
	}
	else if (strcmp(color_str, "azul")==0){
		//Cuando e sigual a azul invertimos el estado del led azul
		HAL_GPIO_TogglePin(Ledazul_GPIO_Port, Ledazul_Pin);
	}
	else if (strcmp(color_str, "verde")==0){
		//Cuando es igual a verde invertimos el estado del led verde
		HAL_GPIO_TogglePin(LedVerde_GPIO_Port, LedVerde_Pin);
	}
	else {
		//Si lo escrito no coincide con ningun color entonces imprimimos el mensaje de error
		sprintf(tx_buffer, "Color no reconocido\r\n");
		//Imprimimos el mensaje
		HAL_UART_Transmit(&huart2, (uint8_t*) tx_buffer, strlen(tx_buffer), 1000);
	}


}
//Funcion para configurar el tiempo de muestreo
void configurarMuestreo(char* params){
	//Asignamos una posicion de memoria al caracter que no se pueda convertir a float
	char *endptr;
	//Le asignamos un valor nulo a esa posicion de memoria
	endptr = NULL;
	//Convertimos el parametro a un float
	frecMuestreo = strtof (params, &endptr);
	//Hallamos el valor del periodo que debe tener el timer3 para esa frecuencia de muestreo
	periodoTimer3 = 16000/frecMuestreo;
	//Apagamos del TIMER3
	HAL_TIM_Base_Stop(&htim3);
	//Modificamos el periodo del Timer
	htim2.Init.Period = periodoTimer3;
	//Guardamos las nuevas configuraciones
	HAL_TIM_Base_Init(&htim3);
	// Encendemos nuevamente el Timer
	HAL_TIM_Base_Start_IT(&htim3);
}

//Funcion para configurar el tamaño de la FFT
void configurarTamanoFFT (char* params){
	puntosFFT = atoi(params);

}
//Funcion para imprimir la señal ADC
void printADC(void){
	 HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_BUFFER_SIZE);

}
//Funcion para imprimir las configuraciones del equipo
void printConf(void){

}
//Funcion para imprimir la FFT
void printFFT(void){

}
//Funcion para imprimir los valores mas importantes de la FFT
void printImportantes(void){

}
//Llamamos al callback para la transmision por DMA
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	//Nos aseguramos de que es el USART2 el que esta haciendo la interrupcion
	if (huart->Instance == USART2){
	}
}
//Llamamos al callback para la recepcion del comando
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	//Nos aseguramos de que estamos usando el USART2
	if (huart->Instance ==USART2){
		//Preguntamos si estamos en el bufferA
		if (dma_buffer_activo==bufferA){
			//Guardamos el buffer recibido en el buffer_a
			data_ready_packet.buffer = rx_buffer_a;
			//Guardamos el tamaño del buffer
			data_ready_packet.size = Size;
			//Pasamos al buffer B
			dma_buffer_activo= bufferB;
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer_b, UART_RX_BUFFER_SIZE);
		}
		//Si el buffer se encuentra en el bufferb
		else {
			//Guardamos el buffer recibido en el buffer_a
			data_ready_packet.buffer = rx_buffer_b;
			//Guardamos el tamaño del buffer
			data_ready_packet.size = Size;
			//Pasamos al buffer A
			dma_buffer_activo= bufferA;
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer_a, UART_RX_BUFFER_SIZE);
		}
		//Cambiamos de estado a recibir mensaje
		fsm.estado = mensaje;
	}
}
//Llamamos el callback para el blinky
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	// Nos aseguramos de que sea el TIMER2 el que haga la interrupcion
    if (htim->Instance == TIM2){
    	//Cambiamos de estado a blinky
    	fsm.estado = blinky;
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//Activamos la bandera que nos dice que el buffer esta lleno
	//adc_data_ready== 1;
}
//Llamamos la funcion ErrorCallback en caso de que suceda un error
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    // Por ejemplo: reiniciar la recepción si hubo error
    if (huart->Instance == USART2) {
        if (dma_buffer_activo == bufferA) {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer_a, UART_RX_BUFFER_SIZE);
        } else {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer_b, UART_RX_BUFFER_SIZE);
        }
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
