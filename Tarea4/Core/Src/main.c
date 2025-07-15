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
// Dirección I2C con bit R/W en 0
#define MPU6050_ADDR (0x68 << 1)
// Primer registro de datos del acelerómetro
#define MPU6050_REG_ACCEL 0x3B
// Valor para suavizar el filtro de forma exponencial
#define ALPHA 0.1f
//Definimos el tamaño de la FFT
#define FFT_SIZE 256
 //Frecuencia de muestreo
#define SAMPLE_FREQ 1000.0f
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//Inicializamos la variable a la cual vamos a gurdarle el estado que
//tiene la maquina
estadoActual fsm ={0};


SD_MPU6050 mpu6050;
SD_MPU6050_Result result;
volatile bool newDataAvailable = false;

//Variables para las aceleraciones en cada eje
float aceleracionx =0;
float aceleraciony =0;
float aceleracionz =0;

//Variables para dar las aceleraciones filtradas
float accX_filt = 0;
float accY_filt = 0;
float accZ_filt = 0;

//Variables para expresar las aceleracion en terminos de
float g_x =0;
float g_y =0;
float g_z =0;

//Buffer para guardar en string los valores de las aceleraciones
char buffer[32];

char buf[32];
//Contador de lecturas para el acelerometro
static uint16_t contadorLecturas = 0;

//Variables para la fft
// Muestras en Z
float32_t accZ_buffer[FFT_SIZE];
uint16_t accZ_index = 0;
uint8_t bufferFull = 0;

float32_t fft_input[2 * FFT_SIZE];    // Interleaved real/imag para la FFT
float32_t fft_output[FFT_SIZE];       // Magnitudes
//Variable global para la frecuencia dominante
float freqDominante =0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

//Función para asignar cada uno de los posibles estados de la maquina
void maquinaEstados (void);
//Funcion para calcular la frecuencia dominante
void calcularFrecuenciaDominante(void);
//Funcion para mostrar los cuadros principales
void cuadrosPrincipales (void);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //Inicializamos el timer 2 para el blinky
  HAL_TIM_Base_Start_IT(&htim2);
  result = SD_MPU6050_Init(&hi2c1, &mpu6050, SD_MPU6050_Device_0,
                           SD_MPU6050_Accelerometer_2G,
                           SD_MPU6050_Gyroscope_250s);

  if (result == SD_MPU6050_Result_Ok) {
      SD_MPU6050_EnableInterrupts(&hi2c1, &mpu6050);
  } else {
	  Error_Handler();
  }

  //Inicializamos la pantalla
  ssd1306_Init();
  //Probamos en la pantalla
  //ssd1306_TestAll();

  //Inicializamos las etiquetas de la pantalla
  cuadrosPrincipales();


  //Inicializamos el estado de la maquina de estados
  fsm.estado = guardarDato;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  maquinaEstados();
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Period = 100-1;
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

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  HAL_GPIO_WritePin(ledPrueba_GPIO_Port, ledPrueba_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : blinky_Pin */
  GPIO_InitStruct.Pin = blinky_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(blinky_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ledPrueba_Pin */
  GPIO_InitStruct.Pin = ledPrueba_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ledPrueba_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : actualizacionDato_Pin */
  GPIO_InitStruct.Pin = actualizacionDato_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(actualizacionDato_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Funcion de la maquina de estados
void maquinaEstados(void){
	switch (fsm.estado){
	case guardarDato:{
	    SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu6050);
	    //Guardamos los datos en las variables de la aceleracion
	    aceleracionx = mpu6050.Accelerometer_X * mpu6050.Acce_Mult;
	    aceleraciony = mpu6050.Accelerometer_Y * mpu6050.Acce_Mult;
	    aceleracionz = mpu6050.Accelerometer_Z * mpu6050.Acce_Mult;

	    //Filtramos los valores
	    accX_filt = ALPHA * aceleracionx + (1.0f - ALPHA) * accX_filt;
	    accY_filt = ALPHA * aceleraciony + (1.0f - ALPHA) * accY_filt;
	    accZ_filt = ALPHA * aceleracionz + (1.0f - ALPHA) * accZ_filt;

	    //Transformamos los valores en términos de la gravedad g
		g_x = accX_filt * 9.65f;
		g_y = accY_filt * 9.65f;
		g_z = accZ_filt * 9.65f;

		//Guardamos los datos correspondientes a la aceleracion en z
		accZ_buffer[accZ_index++] = aceleracionz;
		//Nos aseguramos de que este arreglo no se vaya a desbordar
		if (accZ_index >= FFT_SIZE) {
		    accZ_index = 0;
		    bufferFull = 1;
		}

		contadorLecturas ++;
		if (contadorLecturas==200){
			contadorLecturas=0;
		}
		if (contadorLecturas==5){
			fsm.estado= mostrarPantalla;
		}
		else{
			fsm.estado =IDLE;
		}

    	break;
	}
	case mostrarPantalla:{
		calcularFrecuenciaDominante();
		ssd1306_FillRectangle(45, 0, 128, 10, Black);  // Borra anterior
		ssd1306_SetCursor(45, 0);
		snprintf(buf, sizeof(buf), "%.2f", g_x);
		ssd1306_WriteString(buf, Font_7x10, White);

	    ssd1306_FillRectangle(45, 16, 128, 26, Black);
		ssd1306_SetCursor(45, 16);
		snprintf(buf, sizeof(buf), "%.2f", g_y);
		ssd1306_WriteString(buf, Font_7x10, White);

		ssd1306_FillRectangle(45, 32, 128, 42, Black);
		ssd1306_SetCursor(45, 32);
		snprintf(buf, sizeof(buf), "%.2f", g_z);
		ssd1306_WriteString(buf, Font_7x10, White);
		ssd1306_UpdateRect(45, 0, 40, 10);
		ssd1306_UpdateRect(45, 16, 40, 10);
		ssd1306_UpdateRect(45, 32, 40, 10);

		bufferFull = 0;

		ssd1306_Fill(Black);
		ssd1306_SetCursor(45, 48);
		snprintf(buffer, sizeof(buffer), "%.1f", freqDominante);
		ssd1306_WriteString(buffer, Font_7x10, White);
		ssd1306_UpdateRect(45, 48, 40, 10);


		break;
	}
	case Blinky:{
		HAL_GPIO_TogglePin(blinky_GPIO_Port, blinky_Pin);
		fsm.estado = IDLE;
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

//Funcion para mostrar los cuadros principales de la pantalla
void cuadrosPrincipales (void){
	  ssd1306_Fill(Black);
	  ssd1306_SetCursor(0, 0);
	  ssd1306_WriteString("acelX:", Font_7x10, White);
	  ssd1306_SetCursor(0, 16);
	  ssd1306_WriteString("acelY:", Font_7x10, White);
	  ssd1306_SetCursor(0, 32);
	  ssd1306_WriteString("acelZ:", Font_7x10, White);
	  ssd1306_SetCursor(0, 48);
	  ssd1306_WriteString("FrecZ:", Font_7x10, White);
	  ssd1306_SetCursor(90, 0);
	  ssd1306_WriteString("m/s2", Font_7x10, White);
	  ssd1306_SetCursor(90, 16);
	  ssd1306_WriteString("m/s2", Font_7x10, White);
	  ssd1306_SetCursor(90, 32);
	  ssd1306_WriteString("m/s2", Font_7x10, White);
	  ssd1306_SetCursor(90, 48);
	  ssd1306_WriteString("Hz", Font_7x10, White);
	  ssd1306_UpdateScreen();
}
//Funcion que calcula la FFT y frecuencia dominante
void calcularFrecuenciaDominante(void) {
	// 1. Quitar el offset (componente DC)
	float promedio = 0.0f;
	for (int i = 0; i < FFT_SIZE; i++) {
	    promedio += fft_input[i];
	}
	promedio /= FFT_SIZE;

	for (int i = 0; i < FFT_SIZE; i++) {
	    fft_input[i] -= promedio;
	}

    arm_rfft_fast_instance_f32 fft_instance;
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

    // Copiamos el buffer
    memcpy(fft_input, accZ_buffer, sizeof(accZ_buffer));

    // Hacemos la FFT
    arm_rfft_fast_f32(&fft_instance, fft_input, fft_input, 0);

    // Calculamos el espectro (magnitud)
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float real = fft_input[2 * i];
        float imag = fft_input[2 * i + 1];
        fft_output[i] = sqrtf(real * real + imag * imag);
    }

    // Buscamos el índice de la magnitud máxima
    uint32_t indexMax = 0;
    float maxValue = 0;
	arm_max_f32(&fft_output[1], FFT_SIZE / 2 - 1, &maxValue, &indexMax);
	indexMax += 1;

    // Convertimos índice en frecuencia
    freqDominante = ((float)indexMax * SAMPLE_FREQ) / FFT_SIZE;
}

//Llamamos el callback para el blinky
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	// Nos aseguramos de que sea el TIMER2 el que haga la interrupcion
    if (htim->Instance == TIM2){
    	//Cambiamos de estado a blinky
    	fsm.estado = Blinky;
    }
}

//Llamamos el callback del EXTI
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_12) { // PB2 conectado a INT2 del acelerometro
    	fsm.estado =guardarDato;
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
