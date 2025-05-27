/**
 ******************************************************************************
 * @file           : main.c
 * @author         : pfernandezc@unal.edu.co
 * @brief          : Tarea 2
 ******************************************************************************
 */
#include <stdint.h>
#include <stm32f4xx.h>
#include "stm32_assert.h"
#include "gpio_driver_hal.h"
#include "timer_driver_hal.h"
#include "exti_driver_hal.h"


//Definimos una enumeracion con los diferentes estados que puede tener la maquina
typedef enum{
	refrescar,
	cambiar_numero,
	cambiar_tasa_refresco,
}Estado;

typedef enum{
	unidad1 =0,
	decena1,
	centena1,
	milUnidad1,
}parteNumero;



void maquinaEstados(Estado actual, uint8_t digito);

//Creamos la variable donde vamos a guardar el numero que se va a mostrar en el display
uint16_t numeroDisplay =0;

//Inicializamos las variables que van a permitir separar el numeroDisplay en 4 partes
uint8_t unidad = 0;
uint8_t decena = 0;
uint8_t centena = 0;
uint8_t milUnidad = 0;


//Inicializamos la variable donde guardamos que digito queremos encender (0,1,2,3)
uint8_t digito =0;

// Definimos un Pin de prueba
GPIO_Handler_t userLed ={0}; //PinA5

//Definimos los pines que estamos utilizando para el display 7 segmentos
GPIO_Handler_t segmento1={0}; //PinD2
GPIO_Handler_t segmento2={0}; //PinC10
GPIO_Handler_t segmento3={0}; //PinC11
GPIO_Handler_t segmento4={0}; //PinC12
GPIO_Handler_t segmento5={0}; //PinB7
GPIO_Handler_t segmento7={0}; //PinC8
GPIO_Handler_t segmento10={0}; //PinC6
GPIO_Handler_t segmento11={0}; //PinC5
GPIO_Handler_t alimentacion0={0}; //PinC13
GPIO_Handler_t alimentacion1={0}; //PinA12
GPIO_Handler_t alimentacion2={0}; //PinB12
GPIO_Handler_t alimentacion3={0}; //PinA11


Timer_Handler_t blinkTimer = {0};

/*
 * Declaramos la funciones que aparecerán próximamente
 */
int separacion_parte (parteNumero parte, uint16_t numeroDisplay);


/*
 * The main function, where everything happens.
 *  */
int main(void)
{
	/*
	 * Vamos a dividir el numeroDisplay en unidades, decenas, centenas y unidades de mil.
	 */
	unidad =  separacion_parte (unidad1, numeroDisplay);
	decena = separacion_parte (decena1, numeroDisplay);
	centena = separacion_parte (centena1, numeroDisplay);
	milUnidad = separacion_parte (milUnidad1, numeroDisplay);


	/*Configuramos lo pines que estamos utilizando*/
	segmento1.pGPIOx							= GPIOD;
	segmento1.pinConfig.GPIO_PinNumber			= PIN_2;
	segmento1.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	segmento1.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	segmento1.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	segmento1.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

	segmento2.pGPIOx							= GPIOC;
	segmento2.pinConfig.GPIO_PinNumber			= PIN_10;
	segmento2.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	segmento2.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	segmento2.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	segmento2.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

	segmento3.pGPIOx							= GPIOC;
	segmento3.pinConfig.GPIO_PinNumber			= PIN_11;
	segmento3.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	segmento3.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	segmento3.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	segmento3.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

	segmento4.pGPIOx							= GPIOC;
	segmento4.pinConfig.GPIO_PinNumber			= PIN_12;
	segmento4.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	segmento4.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	segmento4.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	segmento4.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

	segmento5.pGPIOx							= GPIOB;
	segmento5.pinConfig.GPIO_PinNumber			= PIN_7;
	segmento5.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	segmento5.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	segmento5.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	segmento5.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

	segmento7.pGPIOx							= GPIOC;
	segmento7.pinConfig.GPIO_PinNumber			= PIN_8;
	segmento7.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	segmento7.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	segmento7.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	segmento7.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

	segmento10.pGPIOx							= GPIOC;
	segmento10.pinConfig.GPIO_PinNumber			= PIN_6;
	segmento10.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	segmento10.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	segmento10.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	segmento10.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	segmento11.pGPIOx							= GPIOC;
	segmento11.pinConfig.GPIO_PinNumber			= PIN_5;
	segmento11.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	segmento11.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	segmento11.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	segmento11.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	alimentacion0.pGPIOx							= GPIOC;
	alimentacion0.pinConfig.GPIO_PinNumber			= PIN_13;
	alimentacion0.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	alimentacion0.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	alimentacion0.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	alimentacion0.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

	alimentacion1.pGPIOx							= GPIOA;
	alimentacion1.pinConfig.GPIO_PinNumber			= PIN_12;
	alimentacion1.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	alimentacion1.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	alimentacion1.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	alimentacion1.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

	alimentacion2.pGPIOx							= GPIOB;
	alimentacion2.pinConfig.GPIO_PinNumber			= PIN_12;
	alimentacion2.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	alimentacion2.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	alimentacion2.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	alimentacion2.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

	alimentacion3.pGPIOx							= GPIOA;
	alimentacion3.pinConfig.GPIO_PinNumber			= PIN_11;
	alimentacion3.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	alimentacion3.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	alimentacion3.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	alimentacion3.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;



	/* Configuramos el pin */
	userLed.pGPIOx								= GPIOA;
	userLed.pinConfig.GPIO_PinNumber			= PIN_5;
	userLed.pinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	userLed.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	userLed.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	userLed.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

	/* Cargamos la configuracion en los registros que gobiernan el puerto*/
	gpio_Config(&userLed);

	gpio_WritePin(&userLed,SET);

	blinkTimer.pTIMx								=TIM2;
	blinkTimer.TIMx_Config.TIMx_Prescaler			=16000; //Genera incrementos de 1 ms
	blinkTimer.TIMx_Config.TIMx_Period				=250;  // De la mano con el prescaler,
	blinkTimer.TIMx_Config.TIMx_mode				= TIMER_UP_COUNTER;
	blinkTimer.TIMx_Config.TIMx_InterruptEnable 	= TIMER_INT_ENABLE;

	blinkTimer.pTIMx								=TIM3;
	blinkTimer.TIMx_Config.TIMx_Prescaler			=16000; //Genera incrementos de 1 ms
	blinkTimer.TIMx_Config.TIMx_Period				=20;  // De la mano con el prescaler,
	blinkTimer.TIMx_Config.TIMx_mode				= TIMER_UP_COUNTER;
	blinkTimer.TIMx_Config.TIMx_InterruptEnable 	= TIMER_INT_ENABLE;

	/* Configuramos el Timer */
	timer_Config(&blinkTimer);

	// Encendemos el Timer.
	timer_SetState(&blinkTimer, TIMER_ON);

	//Definimos la variable que va a gurdar el estado en el que se encuentra mi maquina de estados

	Estado actual = actual;

	maquinaEstados(actual,digito,unidad, decena, centena, milUnidad);

    /* Loop forever */
	while(1){

	}
	return 0;
}

/*
 *  Overwrite function
 *   */

/*
 * Creamos la funcion que le indica a la maquina de estados que debe hacer para cada caso
 */


int separacion_parte (parteNumero parte, uint16_t numeroDisplay){
	if (parte == unidad1){
		uint8_t unidad = 0;
		unidad = numeroDisplay%10;
		return unidad;
	}
	else if (parte == decena1){
		uint8_t unidad = 0;
		uint8_t decena = 0;
		unidad = numeroDisplay%10;
		decena = ((numeroDisplay-unidad)/10)%10;
		return decena;
	}
	else if (parte == centena1){
		uint8_t residuoCentena = 0;
		uint8_t centena = 0;
		residuoCentena = numeroDisplay%100;
		centena = ((numeroDisplay - residuoCentena)/100)%10;
		return centena;
	}
	else if (parte == milUnidad1){
		uint8_t residuoMil = 0;
		residuoMil = numeroDisplay%1000;
		uint8_t milUnidad = 0;
		milUnidad = (numeroDisplay-residuoMil)/1000;
		return milUnidad;
	}
	else {
		__NOP();
	}

int maquinaEstados(Estado actual,uint8_t digito,uint8_t unidad, uint8_t decena, uint8_t centena, uint8_t milUnidad){
	if (actual ==refrescar){

		//Primero debemos cersiorarnos de que el digito que queremos encender no tenga un
		//valor mayor a 3 (ya que solo tenemos 4 digitos)
		if (digito ==4){
			digito == 0;
		}
		else{
			__NOP();
		}
		digito_encendido(digito,unidad, decena, centena, milUnidad);

		//Llamamos a la funcion que nos indica qué pines deben estar encendidos en el digito
		//que deseo mostrar
	}
	return digito;
}

void digito_encendido(uint8_t digito, uint8_t unidad,uint8_t decena, uint8_t centena, uint8_t milUnidad){
	switch (digito){
	//Caso en el cual queremos ver el numero en el digito de las unidades
	case 0: {
		gpio_WritePin (&alimentacion3, SET);
		definir_numero (unidad);
		gpio_WritePin (&alimentacion0, RESET);
		break;
	}
	case 1: {
		gpio_WritePin (&alimentacion0, SET);
		definir_numero (decena);
		gpio_WritePin (&alimentacion1, RESET);
		break;
	}
	case 2:{
		gpio_WritePin (&alimentacion1, SET);
		definir_numero (centena);
		gpio_WritePin (&alimentacion2, RESET);
		break;
	}
	case 3:{
		gpio_WritePin (&alimentacion2, SET);
		definir_numero (milUnidad);
		gpio_WritePin (&alimentacion3, RESET);
		break;
	}
	default:{
		__NOP();
		break;
	}
	}// Fin del Switch-case
}
void definir_numero (uint8_t numero){
	if (numero == 0){

		gpio_WritePin(&segmento1, RESET);
		gpio_WritePin(&segmento2, RESET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento7, RESET);
		gpio_WritePin(&segmento10, RESET);
		gpio_WritePin(&segmento11, RESET);
	} else if (numero == 1){
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento7, RESET);
	} else if (numero ==2){
		gpio_WritePin(&segmento1, RESET);
		gpio_WritePin(&segmento2, RESET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento7, RESET);
		gpio_WritePin(&segmento11, RESET);
	} else if (numero==3){
		gpio_WritePin(&segmento2, RESET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento7, RESET);
		gpio_WritePin(&segmento11, RESET);
	} else if (numero == 4){
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento7, RESET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento10, RESET);
	} else if (numero == 5){
		gpio_WritePin(&segmento2, RESET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento10, RESET);
		gpio_WritePin(&segmento11, RESET);
	} else if (numero == 6){
		gpio_WritePin(&segmento2, RESET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento10, RESET);
		gpio_WritePin(&segmento11, RESET);
		gpio_WritePin(&segmento1, RESET);
	} else if (numero == 7){
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento7, RESET);
		gpio_WritePin(&segmento11, RESET);
	} else if (numero ==8){
		gpio_WritePin(&segmento2, RESET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento10, RESET);
		gpio_WritePin(&segmento11, RESET);
		gpio_WritePin(&segmento1, RESET);
		gpio_WritePin(&segmento7, RESET);
	} else if (numero ==9){
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento10, RESET);
		gpio_WritePin(&segmento11, RESET);
		gpio_WritePin(&segmento7, RESET);
	} else {
		__NOP();
	}
}
void Timer2_Callback(void){
	gpio_TooglePin(&userLed);
}

void Timer3_Callback(void){
	digito ++;
}
/*
 * ESta funcion sirve para detectar problemas de parametros
 * incorrectos al momentos de ejecutar un programa.
 * */
void assert_failed(uint8_t* file, uint32_t line){
	while(1){
		// problems...
	}
}



