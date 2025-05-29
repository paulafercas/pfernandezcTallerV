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


//Creamos la variable donde vamos a guardar el numero que se va a mostrar en el display
uint16_t numeroDisplay =4095;
Estado actual = refrescar;

//Inicializamos las variables que van a permitir separar el numeroDisplay en 4 partes
uint8_t unidad = 0;
uint8_t decena = 0;
uint8_t centena = 0;
uint8_t milUnidad = 0;


//Inicializamos la variable donde guardamos que digito queremos encender (0,1,2,3)
uint8_t digito = 0;

// Definimos un Pin de prueba
GPIO_Handler_t userLed ={0}; //PinH1

//Definimos los pines que estamos utilizando para el display 7 segmentos
GPIO_Handler_t segmento1={0}; //PinD2
GPIO_Handler_t segmento2={0}; //PinC10
GPIO_Handler_t segmento3={0}; //PinC11
GPIO_Handler_t segmento4={0}; //PinC12
GPIO_Handler_t segmento5={0}; //PinB7
GPIO_Handler_t segmento7={0}; //PinC8
GPIO_Handler_t segmento10={0}; //PinA12
GPIO_Handler_t segmento11={0}; //PinCB12
GPIO_Handler_t alimentacion0={0}; //PinC13
GPIO_Handler_t alimentacion1={0}; //PinC5
GPIO_Handler_t alimentacion2={0}; //PinC6
GPIO_Handler_t alimentacion3={0}; //PinA11

//Definimos los timers
Timer_Handler_t blinkTimer = {0};
Timer_Handler_t refreshTimer ={0};

/*
 * Declaramos la funciones que aparecerán próximamente
 */
//Funcion que define qué se va a hacer según el estado en el que se encuentre la maquina
void maquinaEstados(Estado actual,uint8_t digito,uint8_t unidad, uint8_t decena, uint8_t centena, uint8_t milUnidad);
//Funcion que separa el numeroDisplay en 4 partes (unidad, decena, centena, milUnidad)
int separacion_parte (parteNumero parte, uint16_t numeroDisplay);
//Funcion que nos asegura de que el parametro "digito" no vaya a ser mayor que 3,
//ya que solo tenemos 4 digitos enumerados desde 0 a 3
uint8_t valor_digito (uint8_t digito);
//Funcion que nos permite encender el numero que queremos en el digito que queremos
void digito_encendido(uint8_t digito, uint8_t unidad,uint8_t decena, uint8_t centena, uint8_t milUnidad);
//Funcion que se encuentra dentro de "digito_encendido" para definir que numero
//quiero ver en cada digito
void definir_numero (uint8_t numero);
//Funion para configurar los pines que usamos
void configurarPines();

/*
 * The main function, where everything happens.
 *  */
int main(void)
{
	configurarPines();
	/*
	 * Vamos a dividir el numeroDisplay en unidades, decenas, centenas y unidades de mil.
	 */
	unidad =  separacion_parte (unidad1, numeroDisplay);
	decena = separacion_parte (decena1, numeroDisplay);
	centena = separacion_parte (centena1, numeroDisplay);
	milUnidad = separacion_parte (milUnidad1, numeroDisplay);

	/* Cargamos la configuracion en los registros que gobiernan el puerto*/
	gpio_Config(&userLed);

	gpio_WritePin(&userLed,SET);

	blinkTimer.pTIMx								=TIM2;
	blinkTimer.TIMx_Config.TIMx_Prescaler			=16000; //Genera incrementos de 1 ms
	blinkTimer.TIMx_Config.TIMx_Period				=250;  // De la mano con el prescaler,
	blinkTimer.TIMx_Config.TIMx_mode				= TIMER_UP_COUNTER;
	blinkTimer.TIMx_Config.TIMx_InterruptEnable 	= TIMER_INT_ENABLE;

	refreshTimer.pTIMx								=TIM3;
	refreshTimer.TIMx_Config.TIMx_Prescaler			=16000; //Genera incrementos de 1 ms
	refreshTimer.TIMx_Config.TIMx_Period			=3;  // De la mano con el prescaler,
	refreshTimer.TIMx_Config.TIMx_mode				= TIMER_UP_COUNTER;
	refreshTimer.TIMx_Config.TIMx_InterruptEnable 	= TIMER_INT_ENABLE;

	/* Configuramos el BlinkTimer */
	timer_Config(&blinkTimer);

	// Encendemos el BlinkTimer.
	timer_SetState(&blinkTimer, TIMER_ON);

	/* Configuramos el Timer */
	timer_Config(&refreshTimer);

	// Encendemos el Timer.
	timer_SetState(&refreshTimer, TIMER_ON);


	//Definimos la variable que va a gurdar el estado en el que se encuentra mi maquina de estados

    /* Loop forever */
	while(1){
		maquinaEstados(actual,digito,unidad, decena, centena, milUnidad);
	}
	return 0;
}

/*
 *  Overwrite function
 *   */

/*
 * Creamos la funcion que le indica a la maquina de estados que debe hacer para cada caso
 */

void configurarPines (void){

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

		segmento10.pGPIOx							= GPIOA;
		segmento10.pinConfig.GPIO_PinNumber			= PIN_12;
		segmento10.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
		segmento10.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
		segmento10.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
		segmento10.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

		segmento11.pGPIOx							= GPIOB;
		segmento11.pinConfig.GPIO_PinNumber			= PIN_12;
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

		alimentacion1.pGPIOx							= GPIOC;
		alimentacion1.pinConfig.GPIO_PinNumber			= PIN_5;
		alimentacion1.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
		alimentacion1.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
		alimentacion1.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
		alimentacion1.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

		alimentacion2.pGPIOx							= GPIOC;
		alimentacion2.pinConfig.GPIO_PinNumber			= PIN_6;
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



		/* Configuramos el pin para el BLinky*/
		userLed.pGPIOx								= GPIOH;
		userLed.pinConfig.GPIO_PinNumber			= PIN_1;
		userLed.pinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
		userLed.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
		userLed.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
		userLed.pinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;

		/* Cargamos la configuracion en los registros que gorbiernan cada puerto*/

		gpio_Config(&segmento1);
		gpio_Config(&segmento2);
		gpio_Config(&segmento3);
		gpio_Config(&segmento4);
		gpio_Config(&segmento5);
		gpio_Config(&segmento7);
		gpio_Config(&segmento10);
		gpio_Config(&segmento11);
		gpio_Config(&alimentacion0);
		gpio_Config(&alimentacion1);
		gpio_Config(&alimentacion2);
		gpio_Config(&alimentacion3);

		/*Encendemos incicialmente el numero 0 en los 4 digitos*/
		gpio_WritePin (&segmento1, RESET);
		gpio_WritePin (&segmento2, RESET);
		gpio_WritePin (&segmento3, SET);
		gpio_WritePin (&segmento4, RESET);
		gpio_WritePin (&segmento5, RESET);
		gpio_WritePin (&segmento7, RESET);
		gpio_WritePin (&segmento10, RESET);
		gpio_WritePin (&segmento11, RESET);
		gpio_WritePin (&alimentacion0, SET);
		gpio_WritePin (&alimentacion1, SET);
		gpio_WritePin (&alimentacion2, SET);
		gpio_WritePin (&alimentacion3, SET);

}
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
		return 0;
	}
}

void maquinaEstados(Estado actual,uint8_t digito,uint8_t unidad, uint8_t decena, uint8_t centena, uint8_t milUnidad){
	if (actual ==refrescar){
		//Apagamos todos los digitos
		gpio_WritePin (&alimentacion3, RESET);
		gpio_WritePin (&alimentacion1, RESET);
		gpio_WritePin (&alimentacion2, RESET);
		gpio_WritePin (&alimentacion0, RESET);
		//Llamamos a la funcion que nos indica qué pines deben estar encendidos en el digito
		//que deseo mostrar
		digito_encendido(digito,unidad, decena, centena, milUnidad);

	}
}
/*uint8_t valor_digito (uint8_t digito){
	if (digito>=4){
			digito -= 4;
			return digito;
		}
		else{
			__NOP();
		}
	return digito;
}*/

void digito_encendido(uint8_t digito, uint8_t unidad,uint8_t decena, uint8_t centena, uint8_t milUnidad){
	switch (digito){
	//Caso en el cual queremos ver el numero en el digito de las unidades
	case 0: {
		//Encendemos los pines correspondientes al numero que se desea
		definir_numero (unidad);
		//Encedemos el digito (1) donde se va a mostrar el numero seleccionado anteriormente
		gpio_WritePin (&alimentacion0, SET);
		break;
	}
	case 1: {
		//Encendemos los pines correspondientes al numero que se desea
		definir_numero (decena);
		//Encedemos el digito (1) donde se va a mostrar el numero seleccionado anteriormente
		gpio_WritePin (&alimentacion1, SET);
		break;
	}
	case 2:{
		//Encendemos los pines correspondientes al numero que se desea
		definir_numero (centena);
		//Encedemos el digito (1) donde se va a mostrar el numero seleccionado anteriormente
		gpio_WritePin (&alimentacion2, SET);
		break;
	}
	case 3:{
		//Encendemos los pines correspondientes al numero que se desea
		definir_numero (milUnidad);
		//Encedemos el digito (1) donde se va a mostrar el numero seleccionado anteriormente
		gpio_WritePin (&alimentacion3, SET);
		break;
	}
	default:{
		__NOP();
		break;
	}
	}// Fin del Switch-case
}
void definir_numero (uint8_t numero){
	switch (numero){
	case 0: {
		gpio_WritePin(&segmento1, RESET);
		gpio_WritePin(&segmento2, RESET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento5, SET);
		gpio_WritePin(&segmento7, RESET);
		gpio_WritePin(&segmento10, RESET);
		gpio_WritePin(&segmento11, RESET);
		gpio_WritePin(&segmento3, SET);
		break;
	}
	case 1: {
		gpio_WritePin(&segmento1, SET);
		gpio_WritePin(&segmento2, SET);
		gpio_WritePin(&segmento3, SET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento5, SET);
		gpio_WritePin(&segmento7, RESET);
		gpio_WritePin(&segmento10, SET);
		gpio_WritePin(&segmento11, SET);
		break;
	}
	case 2: {
		gpio_WritePin(&segmento1, RESET);
		gpio_WritePin(&segmento2, RESET);
		gpio_WritePin(&segmento3, SET);
		gpio_WritePin(&segmento4, SET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento7, RESET);
		gpio_WritePin(&segmento10, SET);
		gpio_WritePin(&segmento11, RESET);
		break;
	}
	case 3: {
		gpio_WritePin(&segmento1, SET);
		gpio_WritePin(&segmento2, RESET);
		gpio_WritePin(&segmento3, SET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento7, RESET);
		gpio_WritePin(&segmento10, SET);
		gpio_WritePin(&segmento11, RESET);
		break;
	}
	case 4: {
		gpio_WritePin(&segmento1, SET);
		gpio_WritePin(&segmento2, SET);
		gpio_WritePin(&segmento3, SET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento7, RESET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento10, RESET);
		gpio_WritePin(&segmento11, SET);
		break;
	}
	case 5: {
		gpio_WritePin(&segmento1, SET);
		gpio_WritePin(&segmento2, RESET);
		gpio_WritePin(&segmento3, SET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento7, SET);
		gpio_WritePin(&segmento10, RESET);
		gpio_WritePin(&segmento11, RESET);
		break;
	}
	case 6: {
		gpio_WritePin(&segmento1, RESET);
		gpio_WritePin(&segmento2, RESET);
		gpio_WritePin(&segmento3, SET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento10, RESET);
		gpio_WritePin(&segmento11, RESET);
		gpio_WritePin(&segmento7, SET);
		break;
	}
	case 7: {
		gpio_WritePin(&segmento1, SET);
		gpio_WritePin(&segmento2, SET);
		gpio_WritePin(&segmento3, SET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento5, SET);
		gpio_WritePin(&segmento7, RESET);
		gpio_WritePin(&segmento10, SET);
		gpio_WritePin(&segmento11, RESET);
		break;
	}
	case 8: {
		gpio_WritePin(&segmento2, RESET);
		gpio_WritePin(&segmento3, SET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento10, RESET);
		gpio_WritePin(&segmento11, RESET);
		gpio_WritePin(&segmento1, RESET);
		gpio_WritePin(&segmento7, RESET);
		break;
	}
	case 9: {
		gpio_WritePin(&segmento1, SET);
		gpio_WritePin(&segmento2, RESET);
		gpio_WritePin(&segmento3, SET);
		gpio_WritePin(&segmento4, RESET);
		gpio_WritePin(&segmento5, RESET);
		gpio_WritePin(&segmento10, RESET);
		gpio_WritePin(&segmento11, RESET);
		gpio_WritePin(&segmento7, RESET);
		break;
	}
	default: {
		__NOP();
		break;
	}
	}// Fin del switch-case

}

void timer2_Callback(void){
	gpio_TooglePin(&userLed);
}
void timer3_Callback(void){
	digito +=1;
	//Debemos cersiorarnos de que el digito que queremos encender no tenga un
	//valor mayor a 3 (ya que solo tenemos 4 digitos
	if (digito >= 4){
		digito=0;
	}
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




