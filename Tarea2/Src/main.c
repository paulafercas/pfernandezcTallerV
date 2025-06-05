/**
 ******************************************************************************
 * @file           : main.c
 * @author         : pfernandezc@unal.edu.co
 * @brief          : Tarea 2
 ******************************************************************************
 */
#include <stdint.h>
#include <stm32f4xx.h>
//#include "stm32_assert.h"
#include "gpio_driver_hal.h"
#include "timer_driver_hal.h"
#include "exti_driver_hal.h"
#include "main.h"

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

// Definimos el Pin Blinky
GPIO_Handler_t blinky ={0}; //PinH1

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

//Definimos los pines que estamos utilizando para los botones
//y el encoder
GPIO_Handler_t gpioCLK ={0}; //PinC3
EXTI_Handler_t extiCLK = {0}; //EXTI del CLK
GPIO_Handler_t gpioDT = {0}; //PinPC9
EXTI_Handler_t extiDT ={0}; //EXTI del DT
GPIO_Handler_t gpioSW ={0}; //PinB8
EXTI_Handler_t extiSW ={0}; //EXTI del SW
GPIO_Handler_t gpioAumentarRefresh = {0}; //PinA5
EXTI_Handler_t extiAumentarRefresh ={0}; //EXTI del botonAumentarRefresh
GPIO_Handler_t gpioDisminuirRefresh ={0}; //PinC2
EXTI_Handler_t extiDisminuirRefresh ={0}; //EXTI del botonDisminuirRefresh


//Definimos los timers
Timer_Handler_t blinkTimer = {0};
Timer_Handler_t refreshTimer ={0};

//Inicializamos la variable que va a modificar la tasa de refresco
//Comenzamos con un periodo de 20ms
uint16_t tasa_refresco = 20;

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

/*
 * The main function, where everything happens.
 *  */
int main(void)
{
	configurar7Segmentos();
	configurarTimers ();
	configurarExti ();
	fsm.estado = refrescar;

    /* Loop forever */
	while(1){
		if(fsm.estado != IDLE){
			maquinaEstados(numeroDisplay,digito);
		}
	}
	return 0;
}
/*
 * Funcion que configura los pines que estamos utilizando
 */
void configurar7Segmentos (void){

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
}

void configurarTimers (void){

	/* Configuramos el pin para el BLinky*/
	blinky.pGPIOx							= GPIOH;
	blinky.pinConfig.GPIO_PinNumber			= PIN_1;
	blinky.pinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	blinky.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	blinky.pinConfig.GPIO_PinOutputSpeed	= GPIO_OSPEED_MEDIUM;
	blinky.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	/* Cargamos la configuracion en el pin que gobierna el puerto*/
	gpio_Config(&blinky);

	gpio_WritePin(&blinky,SET);

	blinkTimer.pTIMx								=TIM2;
	blinkTimer.TIMx_Config.TIMx_Prescaler			=16000; //Genera incrementos de 1 ms
	blinkTimer.TIMx_Config.TIMx_Period				=250;  // De la mano con el prescaler,
	blinkTimer.TIMx_Config.TIMx_mode				= TIMER_UP_COUNTER;
	blinkTimer.TIMx_Config.TIMx_InterruptEnable 	= TIMER_INT_ENABLE;

	refreshTimer.pTIMx								=TIM3;
	refreshTimer.TIMx_Config.TIMx_Prescaler			=16000; //Genera incrementos de 1 ms
	refreshTimer.TIMx_Config.TIMx_Period			=tasa_refresco;  // De la mano con el prescaler,
	refreshTimer.TIMx_Config.TIMx_mode				= TIMER_UP_COUNTER;
	refreshTimer.TIMx_Config.TIMx_InterruptEnable 	= TIMER_INT_ENABLE;

	/* Configuramos el BlinkTimer */
	timer_Config(&blinkTimer);

	// Encendemos el BlinkTimer.
	timer_SetState(&blinkTimer, TIMER_ON);

	/* Configuramos el refreshTimer */
	timer_Config(&refreshTimer);

	// Encendemos el Timer.
	timer_SetState(&refreshTimer, TIMER_ON);
}

void configurarExti (void){
	gpioCLK.pGPIOx							= GPIOC;
	gpioCLK.pinConfig.GPIO_PinNumber		= PIN_3;
	gpioCLK.pinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	gpioCLK.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	//extiCLK.pGPIOHandler					= &gpioCLK;
	//extiCLK.edgeType 						= EXTERNAL_INTERRUPT_RISING_EDGE;

	gpioDT.pGPIOx							= GPIOC;
	gpioDT.pinConfig.GPIO_PinNumber			= PIN_9;
	gpioDT.pinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	gpioDT.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	extiDT.pGPIOHandler						= &gpioDT;
	extiDT.edgeType 						= EXTERNAL_INTERRUPT_RISING_EDGE;

	gpioSW.pGPIOx							= GPIOB;
	gpioSW.pinConfig.GPIO_PinNumber			= PIN_8;
	gpioSW.pinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	gpioSW.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	extiSW.pGPIOHandler						= &gpioSW;
	extiSW.edgeType							= EXTERNAL_INTERRUPT_RISING_EDGE;

	gpioAumentarRefresh.pGPIOx							= GPIOA;
	gpioAumentarRefresh.pinConfig.GPIO_PinNumber		= PIN_5;
	gpioAumentarRefresh.pinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	gpioAumentarRefresh.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	extiAumentarRefresh.pGPIOHandler					= &gpioAumentarRefresh;
	extiAumentarRefresh.edgeType						= EXTERNAL_INTERRUPT_FALLING_EDGE;

	gpioDisminuirRefresh.pGPIOx							= GPIOC;
	gpioDisminuirRefresh.pinConfig.GPIO_PinNumber		= PIN_2;
	gpioDisminuirRefresh.pinConfig.GPIO_PinMode			= GPIO_MODE_IN;
	gpioDisminuirRefresh.pinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;

	extiDisminuirRefresh.pGPIOHandler					= &gpioDisminuirRefresh;
	extiDisminuirRefresh.edgeType						= EXTERNAL_INTERRUPT_FALLING_EDGE;

	/*Cargamos la configuracion a los pines que gobiernan sus respectivos puertos*/

	gpio_Config (&gpioCLK);
	gpio_Config (&gpioDT);
	gpio_Config (&gpioSW);
	gpio_Config (&gpioAumentarRefresh);
	gpio_Config (&gpioDisminuirRefresh);

	/*Cargamos la configuracion para los exti*/
	//exti_Config (&extiCLK);
	exti_Config (&extiDT);
	exti_Config (&extiSW);
	exti_Config (&extiAumentarRefresh);
	exti_Config (&extiDisminuirRefresh);



}
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
		gpio_WritePin (&alimentacion3, SET);
		gpio_WritePin (&alimentacion1, SET);
		gpio_WritePin (&alimentacion2, SET);
		gpio_WritePin (&alimentacion0, SET);
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
		timer_SetState(&refreshTimer, TIMER_OFF);
		//Nos aseguramos de que la tasa de refresco no vaya a sea menor a 4ms
		if (tasa_refresco<=1){
			tasa_refresco = 20;
		}
		//Disminuimoss en 1 ms la tasa de refresco
		else {
			tasa_refresco/=2;
		}

		//Configuramos el refresh Timer
		refreshTimer.TIMx_Config.TIMx_Period			=tasa_refresco;
		//Guardamos las nuevas configuraciones
		timer_Config (&refreshTimer);
		// Encendemos nuevamente el Timer
		timer_SetState(&refreshTimer, TIMER_ON);
		fsm.estado = refrescar;
		break;
	}
	case disminuir_tasa_refresco:{
		// Apagamos el Timer.
		timer_SetState(&refreshTimer, TIMER_OFF);
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
		refreshTimer.TIMx_Config.TIMx_Period			=tasa_refresco;
		//Guardamos las nuevas configuraciones
		timer_Config (&refreshTimer);
		// Encendemos nuevamente el Timer
		timer_SetState(&refreshTimer, TIMER_ON);
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
		gpio_TooglePin(&blinky);
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
		gpio_WritePin (&alimentacion0, RESET);
		break;
	}
	case 1: {
		//Encendemos los pines correspondientes al numero que se desea
		definir_numero (decena);
		//Encedemos el digito (1) donde se va a mostrar el numero seleccionado anteriormente
		gpio_WritePin (&alimentacion1, RESET);
		break;
	}
	case 2:{
		//Encendemos los pines correspondientes al numero que se desea
		definir_numero (centena);
		//Encedemos el digito (1) donde se va a mostrar el numero seleccionado anteriormente
		gpio_WritePin (&alimentacion2, RESET);
		break;
	}
	case 3:{
		//Encendemos los pines correspondientes al numero que se desea
		definir_numero (milUnidad);
		//Encedemos el digito (1) donde se va a mostrar el numero seleccionado anteriormente
		gpio_WritePin (&alimentacion3, RESET);
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
uint16_t cambioNumero (uint16_t numeroLocal){
	//Creamos una variable en la cual guardamos el valor del pin
	//donde está el DT
	uint32_t valor_DT = 0;
	//Cargamos el valor del pin en la variable
	valor_DT = gpio_ReadPin(&gpioCLK);
	//valor_DT = gpio_ReadPin(&gpioDT);
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

/*
 * Timer que controla el blinky
 */
void timer2_Callback(void){
	fsm.estado = Blinky;
}
/*
 * Timer que controla el refresh
 */
void timer3_Callback(void){
	digito +=1;
	//Debemos cersiorarnos de que el digito que queremos encender no tenga un
	//valor mayor a 3 (ya que solo tenemos 4 digitos)
	if (digito >= 4){
		digito=0;
	}
}

//void callback_ExtInt3 (void){
	//Cambiamos el estado a "cambiar numero"
	//fsm.estado = cambiar_numero;
//}

void callback_ExtInt9 (void){
	//Cambiamos el estado a "cambiar numero"
	fsm.estado = cambiar_numero;
}

void callback_ExtInt5 (void){
	//cambiamos el estado a "aumentar tasa de refresco"
	fsm.estado = aumentar_tasa_refresco;
}
void callback_ExtInt2 (void){
	//Cambiamos el estado a "disminuir tasa de refresco"
	fsm.estado = disminuir_tasa_refresco;
}

void callback_ExtInt8 (void){
	//Cambiamos el estado a resetear
	fsm.estado = resetear;
}

/*
 * Esta funcion sirve para detectar problemas de parametros
 * incorrectos al momento de ejecutar un programa.
 * */
void assert_failed(uint8_t* file, uint32_t line){
	while(1){
		// problems...
	}
}




