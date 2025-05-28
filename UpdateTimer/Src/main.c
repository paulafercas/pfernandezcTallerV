/**
 ******************************************************************************
 * @file           : main.c
 * @author         : pfernandezc@unal.edu.co
 * @brief          : Main program body
 ******************************************************************************
 */
#include <stdint.h>
#include <stm32f4xx.h>
#include "stm32_assert.h"
#include "gpio_driver_hal.h"
#include "timer_driver_hal.h"

// Definimos un Pin de prueba
GPIO_Handler_t userLed ={0}; //PinA5

Timer_Handler_t blinkTimer = {0};

/*
 * The main function, where everything happens.
 *  */
int main(void)
{
	/* Configuramos el pin */
	userLed.pGPIOx								= GPIOH;
	userLed.pinConfig.GPIO_PinNumber			= PIN_1;
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

	/* Configuramos el Timer */
	timer_Config(&blinkTimer);

	// Encendemos el Timer.
	timer_SetState(&blinkTimer, TIMER_ON);


    /* Loop forever */
	while(1){

	}
	return 0;
}

/*
 *  Overwrite function
 *   */
void Timer2_Callback(void){
	gpio_TooglePin(&userLed);
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
