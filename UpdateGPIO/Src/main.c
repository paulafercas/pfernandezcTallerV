/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Paula Andrea Fernández
 * @brief          : GPIO
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32_assert.h"
#include "gpio_driver_hal.h"

//Headers definition
int add(int x, int y);

//Definimos un Pin de prueba
GPIO_Handler_t userled = {0}; //PinA5

/*
 * The main function, where everythin happens.
 * */
int main (void)
{
	/* Configuramos el pin */
	userled.pGPIOx								= GPIOA;
	userled.pinConfig.GPIO_PinNumber			= PIN_5;
	userled.pinConfig.GPIO_PinMode				= GPIO_MODE_OUT;
	userled.pinConfig.GPIO_PinOutputType		= GPIO_OTYPE_PUSHPULL;
	userled.pinConfig.GPIO_PinOutputSpeed		= GPIO_OSPEED_MEDIUM;
	userled.pinConfig.GPIO_PinPuPdControl		= GPIO_PUDR_NOTHING;

	/* Cargamos la configuracion en los registros que gobiernan el puerto */
	gpio_Config (&userled);

	gpio_WritePin(&userled, SET);


	while(1){
		gpio_TooglePin(&userled);
		for(uint32_t j=0; j<200000; j++){

		}
	}
}

/*
 * Esta funcion sirve para detectar problemas de parametros
 * incorrectos al momento de ejecutar un programa.
 * */
void assert_failed(uint8_t* file, uint32_t line){
	while(1){
		//problems...
	}
}
