/**
 * *****************************************************************************
 * @file				:	main.c
 * @author				:	pfernandezc
 * @brief				:	Main program body
 * *****************************************************************************
 *
 * Assert works like
 * if (condition){
 * 		// then is true, and codes executes.
 * }else
 * {
 * 		// code breaks
 * 	}
 *
 * 	(assert(condition) -> if condition is not true, the code breaks...)
 *
 * 	*****************************************************************************
 */
#include <stdint.h>
#include "stm32f4xx_hal.h"
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
		gpio_WritePin(&userled, SET);
		for(uint32_t i=0; i<4000000; i++){
		}
		gpio_WritePin(&userled, RESET);
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
