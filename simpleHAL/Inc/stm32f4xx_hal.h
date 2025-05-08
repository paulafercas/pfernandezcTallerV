/**
 ******************************************************************************
 * @file           : main.c
 * @author         : pfernandezc
 * @brief          : simpleHAL
 ******************************************************************************
 *Este archivo contiene la informacion mas basica del micro:
 * -Valores del reloj principal
 * -Distribucion basica de la memoria (descripcion en la figura 14 de la hoja de datos del micro)
 * -Posiciones de memoria de los perifericos disponibles en el micro descrito en la
 * tabla 1 (Memory Map)
 * -Incluir los demas drivers de los perifericos
 * -Definiciones de las constantes mas basicas
 *
 * NOTA: la definicion del NVIC sera realizada al momento de describir el uso de las
 * interrupciones
 *
 ******************************************************************************
 */

#ifndef _STM32F4XX_HAL_H_
#define _STM32F4XX_HAL_H_

#include <stdint.h>
#include <stddef.h>

#define HSI_CLOCK_SPEED 8000000 //Value for the main clock signal (HSI -> High speed Internal)
#define HSE_CLOCK_SPEED 16000000 //Value for the main clock signal (HSE -> HIgh speed EXternal)

//#define NOP () (asm ("NOP"))
#define NOP() asm ("NOP")
#define _weak 	_attribute_((weak))

/*
 * Base addresses of flash and sram memories
 * Datasheet, memory map, figure 14
 * (Remember, 1kbyte = 1024 bytes
 */

#define FLASH_BASE_ADDR 0x08000000U //Esta es la memoria del programa, 512KB
#define SRMA_BASE_ADDR  0x20000000U //Esta es la memoria RAM, 128KN

/*NOTA: observar que existen unos registros específicos del Cortex M4 en la región 0xE000000U
 * Los controladores de las interrupciones se encuentran allí, por ejemplo. ESto se vera
 * a su debido tiempo
 */

/*NOTA:
 * Ahora agregamos la direccion de memoria base para cada uno de los perifericos que posee el micro
 * En el "datasheet" del micor, figura 14 (Memory map) encontramos el mapa de los buses:
 * -APB1 (advance peripherical Bus)
 * -APB2
 */
/**
 * AHBx and APBx Bus Peripherals base addresses
 */

#define APB1_BASE_ADDR 0x4000000U
#define APB2_BASE_ADDR 0x4001000U
#define AHB1_BASE_ADDR 0x4002000U
#define AHB2_BASE_ADDR 0x5000000U

/**
 * Y ahora debemos hacer lo mismo pero cada una de las posiciones de memoria de cada uno de los
 * periféricos descritos en la tabla 1 del manual de referencia del micro.
 * Observe que en dicha tabla está a su vez dividida en cuatro segmentos, cada uno correspondiente
 * a APB1, APB2, AHB1, AHB2.
 *
 * Comenzar de arriba hacia abajo como se muestra en la tabla. Inicia USB_OTG_FS (AHB2)
 * */
/*Posiciones de memoria para perifericos del AHB2*/
#define USB_OTG_FS_BASE_ADDR (AHB2_BASE_ADDR + 0x0000U)

/*Posiciones de memoria para perifericos del AHB1 */
#define DMA2_BASE_ADDR 			(AHB1_BASE_ADDR + 0x6400U)
#define DMA1_BASE_ADDR 			(AHB1_BASE_ADDR + 0x600U)
#define FIR_BASE_ADDR 			(AHB1_BASE_ADDR + 0x3C00U) //Flash
#define RCC_BASE_ADDR 			(AHB1_BASE_ADDR + 0x3800U)
#define CRC_ASE_ADDR 			(AHB1_BASE_ADDR + 0x3000U)
#define GPIOH_BASE_ADDR 		(AHB1_BASE_ADDR + 0x1C00U)
#define GPIOE_BASE_ADDR 		(AHB1_BASE_ADDR + 0x1000U)
#define GPIOD_BASE_ADDR 		(AHB1_BASE_ADDR + 0x0C00U)
#define GPIOC_BASE_ADDR 		(AHB1_BASE_ADDR + 0x0800U)
#define GPIOB_BASE_ADDR 		(AHB1_BASE_ADDR + 0x0400U)
#define GPIOA_BASE_ADDR 		(AHB1_BASE_ADDR + 0x0000U)

/*Posiciones de memoria para perifericos del APB2 */
//#define SP15_BASE_ADDR 		(APB2_BASE_ADDR + 0x5000U)
//#define TIM11_BASE_ADDR 		(APB2_BASE_ADDR + 0x4800U)
//#define TIM10_BASE_ADDR 		(APB2_BASE_ADDR + 0x4400U)
//#define TIM9_BASE_ADDR 		(APB2_BASE_ADDR + 0x4000U)
//#define EXTI_BASE_ADDR 		(APB2_BASE_ADDR + 0x3C00U)
//#define SYSFG_BASE_ADDR 		(APB2_BASE_ADDR + 0x3800U)
//#define SPI4_BASE_ADDR 		(APB2_BASE_ADDR + 0x3400U)
//#define SPI1_BASE_ADDR 		(APB2_BASE_ADDR + 0x3000U)
//#define SDIO_BASE_ADDR 		(APB2_BASE_ADDR + 0x2C00U)
//#define ADC1_BASE_ADDR 		(APB2_BASE_ADDR + 0x2800U)
//#define USART6_BASE_ADDR 		(APB2_BASE_ADDR + 0x1400U)
//#define USART1_BASE_ADDR 		(APB2_BASE_ADDR + 0x1000U)
//#define TIM1_BASE_ADDR 		(APB2_BASE_ADDR + 0x0000U)



int main(void)
{
    /* Loop forever */
	for(;;);
}

