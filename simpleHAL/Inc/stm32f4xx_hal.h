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

/*Posiciones de memoria para perifericos del APB1 */
//#define PWR_BASE_ADDR 		(APB1_BASE_ADDR + 0x7000U)
//#define I2C3_BASE_ADDR 		(APB1_BASE_ADDR + 0x5C00U)
//#define I2C2_BASE_ADDR 		(APB1_BASE_ADDR + 0x5800U)
//#define I2C1_BASE_ADDR 		(APB1_BASE_ADDR + 0x5400U)
//#define USART2_BASE_ADDR 		(APB1_BASE_ADDR + 0x4400U)
//#define I2Sext_BASE_ADDR 		(APB1_BASE_ADDR + 0x4000U)
//#define SPI3_BASE_ADDR 		(APB1_BASE_ADDR + 0x3C00U)
//#define SPI2_BASE_ADDR 		(APB1_BASE_ADDR + 0x3800U)
//#define I2S2ext_BASE_ADDR 	(APB1_BASE_ADDR + 0x3400U)
//#define IWDG_BASE_ADDR 		(APB1_BASE_ADDR + 0x3000U)
//#define WWDG_BASE_ADDR 		(APB1_BASE_ADDR + 0x2C00U)
//#define RTC_BASE_ADDR 		(APB1_BASE_ADDR + 0x2800U)
//#define TIM5_BASE_ADDR 		(APB1_BASE_ADDR + 0x0C00U)
//#define TIM4_BASE_ADDR 		(APB1_BASE_ADDR + 0x0800U)
//#define TIM3_BASE_ADDR 		(APB1_BASE_ADDR + 0x0400U)
//#define TIM2_BASE_ADDR 		(APB1_BASE_ADDR + 0x0000U)

/*
 * Macros genéricos
 */

#define DISABLE			(0)
#define ENABLE			(1)
#define SET				ENABLE
#define CLEAR			DISABLE
#define RESET			DISABLE
#define FLAG_SET		SET
#define FLAG_RESET		RESET
#define I2C_WRITE		(0)
#define I2C_READ		(1)

/* +++========= INICIO de la descripcion de los elementos que componen el periferico ========+++ */

/*Definicion de la estructura de datos que representa a cada uno de los registros que componen el
 * periferico RCC.
 *
 * Debido a los temas que se van a manejar en el curso, solo se deben definir los bits de los registros:
 * 6. 3. 1 (RCC_CR) hasta el 6.3.12 (RCC_APB2ENR), 6.3.17 (RCC_BDCR) y 6.3.18 (RCC_CSR)
 *
 * NOTA: La posicion de memoria (offset) debe encajar perfectamente con la posicion de memoria indicada
 * en la hoja de datos del equipo. Observe que los elementos "reservedx" tambien estan presentes alli.
 */

typedef struct
{
	volatile uint32_t CR; 			//CLock Control Register					ADDR_OFFSET:	0x00
	volatile uint32_t PLLCFGR;  	//PLL Configuration Register    			ADDR_OFFSET:	0x04
	volatile uint32_t CFGR;     	//Clock Configuration register  			ADDR_OFFSET:	0x08
	volatile uint32_t CIR;      	//Clock Interrupt Register      			ADDR_OFFSET:	0x0C
	volatile uint32_t AHB1RSTR; 	//AHB1 Peripheral reset register  			ADDR_OFFSET:	0x10
	volatile uint32_t AHB2RSTR; 	//AHB2 peripheral reset register			ADDR_OFFSET:	0x14
	volatile uint32_t reserved0;  	//reserved									ADDR_OFFSET:	0x18
	volatile uint32_t reserved1; 	//reserved									ADDR_OFFSET:	0x1C
	volatile uint32_t APB1RSTR;		//APB1 peripheral reset register    		ADDR_OFFSET:	0x20
	volatile uint32_t APB2RSTR;		//APB2 peripheral reset register			ADDR_OFFSET:	0x24
	volatile uint32_t reserved2; 	//reserved									ADDR_OFFSET:	0x28
	volatile uint32_t reserved3; 	//reserved									ADDR_OFFSET:	0x2C
	volatile uint32_t AHB1ENR; 		//AHB1 peripheral clock enable register		ADDR_OFFSET:	0x30
	volatile uint32_t AHB2ENR; 		//AHB2 peripheral clock enable register		ADDR_OFFSET:	0x34
	volatile uint32_t reserved4; 	//reserved									ADDR_OFFSET:	0x38
	volatile uint32_t reserved5; 	//reserved									ADDR_OFFSET:	0x3C
	volatile uint32_t APB1ENR; 		//APB1 peripheral clock enable register		ADDR_OFFSET:	0x40
	volatile uint32_t APB2ENR; 		//APB2 peripheral clock enable register		ADDR_OFFSET:	0x44
	volatile uint32_t reserved6; 	//reserved									ADDR_OFFSET:	0x48
	volatile uint32_t reserved7; 	//reserved									ADDR_OFFSET:	0x4c
	volatile uint32_t AHB1LPENR; 	//AHB1 clock enable low power register		ADDR_OFFSET:	0x50
	volatile uint32_t AHB2LPENR; 	//AHB2 clock enable low power register		ADDR_OFFSET:	0x54
	volatile uint32_t reserved8; 	//reserved									ADDR_OFFSET:	0x58
	volatile uint32_t reserved9; 	//reserved									ADDR_OFFSET:	0x5C
	volatile uint32_t APB1LPENR; 	//AHB1 clock enable low power register		ADDR_OFFSET:	0x60
	volatile uint32_t APB2LPENR; 	//AHB2 clock enable low power register		ADDR_OFFSET:	0x64
	volatile uint32_t reserved10; 	//reserved									ADDR_OFFSET:	0x68
	volatile uint32_t reserved11; 	//reserved									ADDR_OFFSET:	0x6C
	volatile uint32_t BDCR; 		//Backup domain control register			ADDR_OFFSET:	0x70
	volatile uint32_t CSR; 			//Clock control & status register			ADDR_OFFSET:	0x74
	volatile uint32_t reserved12; 	//reserved									ADDR_OFFSET:	0x78
	volatile uint32_t reserved13; 	//reserved									ADDR_OFFSET:	0x7C
	volatile uint32_t SSCGR;		//spread spectrum clock generation reg		ADDR_OFFSET:	0x80
	volatile uint32_t PLLI2SCFGR; 	//PLLI2S configuration register				ADDR_OFFSET:	0x84
	volatile uint32_t reserved14; 	//reserved									ADDR_OFFSET:	0x88
	volatile uint32_t DCKCFGR; 		//Dedicated Clocks configuration			ADDR_OFFSET:	0x8C
}RCC_RegDef_t;

/*
 * Hacemos como un "merge", en el cual ubicacmmos la estructura RCC_Regdef_t a apuntar a la poiscion de
 * memoria correspondiente, de forma que cada variable dentro de la estructura coincide con cada uno
 * de los SFR en la	 memoria del micro-controlador
 */
#define RCC					((RCC_RegDef_t *) RCC_BASE_ADDR)

/* Descripcion bit a bit de cada uno de los registros del que componen al periferico RCC */
/* 6.3.1 RCC_CR */
/* 6.3.2 RCC_PLLCFGR */
/* 6.3.3 RCC_CFGR */
/* 6.3.4 RCC_CIR */
/* 6.3.5 RCC_AHB1RSTR */
/* 6.3.6 RCC_AHB2RSTR */
/* 6.3.7 RCC_APB1RSTR */
/* 6.3.8 RCC_APB2RSTR */

/* 6.3.9 RCC_AHB1ENR */
#define RCC_AHB1ENR_GPIOAEN			0
#define RCC_AHB1ENR_GPIOBEN			1
#define RCC_AHB1ENR_GPIOCEN			2
#define RCC_AHB1ENR_GPIODEN			3
#define RCC_AHB1ENR_GPIOEEN			4
#define RCC_AHB1ENR_GPIOHEN			7
#define RCC_AHB1ENR_CRCEN			12
#define RCC_AHB1ENR_DMA1EN			21
#define RCC_AHB1ENR_DMA2EN			22

/* 6.3.10 RCC_AHB2ENR */
/* 6.3.11 RCC_APB1ENR */
/* 6.3.12 RCC_AHB2ENR */
/* 6.3.17 RCC_BDCR */
/* 6.3.18 RCC_CSR */

/* ====== FIN de la descripcion de los elementos que componen el periferico ====== */

/* ++++++ Posibles valores de las configuraciones */
/* System CLock (main clock) possible options*/
/* Microcontroller clock output 1 (MCO1) optionns */
/* Microcontroller preescaler clock output1 (MCO1PRE) options */
/* Microcontroller preescaler clock output1 (MCO1PRE) options */
/* Microcontroller clos output 2 (MCO2) optiones */

/* +++====== INICIO de la descripcion de los elementos que componen el periferico ========+++ */
/* Definicion de la estructura de datos que representa a cada uno de los registros que componen el
 * periferico GPIO.
 *
 * Debido a que el periferico GPIOx es muy simple, no es necesario crear la descripcion bit a bit
 * de cada uno de loss registros que componen el dicho periferico, pero di es necesario comprendeer que
 * hace cada registro, para poder cargar correctamente la configuracion.
 *
 */

typedef struct
{
	volatile uint32_t MODER; 		//port mode register					ADDR_OFFET: 0x00
	volatile uint32_t OTYPER;		//port output type register				ADDR_OFFET: 0x04
	volatile uint32_t OSPEEDR;		//port output speed register			ADDR_OFFET: 0x08
	volatile uint32_t PUPDR;		//port pull-up/pull-down register		ADDR_OFFET: 0x0C
	volatile uint32_t IDR;			//port input data register				ADDR_OFFET: 0x10
	volatile uint32_t ODR;			//port output data register				ADDR_OFFET: 0x14
	volatile uint32_t BSRR;			//port bit set/reset register			ADDR_OFFET: 0x18
	volatile uint32_t LCKR;			//port configuration lock register		ADDR_OFFET: 0x1C
	volatile uint32_t AFRI [2];		//port alternate function registers, 	ADDR_OFFET: 0x20-0x24
} GPIO_TypeDef;

/* Creamos un objeto de la estructura definida y hacemos que quede ubicada exactamente sobre
 * la posicion de memoria donde se encuentra en registro
 * Debido a que son varios perifericos GPIOx, es necesario hacer la definicion para cada uno.
 *
 * Tener cuidado que cada elemento coincida con su respectiva direccion base.
 * */
#define GPIOA				((GPIO_TypeDef *) GPIOA_BASE_ADDR)
#define GPIOB				((GPIO_TypeDef *) GPIOB_BASE_ADDR)
#define GPIOC				((GPIO_TypeDef *) GPIOC_BASE_ADDR)
#define GPIOD				((GPIO_TypeDef *) GPIOD_BASE_ADDR)
#define GPIOE				((GPIO_TypeDef *) GPIOE_BASE_ADDR)
#define GPIOH				((GPIO_TypeDef *) GPIOH_BASE_ADDR)

/* For testing assert parameter. */
#define IS_GPIO_ALL_INSTANCE (GPIOx) ((GPIOx == GPIOA) || \
									  (GPIOx == GPIOB) || \
									  (GPIOx == GPIOC) || \
									  (GPIOx == GPIOD) || \
									  (GPIOx == GPIOE) || \
									  (GPIOx == GPIOH))

/*
typedef struct
{
	volatile uint32_t dummy; 		//Dummy Example register		ADDR_OFFSET: 	 0x00
} DUMMY_RegDef_t;

*/
/* No es necesario hacer nada mas en este archivo, para este primer proyecto */

/* Incluimos las librerias de cada periferico */

#endif /* _STM32F4XX_HAL_ */


