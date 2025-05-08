/**
 ******************************************************************************
 * @file           : main.c
 * @author         : pfernandezc
 * @brief          : MBreve introduccion a la programacion MCUs
 ******************************************************************************
 *
 ******************************************************************************
 */

#include <stdint.h>
#define RCC_BASE_ADRESS 0x40023800UL //Las dos letras al final significan unsigned long
#define RCC_AHB1ENR_OFFSET 0x30UL
#define RCC_AHB1ENR (RCC_BASE_ADRESS+RCC_AHB1ENR_OFFSET)

#define GPIOA_BASE_ADRESS 0x40020000 //Posicion de memoria donde esta definido el GPIOA
#define GPIOA_MODE_REG_OFFSET  0x00UL
#define GPIOA_ODR_REG_OFFSET   0x14UL
#define GPIOA_MODE_REG (GPIOA_BASE_ADRESS + GPIOA_MODE_REG_OFFSET)
#define GPIOA_ODR_REG (GPIOA_BASE_ADRESS+ GPIOA_ODR_REG_OFFSET)


int main(void)
{
	uint32_t *registerAHB1enb =(uint32_t *)RCC_AHB1ENR; //Haciendo que la variable puntero registerAHB1enb apunte a donde se encuentra ese registro
	/*utilizando las operaciones bitwise, deseamos escribir 1 en la posicion 0 del registro*/
	*registerAHB1enb &= (1<<0); //Escribiendo un cero en la posicion 0 -> borrando lo que se tiene alli
	*registerAHB1enb |= (1<<0); //Escribiendo un 1 en la posicion 0 del registro RCC AHB1ENR -> activando la señal de reloj para encender el GPIO

	uint32_t *registerGPIOA_MODE =(uint32_t *)GPIOA_MODE_REG; //Creando una variable que apunta al registro GPIO register
	*registerGPIOA_MODE &= ~(0b11 << 10); //Limpiando la posicion que controla al pin 5 del GPIOA
	*registerGPIOA_MODE |= (0b01 <<10); //Activando al pin GPIOA_5 en modo de salida de proposito general

	uint32_t *registerGPIOA_ODR = (uint32_t *)GPIOA_ODR_REG; //Creando una variable que apunta al registro GPIOA_ODR register
	*registerGPIOA_ODR |=(1<<5); //encendiendo el pin GPIO_5 -> el Led verde de la board se debe encender




	/* Loop forever */
	while(1){

	}
	return 0;
}
