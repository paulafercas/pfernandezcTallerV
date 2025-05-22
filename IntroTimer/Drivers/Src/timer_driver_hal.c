/*
 * timer_driver_hal.c
 *
 *  Created on: May 22, 2025
 *      Author: paula-fdez
 */
#include "stm32f4xx.h"
#include "stm32_assert.h"

#include "timer_driver_hal.h"

/* Variable que guarda la referencia del periferico que se esta utilizando*/
TIM_TypeDef *ptrTimerUsed;

/* === Headers for private functions === */
static void timer_enable_clock_peripheral (Timer_Handler_t *pTimerHandler);
static void timer_set_prescaler (Timer_Handler_t *pTimerHandler);
static void timer_set_period (Timer_Handler_t *pTimerHandler);
static void timer_set_mode (Timer_Handler_t *pTimerHandler);
static void timer_config_interrupt (Timer_Handler_t *pTimerHandler);

/* Funcion en la que cargamos la configuracion del Timer
 * Recordar que siempre se debe comenzar con activar la señal de reloj
 * del periferico que se esta utilizando.
 * Además, en este caso, debemos ser cuidadosos al momento de utilizar las interrupciones.
 * Los Timer están conectados directamente al elemento NVIC del Cortex-Mx
 * Debemos configurar y/o utilizar_
 * - TIMx_CR1 (Control Register 1)
 * - TIMx_SMCR (slave mode control register) -> mantener en 0 para modo Timer Basico)
 * -
 *  */


