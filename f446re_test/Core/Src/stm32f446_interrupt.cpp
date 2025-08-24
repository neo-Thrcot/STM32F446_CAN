/*
 * stm32f446_interrupt.c
 *
 *  Created on: Aug 20, 2025
 *      Author: neoki
 */

#include <stm32f446_sys.h>
#include <stm32f446_usart.h>

#ifdef __cplusplus
extern "C" {
#endif

void SysTick_Handler(void)
{
	IncTick();
}

void USART2_IRQHandler(void)
{
	Serial.IRQ_Handler();
	GPIOA->ODR ^= (1UL << 5);
}

#ifdef __cplusplus
}
#endif
