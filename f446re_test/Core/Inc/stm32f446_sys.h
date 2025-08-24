/*
 * stm32f446_sys.h
 *
 *  Created on: Aug 20, 2025
 *      Author: neoki
 */

#ifndef INC_STM32F446_SYS_H_
#define INC_STM32F446_SYS_H_

#include <stm32f4xx.h>

#define SYSCLK				180000000
#define AHBCLK				180000000
#define APB1CLK				45000000
#define APB2CLK				90000000

#define ABS_DIFF(a, b)		((a > b) ? a - b : b - a)

typedef enum
{
	SYS_OK		= 0UL,
	SYS_ERROR,
	SYS_BUSY,
	SYS_TIMEOUT
}SysError_t;

typedef void (*CallbackFunc_t)(void);

void SYS_NVIC_SetPriority(IRQn_Type irq, uint32_t priority, uint32_t subPriority);

void RCC_Init(void);
void SysTick_Init(void);

void IncTick(void);
uint32_t GetTick(void);

void delay(uint32_t ms);

#endif /* INC_STM32F446_SYS_H_ */
