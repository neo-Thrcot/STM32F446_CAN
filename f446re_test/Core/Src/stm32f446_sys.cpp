/*
 * stm32f446_sys.c
 *
 *  Created on: Aug 20, 2025
 *      Author: neoki
 */

#include <stm32f446_sys.h>

static volatile uint32_t msTick = 0;

void SYS_NVIC_SetPriority(IRQn_Type irq, uint32_t priority, uint32_t subPriority)
{
	uint32_t group = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(irq, NVIC_EncodePriority(group, priority, subPriority));
}

void RCC_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;

	/*Clock config*/
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLP_Msk);
	RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos);
	RCC->PLLCFGR |= (360 << RCC_PLLCFGR_PLLN_Pos);
	RCC->PLLCFGR |= (0b00UL << RCC_PLLCFGR_PLLP_Pos);
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

	RCC->CFGR &= ~(RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk);
	RCC->CFGR |= (RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2);

	/*Flash latency config*/
	FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;
	FLASH->ACR |= (FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS);

	/*PLL on, Sysclock:PLL*/
	RCC->CR |= (RCC_CR_HSEBYP | RCC_CR_HSEON);
	while(!(RCC->CR & RCC_CR_HSERDY));
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));

	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
}

void SysTick_Init(void)
{
	SysTick->LOAD = (AHBCLK / 1000) - 1;
	SysTick->VAL = 0;
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void IncTick(void)
{
	msTick++;
}

uint32_t GetTick(void)
{
	return msTick;
}

void delay(uint32_t ms)
{
	uint32_t start = msTick;
	while((msTick - start) < ms);
}
