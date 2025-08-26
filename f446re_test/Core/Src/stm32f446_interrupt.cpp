/*
 * stm32f446_interrupt.c
 *
 *  Created on: Aug 20, 2025
 *      Author: neoki
 */

#include <stm32f446_sys.h>
#include <stm32f446_usart.h>
#include <stm32f446_can.h>

#ifdef __cplusplus
extern "C" {
#endif

//extern CAN Can1;
extern CAN Can2;

void SysTick_Handler(void)
{
	IncTick();
}

void USART2_IRQHandler(void)
{
	Serial.IRQ_Handler();
}

//void CAN1_TX_IRQHander(void)
//{
//	Can1.TX_IRQHander();
//}
//
//void CAN1_RX0_IRQHandler(void)
//{
//	Can1.RX0_IRQHander();
//}
//
//void CAN1_RX1_IRQHandler(void)
//{
//	Can1.RX1_IRQHander();
//}

void CAN2_TX_IRQHander(void)
{
	Can2.TX_IRQHander();
}

void CAN2_RX0_IRQHandler(void)
{
	Can2.RX0_IRQHander();
}

void CAN2_RX1_IRQHandler(void)
{
	Can2.RX1_IRQHander();
}

#ifdef __cplusplus
}
#endif
