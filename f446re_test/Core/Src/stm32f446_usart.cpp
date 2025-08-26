/*
 * stm32f446_usart.cpp
 *
 *  Created on: Aug 20, 2025
 *      Author: neoki
 */

#include <stm32f446_usart.h>

USART::USART(USART_TypeDef* husart)
{
	ch 						= husart;
	txdata_buf				= NULL;
	txdata_num				= 0;
	txdata_cnt				= 0;
	tx_states				= SYS_OK;
	Transmit_CmpltCallback 	= NULL;
	rxdata_buf				= NULL;
	rxdata_num				= 0;
	rxdata_cnt				= 0;
	rx_states				= SYS_OK;
	Receive_CmpltCallback	= NULL;
	Receive_OvrunCallback	= NULL;

	switch ((uint32_t)ch) {
		case USART1_BASE:
		case USART6_BASE:
			APBxClk = APB2CLK;
			break;

		case USART2_BASE:
		case USART3_BASE:
		case UART4_BASE:
		case UART5_BASE:
			APBxClk = APB1CLK;
			break;

		default:
			ch = NULL;
			break;
	}
}

USART::USART(USART_TypeDef* husart, GPIOPin_t rx, GPIOPin_t tx)
{
	ch 						= husart;
	txdata_buf				= NULL;
	txdata_num				= 0;
	txdata_cnt				= 0;
	tx_states				= SYS_OK;
	Transmit_CmpltCallback 	= NULL;

	switch ((uint32_t)ch) {
		case USART1_BASE:
		case USART6_BASE:
			APBxClk = APB2CLK;
			break;

		case USART2_BASE:
		case USART3_BASE:
		case UART4_BASE:
		case UART5_BASE:
			APBxClk = APB1CLK;
			break;

		default:
			ch = NULL;
			break;
	}

	rx_pin = rx;
	tx_pin = tx;
}

SysError_t USART::init(uint32_t baudrate)
{
	if(ch == NULL) {
		return SYS_ERROR;
	}

	switch ((uint32_t)ch) {
		case USART1_BASE:
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
			USARTx_IRQn = USART1_IRQn;
			break;

		case USART6_BASE:
			RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
			USARTx_IRQn = USART6_IRQn;
			break;

		case USART2_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
			USARTx_IRQn = USART2_IRQn;
			break;

		case USART3_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
			USARTx_IRQn = USART3_IRQn;
			break;

		case UART4_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
			USARTx_IRQn = UART4_IRQn;
			break;

		case UART5_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
			USARTx_IRQn = UART5_IRQn;
			break;

		default:
			break;
	}

	ch->BRR = APBxClk / baudrate;
	NVIC_EnableIRQ(USARTx_IRQn);

	ch->CR1 &= ~(USART_CR1_OVER8);
	ch->CR1 |= (USART_CR1_TE | USART_CR1_RE);
	ch->CR1 |= USART_CR1_UE;

	pinInit();

	return SYS_OK;
}

SysError_t USART::init(uint32_t baudrate, GPIOPin_t rx, GPIOPin_t tx)
{
	if(ch == NULL) {
		return SYS_ERROR;
	}

	switch ((uint32_t)ch) {
		case USART1_BASE:
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
			USARTx_IRQn = USART1_IRQn;
			break;

		case USART6_BASE:
			RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
			USARTx_IRQn = USART6_IRQn;
			break;

		case USART2_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
			USARTx_IRQn = USART2_IRQn;
			break;

		case USART3_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
			USARTx_IRQn = USART3_IRQn;
			break;

		case UART4_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
			USARTx_IRQn = UART4_IRQn;
			break;

		case UART5_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
			USARTx_IRQn = UART5_IRQn;
			break;

		default:
			break;
	}

	ch->BRR = APBxClk / baudrate;
	NVIC_EnableIRQ(USARTx_IRQn);

	ch->CR1 &= ~(USART_CR1_OVER8);
	ch->CR1 |= (USART_CR1_TE | USART_CR1_RE);
	ch->CR1 |= USART_CR1_UE;

	rx_pin = rx;
	tx_pin = tx;
	pinInit();

	return SYS_OK;
}

void USART::setCallback(USARTCallbackType_t type, CallbackFunc_t func)
{
	switch (type) {
		case USART_TX_COMPLETE:
			Transmit_CmpltCallback = func;
			break;

		case USART_RX_COMPLETE:
			Receive_CmpltCallback = func;
			break;

		case USART_RX_OVERRUN:
			Receive_OvrunCallback = func;
			break;

		default:
			break;
	}
}

SysError_t USART::__send(uint8_t data)
{
	if(ch == NULL) {
		return SYS_ERROR;
	}

	while(!(ch->SR & USART_SR_TXE_Msk));
	ch->DR = data;
	while(!(ch->SR & USART_SR_TC_Msk));

	return SYS_OK;
}

SysError_t USART::transmit(uint8_t* buf, uint16_t size, uint32_t timeout)
{
	uint32_t start = GetTick();

	if(ch == NULL || buf == NULL) {
		return SYS_ERROR;
	}

	for(uint16_t i = 0; i < size; i++) {
		while(!(ch->SR & USART_SR_TXE_Msk)) {
			if((GetTick() - start) > timeout) {
				return SYS_TIMEOUT;
			}
		}

		ch->DR = buf[i];

		while(!(ch->SR & USART_SR_TC_Msk)) {
			if((GetTick() - start) > timeout) {
				return SYS_TIMEOUT;
			}
		}
	}

	return SYS_OK;
}

SysError_t USART::transmit(const char* str, uint32_t timeout)
{
	uint32_t start = GetTick();
	uint16_t i = 0;

	if(ch == NULL || str == NULL) {
		return SYS_ERROR;
	}

	while(str[i] != '\0') {
		while(!(ch->SR & USART_SR_TXE_Msk)) {
			if((GetTick() - start) > timeout) {
				return SYS_TIMEOUT;
			}
		}

		ch->DR = (uint8_t)str[i++];

		while(!(ch->SR & USART_SR_TC_Msk)) {
			if((GetTick() - start) > timeout) {
				return SYS_TIMEOUT;
			}
		}
	}

	return SYS_OK;
}

SysError_t USART::transmitIT(uint8_t* buf, uint16_t size)
{
	if(ch == NULL || buf == NULL) {
		return SYS_ERROR;
	} else if(tx_states == SYS_BUSY) {
		return SYS_BUSY;
	}

	txdata_buf	= buf;
	txdata_num	= size;
	txdata_cnt	= 0;
	tx_states	= SYS_BUSY;
	ch->CR1		|= USART_CR1_TXEIE;

	return SYS_OK;
}

SysError_t USART::receive(uint8_t* buf, uint16_t size, uint32_t timeout)
{
	uint32_t start = GetTick();

	for(uint16_t i = 0; i < size; i++) {
		while(!(ch->SR & USART_SR_RXNE_Msk)) {
			if((GetTick() - start) > timeout) {
				return SYS_TIMEOUT;
			}
		}

		buf[i] = ch->DR;
	}

	return SYS_OK;
}

SysError_t USART::receiveIT(uint8_t* buf, uint16_t size)
{
	if(ch == NULL || buf == NULL) {
		return SYS_ERROR;
	} else if(rx_states == SYS_BUSY) {
		return SYS_BUSY;
	}

	rxdata_buf	= buf;
	rxdata_num	= size;
	rxdata_cnt	= 0;
	rx_states	= SYS_BUSY;
	ch->CR1		|= USART_CR1_RXNEIE;

	return SYS_OK;
}

void USART::IRQ_Handler(void)
{
	uint32_t istates = ch->SR;
	uint32_t cstates = ch->CR1;

	if((istates & USART_SR_TXE_Msk) && (cstates & USART_CR1_TXEIE_Msk)) {
		ch->DR = txdata_buf[txdata_cnt++];

		if(txdata_cnt >= txdata_num) {
			ch->CR1 &= ~USART_CR1_TXEIE;
			tx_states = SYS_OK;

			if(Transmit_CmpltCallback != NULL) {
				Transmit_CmpltCallback();
			}
		}
	}
	if((istates & USART_SR_ORE_Msk) && (cstates & USART_CR1_RXNEIE_Msk)) {
		(void)ch->DR;
		ch->CR1 &= ~USART_CR1_RXNEIE;

		rx_states = SYS_ERROR;
		if(Receive_OvrunCallback != NULL) {
			Receive_OvrunCallback();
		}
	}
	if((istates & USART_SR_RXNE_Msk) && (cstates & USART_CR1_RXNEIE_Msk)) {
		rxdata_buf[rxdata_cnt++] = ch->DR;

		if(rxdata_cnt >= rxdata_num) {
			ch->CR1 &= ~USART_CR1_RXNEIE;

			rx_states = SYS_OK;
			if(Receive_CmpltCallback != NULL) {
				Receive_CmpltCallback();
			}
		}
	}
}

SysError_t USART::print(const char* str)
{
	return transmit(str, 10000);
}

SysError_t USART::println(const char* str)
{
	SysError_t flag1, flag2;
	flag1 = transmit(str, 10000);
	flag2 = transmit("\n\r", 1000);

	if(flag1 == SYS_OK && flag2 == SYS_OK) {
		return SYS_OK;
	} else if(flag1 == SYS_TIMEOUT || flag2 == SYS_TIMEOUT) {
		return SYS_TIMEOUT;
	} else {
		return SYS_ERROR;
	}
}

void USART::pinInit(void)
{
	pinMode(rx_pin, OTHER);
	pinMode(tx_pin, OTHER);

	switch ((uint32_t)ch) {
		case USART1_BASE:
		case USART2_BASE:
		case USART3_BASE:
			AFSelect(rx_pin, AF7);
			AFSelect(tx_pin, AF7);
			break;

		case USART6_BASE:
		case UART4_BASE:
			AFSelect(rx_pin, AF8);
			AFSelect(tx_pin, AF8);
			break;

		case UART5_BASE:
			if(rx_pin == PC12 || rx_pin == PD2) {
				AFSelect(rx_pin, AF8);
			} else {
				AFSelect(rx_pin, AF7);
			}
			if(tx_pin == PC12 || tx_pin == PD2) {
				AFSelect(tx_pin, AF8);
			} else {
				AFSelect(tx_pin, AF7);
			}
			break;

		default:
			break;
	}
}

USART Serial(USART2, PA3, PA2);

extern "C" int __io_putchar(int ch)
{
	if(ch == '\n') {
		Serial.__send('\r');
	}
	Serial.__send((uint8_t)ch);

	return 0;
}
