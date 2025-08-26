/*
 * stm32f446_usart.h
 *
 *  Created on: Aug 20, 2025
 *      Author: neoki
 */

#ifndef INC_STM32F446_USART_H_
#define INC_STM32F446_USART_H_

#include <string>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <type_traits>
#include <stm32f446_sys.h>
#include <stm32f446_gpio.h>

typedef enum
{
	USART_TX_COMPLETE,
	USART_RX_COMPLETE,
	USART_RX_OVERRUN
} USARTCallbackType_t;

typedef enum
{
	OCT	= 8,
	DEC = 10,
	HEX = 16
} NumFormat_t;

class USART
{
	public:
		/*setting*/
		USART(USART_TypeDef* husart);
		USART(USART_TypeDef* husart, GPIOPin_t rx, GPIOPin_t tx);
		SysError_t init(uint32_t baudrate);
		SysError_t init(uint32_t baudrate, GPIOPin_t rx, GPIOPin_t tx);
		void setCallback(USARTCallbackType_t type, CallbackFunc_t func);

		/*transmit*/
		SysError_t __send(uint8_t data);
		SysError_t transmit(uint8_t* buf, uint16_t size, uint32_t timeout);
		SysError_t transmit(const char* str, uint32_t timeout);
		SysError_t transmitIT(uint8_t* buf, uint16_t size);

		/*receive*/
		SysError_t receive(uint8_t* buf, uint16_t size, uint32_t timeout);
		SysError_t receiveIT(uint8_t* buf, uint16_t size);

		/*string print*/
		SysError_t print(const char* str);
		SysError_t println(const char* str);

		template <typename T>
		typename std::enable_if<std::is_integral<T>::value, SysError_t>::type
		print(T val)
		{
			std::string str = std::to_string(val);
			return transmit(str.c_str(), 10000);
		}

//		template <typename T>
//		typename std::enable_if<std::is_integral<T>::value, SysError_t>::type
//		print(T val, NumFormat_t format)
//		{
//			std::string str;
//			std::stringstream ss;
//
//			switch (format) {
//				case OCT:
//					ss << std::oct << static_cast<unsigned int>(val);
//					str = ss.str();
//					break;
//
//				case DEC:
//					str = std::to_string(val);
//					break;
//
//				case HEX:
//					ss << std::hex << static_cast<unsigned int>(val);
//					str = ss.str();
//					break;
//
//				default:
//					break;
//			}
//
//			return transmit(str.c_str(), 10000);
//		}

		template <typename T>
		typename std::enable_if<std::is_floating_point<T>::value, SysError_t>::type
		print(T val)
		{
			char str[32];
			if(snprintf(str, 32, "%f", val) < 0) {
				return SYS_ERROR;
			} else {
				return transmit(str, 10000);
			}
		}

		template <typename T>
		typename std::enable_if<std::is_integral<T>::value, SysError_t>::type
		println(T val)
		{
			SysError_t flag1, flag2;
			std::string str = std::to_string(val);

			flag1 = transmit(str.c_str(), 10000);
			flag2 = transmit("\n\r", 1000);

			if(flag1 == SYS_OK && flag2 == SYS_OK) {
				return SYS_OK;
			} else if(flag1 == SYS_TIMEOUT || flag2 == SYS_TIMEOUT) {
				return SYS_TIMEOUT;
			} else {
				return SYS_ERROR;
			}
		}

//		template <typename T>
//		typename std::enable_if<std::is_integral<T>::value, SysError_t>::type
//		println(T val, NumFormat_t format)
//		{
//			SysError_t flag1, flag2;
//			std::string str;
//			std::stringstream ss;
//
//			switch (format) {
//				case OCT:
//					ss << std::oct << val;
//					str = ss.str();
//					break;
//
//				case DEC:
//					str = std::to_string(val);
//					break;
//
//				case HEX:
//					ss << std::hex << val;
//					str = ss.str();
//					break;
//
//				default:
//					return SYS_ERROR;
//					break;
//			}
//
//			flag1 = transmit(str.c_str(), 10000);
//			flag2 = transmit("\n\r", 1000);
//
//			if(flag1 == SYS_OK && flag2 == SYS_OK) {
//				return SYS_OK;
//			} else if(flag1 == SYS_TIMEOUT || flag2 == SYS_TIMEOUT) {
//				return SYS_TIMEOUT;
//			} else {
//				return SYS_ERROR;
//			}
//		}

		template <typename T>
		typename std::enable_if<std::is_floating_point<T>::value, SysError_t>::type
		println(T val)
		{
			char str[32];
			SysError_t flag1, flag2;

			if(snprintf(str, 32, "%f", val) < 0) {
				return SYS_ERROR;
			} else {
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
		}

		/*alternate handler*/
		void IRQ_Handler(void);

	private:
		USART_TypeDef* 	ch;
		IRQn_Type		USARTx_IRQn;
		uint32_t 		APBxClk;
		GPIOPin_t 		rx_pin, tx_pin;

		/*Transmit interrupt*/
		volatile uint8_t		*txdata_buf;
		volatile uint16_t		txdata_num;
		volatile uint16_t		txdata_cnt;
		volatile SysError_t		tx_states;
		CallbackFunc_t			Transmit_CmpltCallback;

		/*Receive interrupt*/
		volatile uint8_t		*rxdata_buf;
		volatile uint16_t		rxdata_num;
		volatile uint16_t		rxdata_cnt;
		volatile SysError_t		rx_states;
		CallbackFunc_t			Receive_CmpltCallback;
		CallbackFunc_t			Receive_OvrunCallback;

		/*GPIO config function*/
		void pinInit(void);
};

extern "C" int __io_putchar(int ch);

extern USART Serial;

#endif /* INC_STM32F446_USART_H_ */
