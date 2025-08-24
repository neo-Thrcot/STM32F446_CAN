/*
 * stm32f446_gpio.h
 *
 *  Created on: Aug 20, 2025
 *      Author: neoki
 */

#ifndef INC_STM32F446_GPIO_H_
#define INC_STM32F446_GPIO_H_

#include <stm32f446_sys.h>

#define GPIOx_BASE(port)	(AHB1PERIPH_BASE + (port * 0x0400UL))
#define GPIOx(port)			((GPIO_TypeDef*)GPIOx_BASE(port))

#define PORT_NUM(pin)		((pin & 0xF0) >> 4)
#define PIN_NUM(pin)		(pin & 0x0F)

#define LOW					false
#define HIGH				true

typedef enum
{
	PORTA 	= 0U,
	PORTB,
	PORTC,
	PORTD,
	PORTH 	= 7U
} GPIOPort_t;

typedef enum
{
	PA0		= 0U,
	PA1,
	PA2,
	PA3,
	PA4,
	PA5,
	PA6,
	PA7,
	PA8,
	PA9,
	PA10,
	PA11,
	PA12,
	PA13,
	PA14,
	PA15,
	PB0,
	PB1,
	PB2,
	PB3,
	PB4,
	PB5,
	PB6,
	PB7,
	PB8,
	PB9,
	PB10,
	PB12	= 0x1CUL,
	PB13,
	PB14,
	PB15,
	PC0,
	PC1,
	PC2,
	PC3,
	PC4,
	PC5,
	PC6,
	PC7,
	PC8,
	PC9,
	PC10,
	PC11,
	PC12,
	PC13,
	PC14,
	PC15,
	PD2		= 0x32UL,
	PH0		= 0x70UL,
	PH1
} GPIOPin_t;

typedef enum
{
	INPUT = 0b00UL,
	OUTPUT,
	OTHER,
	ANALOG
} GPIOMode_t;

typedef enum
{
	AF0		= 0U,
	AF1,
	AF2,
	AF3,
	AF4,
	AF5,
	AF6,
	AF7,
	AF8,
	AF9,
	AF10,
	AF11,
	AF12,
	AF13,
	AF14,
	AF15
} GPIOAF_t;

typedef enum
{
	PUSHPULL	= 0U,
	OPENDRAIN
} GPIOOutType_t;

typedef enum
{
	LOWSPEED	= 0U,
	MEDSPEED,
	FASTSPEED,
	HIGHSPEED
} GPIOSpeed_t;

void pinMode(GPIOPin_t pin, GPIOMode_t mode);
void pinOutType(GPIOPin_t pin, GPIOOutType_t type);
void pinSpeed(GPIOPin_t pin, GPIOSpeed_t speed);
void AFSelect(GPIOPin_t pin, GPIOAF_t af_num);

void pinWrite(GPIOPin_t pin, bool states);
void portWrite(GPIOPort_t port, uint16_t states);

bool pinRead(GPIOPin_t pin);
uint16_t portRead(GPIOPort_t port);

#endif /* INC_STM32F446_GPIO_H_ */
