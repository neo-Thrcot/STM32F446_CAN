/*
 * stm32f446_can.h
 *
 *  Created on: Aug 21, 2025
 *      Author: neoki
 */

#ifndef INC_STM32F446_CAN_H_
#define INC_STM32F446_CAN_H_

#include <stm32f446_sys.h>
#include <stm32f446_gpio.h>

#define CAN_RATE(TS1, TS2, BRP)			((double)APB1CLK / ((TS1 + TS2 + 3) * (BRP + 1)))
#define CAN_SP(TS1, TS2)				(((double)(TS1 + 2) / (TS1 + TS2 + 3)) * 100.0)

typedef enum
{
	SLEEPMODE	= 0UL,
	NORMALMODE,
	TESTMODE
} CanStates_t;

typedef enum
{
	MASK_MODE	= 0UL,
	LIST_MODE
} CANFilterMode_t;

typedef enum
{
	DUAL16BIT	= 0UL,
	SINGLE32BIT
} CANFilterSize_t;

typedef enum
{
	FIFO0 		= 0UL,
	FIFO1
} CANFifo_t;

typedef enum
{
	STID		= 0UL,
	EXID
} CANIDType_t;

typedef enum
{
	DATA_FRAME	= 0UL,
	REMOTE_FRAME
} CANFrameType_t;

typedef enum
{
	CAN_TX_COMPLETE,
	CAN_RX0_COMPLETE,
	CAN_RX1_COMPLETE,
} CANCallbackType_t;

typedef struct
{
	uint8_t		TS1;
	uint8_t		TS2;
	uint8_t		SJW;
	uint16_t	BRP;
	uint64_t	error;
} CANBaudRateVal_t;

typedef struct
{
	uint8_t			filter_num;
	CANFilterMode_t	mode;
	CANFilterSize_t	scale;
	CANFifo_t		fifo_assign;
	bool			active;

	CANIDType_t		id_type;
	CANFrameType_t	frame_type;
	uint32_t		id;
	uint32_t		mask;
} CANFilter_t;

typedef struct
{
	uint32_t		id;
	CANIDType_t		id_type;
	CANFrameType_t	rtr;
	uint8_t			dlc;
	uint8_t			data[8];
} CANTxHeader_t;

typedef struct
{
	uint32_t		id;
	CANIDType_t		id_type;
	CANFrameType_t	rtr;
	uint8_t			dlc;
	uint8_t			data[8];
	uint8_t			filterMach;
} CANRxHeader_t;

class CAN
{
	public:
		CanStates_t			can_state;

		/*init function*/
		CAN(CAN_TypeDef* hcan);
		CAN(CAN_TypeDef* hcan, GPIOPin_t rx, GPIOPin_t tx);
		SysError_t init(uint64_t bitrate);
		SysError_t init(uint64_t bitrate, GPIOPin_t rx, GPIOPin_t tx);
		SysError_t singleFilter(CANFilter_t filter);
		SysError_t allFilter(CANFilter_t* pfilter);
		SysError_t start(void);
		void setCallback(CANCallbackType_t type, CallbackFunc_t func);

		/*transmit function*/
		SysError_t transmit(CANTxHeader_t* txheader, uint32_t timeout);

		/*receive function*/
		SysError_t receive(CANRxHeader_t* rxheader, CANFifo_t fifo);
		SysError_t receiveIT(CANRxHeader_t* rxheader, CANFifo_t fifo);

		/*alternate handler*/
		void TX_IRQHander(void);
		void RX0_IRQHander(void);
		void RX1_IRQHander(void);

	private:
		CAN_TypeDef*		ch;
		IRQn_Type			CANx_TX_IRQn;
		IRQn_Type			CANx_RX0_IRQn;
		IRQn_Type			CANx_RX1_IRQn;
		GPIOPin_t			rx_pin, tx_pin;
		CANRxHeader_t*		prxheader[2];
		bool				rxfifo_standby[2];

		/*callback function*/
		CallbackFunc_t		CAN_RX0ReceivedCallback;
		CallbackFunc_t		CAN_RX1ReceivedCallback;

		/*config function*/
		void pinInit(void);
		CANBaudRateVal_t calcRateVal(uint64_t bitrate);
};



#endif /* INC_STM32F446_CAN_H_ */
