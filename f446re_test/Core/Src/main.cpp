#include <stm32f446_sys.h>
#include <stm32f446_gpio.h>
#include <stm32f446_usart.h>
#include <stm32f446_can.h>

#define LD2			PA5
#define B1			PC13

#define LD2_ON			(GPIOA->ODR |= 1UL << 5)
#define LD2_OFF			(GPIOA->ODR &= ~(1UL << 5))
#define LD2_TOGGLE		(GPIOA->ODR ^= (1UL << 5))

#define B1_STATES		((GPIOC->IDR & (1UL << 13)) >> 13)

#define CAN1_BITRATE	(1000000)
#define CAN2_BITRATE	(1000000)

/*USART2 callback function*/
void USART2_Transmit_CmpltCallback(void);
void USART2_Receive_OvrunCallback(void);
void USART2_Receive_CmpltCallback(void);

bool usart2_tx_state = false;
bool usart2_rx_state = false;

CAN Can1(CAN1, PA11, PA12);
//CAN Can2(CAN2, PB5, PB6);

CANFilter_t filter0 = {
		.filter_num 	= 0,
		.mode			= MASK_MODE,
		.scale			= SINGLE32BIT,
		.fifo_assign	= FIFO0,
		.active			= true,

		.id_type		= STID,
		.frame_type		= DATA_FRAME,
		.id				= 0x120,
		.mask			= 0x7FF
};

int main(void)
{
	RCC_Init();
	SysTick_Init();

	pinMode(LD2, OUTPUT);
	pinMode(B1, INPUT);

	Serial.init(115200);
	Serial.setCallback(TX_COMPLETE, USART2_Transmit_CmpltCallback);
	Serial.setCallback(RX_OVERRUN, USART2_Receive_OvrunCallback);
	Serial.setCallback(RX_COMPLETE, USART2_Receive_CmpltCallback);

	Can1.init(CAN1_BITRATE);
	if(Can1.singleFilter(filter0) == SYS_ERROR) {
		Serial.println("CAN1 filter NG!");
	} else {
		Serial.println("CAN1 filter OK!");
	}
	if(Can1.start() == SYS_ERROR) {
		Serial.println("CAN1 can't start!");
	} else {
		Serial.println("CAN1 started!");
	}

	CANTxHeader_t txdata = {
			.id			= 0x120,
			.id_type	= STID,
			.rtr		= DATA_FRAME,
			.dlc		= 8
	};
	txdata.data[0] = 0x00;
	txdata.data[1] = 0x11;
	txdata.data[2] = 0x22;
	txdata.data[3] = 0x33;
	txdata.data[4] = 0x44;
	txdata.data[5] = 0x55;
	txdata.data[6] = 0x66;
	txdata.data[7] = 0x77;

	while(1)
	{
		CANRxHeader_t rxdata;
		if(Can1.receive(&rxdata, FIFO0) == SYS_OK) {
			Serial.print("ID:");
			Serial.print(rxdata.id);
			for(uint8_t i = 0; i < rxdata.dlc; i++) {
				Serial.print("   ");
				Serial.print(rxdata.data[i]);
			}
			Serial.println("");
		}
	}

	return 0;
}

void USART2_Transmit_CmpltCallback(void)
{
	usart2_tx_state = true;
}

void USART2_Receive_OvrunCallback(void)
{
	usart2_rx_state = true;
}

void USART2_Receive_CmpltCallback(void)
{
	usart2_rx_state = true;
}
