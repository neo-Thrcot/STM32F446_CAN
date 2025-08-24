/*
 * stm32f446_can.cpp
 *
 *  Created on: Aug 21, 2025
 *      Author: neoki
 */

#include <stm32f446_can.h>

static bool	filter_flag = false;

CAN::CAN(CAN_TypeDef *hcan)
{
	ch			= hcan;
	can_state	= SLEEPMODE;
}

CAN::CAN(CAN_TypeDef* hcan, GPIOPin_t rx, GPIOPin_t tx)
{
	ch			= hcan;
	can_state	= SLEEPMODE;
	rx_pin		= rx;
	tx_pin		= tx;
}

SysError_t CAN::init(uint64_t bitrate)
{
	CANBaudRateVal_t rate;

	switch ((uint32_t)ch) {
		case CAN1_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
			CANx_TX_IRQn	= CAN1_TX_IRQn;
			CANx_RX0_IRQn	= CAN1_RX0_IRQn;
			CANx_RX1_IRQn	= CAN1_RX1_IRQn;
			break;

		case CAN2_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
			CANx_TX_IRQn	= CAN2_TX_IRQn;
			CANx_RX0_IRQn	= CAN2_RX0_IRQn;
			CANx_RX1_IRQn	= CAN2_RX1_IRQn;
			break;

		default:
			return SYS_ERROR;
			break;
	}

	pinInit();

	ch->MCR &= ~CAN_MCR_SLEEP;		while(ch->MSR & CAN_MSR_SLAK_Msk);
	ch->MCR |= CAN_MCR_INRQ;		while(!(ch->MSR & CAN_MSR_INAK_Msk));
	ch->MCR &= ~(CAN_MCR_TTCM | CAN_MCR_AWUM | CAN_MCR_NART | CAN_MCR_RFLM | CAN_MCR_TXFP);
	ch->MCR |= CAN_MCR_ABOM;

	rate = calcRateVal(bitrate);
	if(rate.BRP == 0 && rate.TS1 == 0 && rate.TS2 == 0) {
		return SYS_ERROR;
	}

	ch->BTR &= ~(CAN_BTR_SJW | CAN_BTR_TS2 | CAN_BTR_TS1 | CAN_BTR_BRP);
	ch->BTR |= ((uint32_t)rate.SJW << CAN_BTR_SJW_Pos |
				(uint32_t)rate.TS2 << CAN_BTR_TS2_Pos |
				(uint32_t)rate.TS1 << CAN_BTR_TS1_Pos |
				(uint32_t)rate.BRP << CAN_BTR_BRP_Pos);

	NVIC_EnableIRQ(CANx_TX_IRQn);
	NVIC_EnableIRQ(CANx_RX0_IRQn);
	NVIC_EnableIRQ(CANx_RX1_IRQn);

	return SYS_OK;
}

SysError_t CAN::init(uint64_t bitrate, GPIOPin_t rx, GPIOPin_t tx)
{
	CANBaudRateVal_t rate;

	rx_pin = rx;
	tx_pin = tx;

	switch ((uint32_t)ch) {
		case CAN1_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
			CANx_TX_IRQn	= CAN1_TX_IRQn;
			CANx_RX0_IRQn	= CAN1_RX0_IRQn;
			CANx_RX1_IRQn	= CAN1_RX1_IRQn;
			break;

		case CAN2_BASE:
			RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
			CANx_TX_IRQn	= CAN2_TX_IRQn;
			CANx_RX0_IRQn	= CAN2_RX0_IRQn;
			CANx_RX1_IRQn	= CAN2_RX1_IRQn;
			break;

		default:
			return SYS_ERROR;
			break;
	}

	pinInit();

	ch->MCR &= ~CAN_MCR_SLEEP;		while(ch->MSR & CAN_MSR_SLAK_Msk);
	ch->MCR |= CAN_MCR_INRQ;		while(!(ch->MSR & CAN_MSR_INAK_Msk));
	ch->MCR &= ~(CAN_MCR_TTCM | CAN_MCR_AWUM | CAN_MCR_NART | CAN_MCR_RFLM | CAN_MCR_TXFP);
	ch->MCR |= CAN_MCR_ABOM;

	rate = calcRateVal(bitrate);
	if(rate.BRP == 0 && rate.TS1 == 0 && rate.TS2 == 0) {
		return SYS_ERROR;
	}

	ch->BTR &= ~(CAN_BTR_SJW | CAN_BTR_TS2 | CAN_BTR_TS1 | CAN_BTR_BRP);
	ch->BTR |= ((uint32_t)rate.SJW << CAN_BTR_SJW_Pos |
				(uint32_t)rate.TS2 << CAN_BTR_TS2_Pos |
				(uint32_t)rate.TS1 << CAN_BTR_TS1_Pos |
				(uint32_t)rate.BRP << CAN_BTR_BRP_Pos);

	NVIC_EnableIRQ(CANx_TX_IRQn);
	NVIC_EnableIRQ(CANx_RX0_IRQn);
	NVIC_EnableIRQ(CANx_RX1_IRQn);

	return SYS_OK;
}

SysError_t CAN::singleFilter(CANFilter_t filter)
{
	uint32_t act_filter_num;

	if(ch == NULL) {
		return SYS_ERROR;
	} else if(filter.filter_num >= 14) {
		return SYS_ERROR;
	}

	if((uint32_t)ch == CAN1_BASE) {
		act_filter_num = filter.filter_num;
	} else if((uint32_t)ch == CAN2_BASE) {
		act_filter_num = filter.filter_num + 14;
	} else {
		return SYS_ERROR;
	}

	CAN1->FMR |= CAN_FMR_FINIT;
	CAN1->FA1R &= ~(1UL << act_filter_num);

	if(filter_flag == false) {
		filter_flag = true;

		CAN1->FMR &= ~CAN_FMR_CAN2SB;
		CAN1->FMR |= (14UL << CAN_FMR_CAN2SB_Pos);
	}

	if(filter.mode == MASK_MODE) {
		CAN1->FM1R &= ~(1UL << act_filter_num);
	} else {
		CAN1->FM1R |= (1UL << act_filter_num);
	}

	if(filter.fifo_assign == FIFO0) {
		CAN1->FFA1R &= ~(1UL << act_filter_num);
	} else {
		CAN1->FFA1R |= (1UL << act_filter_num);
	}

	CAN1->sFilterRegister[act_filter_num].FR1 = 0;
	CAN1->sFilterRegister[act_filter_num].FR2 = 0;
	if(filter.scale == DUAL16BIT) {
		CAN1->FS1R &= ~(1UL << act_filter_num);

		CAN1->sFilterRegister[act_filter_num].FR1 |= ((filter.id & 0x07FF0000UL) << 5);
		CAN1->sFilterRegister[act_filter_num].FR1 |= ((filter.id & 0x00007FFUL) << 5);
		CAN1->sFilterRegister[act_filter_num].FR2 |= ((filter.mask & 0x07FF0000UL) << 5);
		CAN1->sFilterRegister[act_filter_num].FR2 |= ((filter.mask & 0x000007FFUL) << 5);

		if(filter.id_type == STID) {
			CAN1->sFilterRegister[act_filter_num].FR1 &= ~(1UL << 19);
			CAN1->sFilterRegister[act_filter_num].FR1 &= ~(1UL << 3);
			CAN1->sFilterRegister[act_filter_num].FR2 &= ~(1UL << 19);
			CAN1->sFilterRegister[act_filter_num].FR2 &= ~(1UL << 3);
		} else {
			CAN1->sFilterRegister[act_filter_num].FR1 |= (((filter.id & (0b111UL << 27)) >> 11) | (1UL << 19));
			CAN1->sFilterRegister[act_filter_num].FR1 |= (((filter.id & (0b111UL << 11)) >> 11) | (1UL << 3));
			CAN1->sFilterRegister[act_filter_num].FR2 |= (((filter.mask & (0b111UL << 27)) >> 11) | (1UL << 19));
			CAN1->sFilterRegister[act_filter_num].FR2 |= (((filter.mask & (0b111UL << 11)) >> 11) | (1UL << 3));
		}

		if(filter.frame_type == DATA_FRAME) {
			CAN1->sFilterRegister[act_filter_num].FR1 &= ~(1UL << 20);
			CAN1->sFilterRegister[act_filter_num].FR1 &= ~(1UL << 4);
			CAN1->sFilterRegister[act_filter_num].FR2 &= ~(1UL << 20);
			CAN1->sFilterRegister[act_filter_num].FR2 &= ~(1UL << 4);
		} else {
			CAN1->sFilterRegister[act_filter_num].FR1 |= (1UL << 20);
			CAN1->sFilterRegister[act_filter_num].FR1 |= (1UL << 4);
			CAN1->sFilterRegister[act_filter_num].FR2 |= (1UL << 20);
			CAN1->sFilterRegister[act_filter_num].FR2 |= (1UL << 4);
		}
	} else {
		CAN1->FS1R |= (1UL << act_filter_num);

		if(filter.id_type == STID) {
			CAN1->sFilterRegister[act_filter_num].FR1 |= ((filter.id & 0x7FF) << 21);
			CAN1->sFilterRegister[act_filter_num].FR2 |= ((filter.mask & 0x7FF) << 21);
		} else {
			CAN1->sFilterRegister[act_filter_num].FR1 |= ((filter.id & 0x1FFFFFFF) << 3) | (1UL << 2);
			CAN1->sFilterRegister[act_filter_num].FR2 |= ((filter.mask & 0x1FFFFFFF) << 3) | (1UL << 2);

			if(filter.frame_type == DATA_FRAME) {
				CAN1->sFilterRegister[act_filter_num].FR1 &= ~(1UL << 1);
				CAN1->sFilterRegister[act_filter_num].FR2 &= ~(1UL << 1);
			} else {
				CAN1->sFilterRegister[act_filter_num].FR1 |= (1UL << 1);
				CAN1->sFilterRegister[act_filter_num].FR2 |= (1UL << 1);
			}
		}
	}

	if(filter.active == true) {
		CAN1->FA1R |= (1UL << act_filter_num);
	}

	return SYS_OK;
}

SysError_t CAN::allFilter(CANFilter_t* pfilter)
{
	if(ch == NULL || pfilter == NULL) {
		return SYS_ERROR;
	}

	for(uint8_t i = 0; i < 14; i++) {
		if(singleFilter(pfilter[i]) == SYS_ERROR) {
			return SYS_ERROR;
		}
	}

	return SYS_OK;
}

SysError_t CAN::start(void)
{
	uint32_t start = 0;

	if(ch == NULL || !(ch->MSR & CAN_MSR_INAK_Msk)) {
		return SYS_ERROR;
	}

	if(filter_flag == true) {
		CAN1->FMR &= ~CAN_FMR_FINIT;
	}

	start = GetTick();
	ch->MCR &= ~CAN_MCR_INRQ;
	while((ch->MSR & CAN_MSR_INAK_Msk) == CAN_MSR_INAK) {
		if((GetTick() - start) > 10000) {
			return SYS_ERROR;
		}
	}

	can_state = NORMALMODE;

	return SYS_OK;
}

SysError_t CAN::transmit(CANTxHeader_t *txheader, uint32_t timeout)
{
	uint32_t tsr;
	uint32_t start;
	uint32_t TXOKx_pos;
	uint8_t mailbox;

	if(ch == NULL || txheader == NULL) {
		return SYS_ERROR;
	} else if(txheader->dlc > 8) {
		return SYS_ERROR;
	}

	tsr = ch->TSR;
	if((tsr & CAN_TSR_TME0_Msk) || (tsr & CAN_TSR_TME1_Msk) || (tsr & CAN_TSR_TME2_Msk)) {
		mailbox = (tsr & CAN_TSR_CODE_Msk) >> CAN_TSR_CODE_Pos;
		if(mailbox == 0b11) {
			return SYS_BUSY;
		}

		if(txheader->id_type == STID) {
			ch->sTxMailBox[mailbox].TIR = (txheader->id << CAN_TI0R_STID_Pos);
		} else {
			ch->sTxMailBox[mailbox].TIR = (txheader->id << CAN_TI0R_EXID_Pos) | (1UL << CAN_TI0R_IDE_Pos);
		}

		if(txheader->rtr == REMOTE_FRAME) {
			ch->sTxMailBox[mailbox].TIR |= (1UL << CAN_TI0R_RTR_Pos);
		}

		ch->sTxMailBox[mailbox].TDTR &= ~CAN_TDT0R_DLC_Msk;
		ch->sTxMailBox[mailbox].TDTR |= txheader->dlc;

		ch->sTxMailBox[mailbox].TDLR = (txheader->data[0] << CAN_TDL0R_DATA0_Pos) |
									   (txheader->data[1] << CAN_TDL0R_DATA1_Pos) |
									   (txheader->data[2] << CAN_TDL0R_DATA2_Pos) |
									   (txheader->data[3] << CAN_TDL0R_DATA3_Pos);
		ch->sTxMailBox[mailbox].TDHR = (txheader->data[4] << CAN_TDH0R_DATA4_Pos) |
									   (txheader->data[5] << CAN_TDH0R_DATA5_Pos) |
									   (txheader->data[6] << CAN_TDH0R_DATA6_Pos) |
									   (txheader->data[7] << CAN_TDH0R_DATA7_Pos);
		ch->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;

		start = GetTick();
		while(!(ch->TSR & (1UL << (CAN_TSR_TME0_Pos + mailbox)))) {
			if((GetTick() - start) > timeout) {
				return SYS_TIMEOUT;
			}
		}

		switch (mailbox) {
			case 0:
				TXOKx_pos = CAN_TSR_TXOK0;
				break;

			case 1:
				TXOKx_pos = CAN_TSR_TXOK1;
				break;

			case 2:
				TXOKx_pos = CAN_TSR_TXOK2;
				break;

			default:
				break;
		}

		if(!(ch->TSR & (1UL << TXOKx_pos))) {
			return SYS_ERROR;
		}
	} else {
		return SYS_BUSY;
	}

	return SYS_OK;
}

SysError_t CAN::receive(CANRxHeader_t* rxheader, CANFifo_t fifo)
{
	if(ch == NULL || rxheader == NULL) {
		return SYS_ERROR;
	}

	if(fifo == FIFO0) {
		if((ch->RF0R & CAN_RF0R_FMP0_Msk) == 0UL) {
			return SYS_ERROR;
		}
	} else if (fifo == FIFO1) {
		if((ch->RF1R & CAN_RF1R_FMP1_Msk) == 0UL) {
			return SYS_ERROR;
		}
	} else {
		return SYS_ERROR;
	}

	if(ch->sFIFOMailBox[fifo].RIR & CAN_RI0R_IDE_Msk) {
		rxheader->id_type = EXID;
		rxheader->id = (ch->sFIFOMailBox[fifo].RIR & CAN_RI0R_EXID_Msk) >> CAN_RI0R_EXID_Pos;
	} else {
		rxheader->id_type = STID;
		rxheader->id = (ch->sFIFOMailBox[fifo].RIR & CAN_RI0R_STID_Msk) >> CAN_RI0R_STID_Pos;
	}

	rxheader->dlc = (ch->sFIFOMailBox[fifo].RDTR & CAN_RDT0R_DLC_Msk) >> CAN_RDT0R_DLC_Pos;
	rxheader->filterMach = (ch->sFIFOMailBox[fifo].RDTR & CAN_RDT0R_FMI_Msk) >> CAN_RDT0R_FMI_Pos;
	if(ch->sFIFOMailBox[fifo].RIR & CAN_RI0R_RTR_Msk) {
		rxheader->rtr = REMOTE_FRAME;
	} else {
		rxheader->rtr = DATA_FRAME;

		rxheader->data[0] = (uint8_t)((ch->sFIFOMailBox[fifo].RDLR & CAN_RDL0R_DATA0) >> CAN_RDL0R_DATA0_Pos);
		rxheader->data[1] = (uint8_t)((ch->sFIFOMailBox[fifo].RDLR & CAN_RDL0R_DATA1) >> CAN_RDL0R_DATA1_Pos);
		rxheader->data[2] = (uint8_t)((ch->sFIFOMailBox[fifo].RDLR & CAN_RDL0R_DATA2) >> CAN_RDL0R_DATA2_Pos);
		rxheader->data[3] = (uint8_t)((ch->sFIFOMailBox[fifo].RDLR & CAN_RDL0R_DATA3) >> CAN_RDL0R_DATA3_Pos);
		rxheader->data[4] = (uint8_t)((ch->sFIFOMailBox[fifo].RDHR & CAN_RDH0R_DATA4) >> CAN_RDH0R_DATA4_Pos);
		rxheader->data[5] = (uint8_t)((ch->sFIFOMailBox[fifo].RDHR & CAN_RDH0R_DATA5) >> CAN_RDH0R_DATA5_Pos);
		rxheader->data[6] = (uint8_t)((ch->sFIFOMailBox[fifo].RDHR & CAN_RDH0R_DATA6) >> CAN_RDH0R_DATA6_Pos);
		rxheader->data[7] = (uint8_t)((ch->sFIFOMailBox[fifo].RDHR & CAN_RDH0R_DATA7) >> CAN_RDH0R_DATA7_Pos);
	}

	if(fifo == FIFO0) {
		ch->RF0R |= CAN_RF0R_RFOM0;
	} else {
		ch->RF1R |= CAN_RF1R_RFOM1;
	}

	return SYS_OK;
}

void CAN::pinInit(void)
{
	pinMode(rx_pin, OTHER);
	pinMode(tx_pin, OTHER);

	AFSelect(rx_pin, AF9);
	AFSelect(tx_pin, AF9);

	pinSpeed(rx_pin, HIGHSPEED);
	pinSpeed(tx_pin, HIGHSPEED);
}

CANBaudRateVal_t CAN::calcRateVal(uint64_t bitrate)
{
	uint64_t calc_rate = 0;
	uint64_t rate_error = 0;
	uint64_t allow_error = bitrate * 0.002;
	double sample_point = 0.0;
	bool allow_flag = false;

	CANBaudRateVal_t rate_pattern1 = {0, 0, 0, 0, 0};
	CANBaudRateVal_t rate_pattern2 = {0, 0, 0, 0, 0};

	for(uint16_t brp = 0; brp < 1024; brp++) {
		for(uint8_t ts1 = 0; ts1 < 16; ts1++) {
			for(uint8_t ts2 = 0; ts2 < 8 && ts2 <= ts1; ts2++) {
				calc_rate = CAN_RATE(ts1, ts2, brp);
				rate_error = ABS_DIFF(calc_rate, bitrate);

				if(rate_error <= allow_error) {
					sample_point = CAN_SP(ts1, ts2);

					if(sample_point >= 75.0 && sample_point <= 85.0) {
						rate_pattern1.BRP = brp;
						rate_pattern1.TS1 = ts1;
						rate_pattern1.TS2 = ts2;
						rate_pattern1.error = rate_error;

						if(rate_pattern1.error == 0) {
							if(rate_pattern1.TS2 == 0) {
								rate_pattern1.SJW = 0;
							} else {
								rate_pattern1.SJW = 1;
							}

							return rate_pattern1;
						} else if(allow_flag == false) {
							rate_pattern2 = rate_pattern1;
							allow_flag = true;
						} else if(rate_pattern1.error < rate_pattern2.error
								  || (rate_pattern1.error == rate_pattern2.error && rate_pattern1.BRP < rate_pattern2.BRP )) {
							rate_pattern2 = rate_pattern1;
						}
					}
				}
			}
		}
	}

	if(rate_pattern2.TS2 == 0) {
		rate_pattern2.SJW = 0;
	} else {
		rate_pattern2.SJW = 1;
	}

	return rate_pattern2;
}
