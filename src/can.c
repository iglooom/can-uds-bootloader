/*
 * can.c
 *
 *  Created on: 2 jan 2017 г.
 *      Author: gl
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencmsis/core_cm3.h>
#include <libopencm3/stm32/flash.h>
#include "can.h"
#include "main.h"
#include "stddef.h"

canMsg TxMessage;
canMsg RxMessage;

volatile uint8_t bootloaderActive;

uint32_t memoryAddress;
uint32_t memorySize;

uint16_t writeCrc;

volatile uint8_t eraseFlag;
volatile uint8_t writeLen;
volatile uint8_t endFlag;
volatile uint8_t blockSequenceCounter;

uint8_t CANRxDataBuf[256];
volatile uint16_t CANRxCurrLen;
volatile uint16_t CANRxCurrWr;
//uint16_t CANRxCurrRd;
uint8_t *CANRxCurrDataPtr;
uint32_t CANRxCurrFlash;

enum {noSession, activeSession, downloadRequested} programmingSessionStatus;

void can_setup()
{
    CANRxCurrLen = 0;
	CANRxCurrWr = 0;
	writeCrc = 0;
	eraseFlag = 0;
	writeLen = 0;
	programmingSessionStatus = noSession;
	blockSequenceCounter = 0;
	endFlag = 0;
	bootloaderActive = 0;

	TxMessage.DLC = 8;
	TxMessage.Id = CANID_TX;
	TxMessage.CanPort = CAN1;

    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_CAN1);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    // CAN1
    /* Configure CAN1 pin: RX (input pull-up). */
    gpio_set_mode(GPIO_BANK_CAN1_RX, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_RX);
    gpio_set(GPIO_BANK_CAN1_RX, GPIO_CAN1_RX);

    /* Configure CAN1 pin: TX. */
    gpio_set_mode(GPIO_BANK_CAN1_TX, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_TX);

    /* NVIC setup. */
    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
    nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1);

    /* Reset CAN. */
    can_reset(CAN1);

    /* CAN1 cell init. 500 kbit/s */
    if (can_init(CAN1,
                 false,           /* TTCM: Time triggered comm mode? */
                 true,            /* ABOM: Automatic bus-off management? */
                 true,           /* AWUM: Automatic wakeup mode? */
                 false,           /* NART: No automatic retransmission? */
                 false,           /* RFLM: Receive FIFO locked mode? */
                 true,           /* TXFP: Transmit FIFO priority? */
                 CAN_BTR_SJW_1TQ,
                 CAN_BTR_TS1_8TQ,
                 CAN_BTR_TS2_7TQ,
                 4,
                 false,
                 false))             /* BRP+1: Baud rate prescaler */
    {
        /* Die because we failed to initialize. */
        while (1)
            __asm__("nop");
    }

    /* CAN filter 0 init. */
    can_filter_id_list_16bit_init(
            0,
            (CANID_RX << 5),
            (CANID_RX << 5),
            (CANID_RX << 5),
            (CANID_RX << 5),
            0,
            true);

    /* Enable CAN RX interrupt. */
    can_enable_irq(CAN1, CAN_IER_FMPIE0);
}

void CANProcessPacket(uint8_t *dataBuf, uint8_t len) // Process received packet
{
	if(len == 0)
	{
		CANRxError();
		return;
	}
	CANRxCurrWr = 0;
	CANRxCurrLen = 0;

	uint8_t serviceId = dataBuf[0];
	uint8_t sendBuf[8];
	uint8_t errCode = 0;

	bootloaderActive = 1;

	switch(serviceId)
	{
	case 0x10: // diagnosticSessionControl
		if(dataBuf[1] == 0x02) // programmingSession
		{
			flash_unlock();
			sendBuf[0] = 0x02;
			sendBuf[1] = 0x0F;
			sendBuf[2] = 0x0F;
			sendBuf[3] = 0x0F;
			sendBuf[4] = 0x0F;
			programmingSessionStatus = activeSession;
			CANAnswer(0x50, sendBuf, 5);
		}else
			errCode = 0x12; // subFunctionNotSupported
		break;
	case 0x11: // ECUReset
		if(dataBuf[1] == 0x01) // HardReset
		{
			sendBuf[0] = 0x01;
			CANAnswer(0x51, sendBuf, 1);
			iwdg_start();
		}else
			errCode = 0x22; // conditionsNotCorrect
		break;
	case 0x22: // ReadDataByIdentifier
		if(dataBuf[1] == 0xF1 && dataBuf[2] == 0x80) // bootSoftwareIdentificationDataIdentifier
		{
			sendBuf[0] = dataBuf[1];
			sendBuf[1] = dataBuf[2];
			sendBuf[2] = 'B';
			sendBuf[3] = '1';
			sendBuf[4] = '-';
			sendBuf[5] = '0';
			CANAnswer(0x62, sendBuf, 6);
		}else
			errCode = 0x31; // requestOutOfRange
		break;
	case 0x34: // RequestDownload
		if(programmingSessionStatus != activeSession)
		{
			errCode = 0x22; // conditionsNotCorrect
		}else
		{
			//uint8_t dataFormatIdentifier = dataBuf[1];
			//uint8_t addressAndLengthFormatIdentifier = dataBuf[2];
			if((dataBuf[1] == 0x00 || dataBuf[1] == 0x01) && dataBuf[2] == 0x44)
			{
				if(len == 11)
				{
					//needDescramble = dataBuf[1];
					memoryAddress = dataBuf[3]<<24;
					memoryAddress += dataBuf[4]<<16;
					memoryAddress += dataBuf[5]<<8;
					memoryAddress += dataBuf[6];
					memorySize = dataBuf[7]<<24;;
					memorySize += dataBuf[8]<<16;;
					memorySize += dataBuf[9]<<8;;
					memorySize += dataBuf[10];
					
                    if(memoryAddress >= PROG_START_ADDR
                            && memoryAddress <= PROG_END_ADDR
                            && memoryAddress + memorySize <= PROG_END_ADDR)
                    {
                        programmingSessionStatus = downloadRequested;

                        // Erase Flash
                        eraseFlag = 1;
                        CANRxCurrFlash = memoryAddress;
                        writeCrc = 0;

                        sendBuf[0] = 0x10;
                        sendBuf[1] = 0xFA; // Max chunk size
                        blockSequenceCounter = 0x01;
                        CANAnswer(0x74, sendBuf, 2);
                    }else
                    {
                        errCode = 0x31; // requestOutOfRange
                    }
				}else
				{
					errCode = 0x13; // incorrectMessageLengthOrInvalidFormat
				}
			}else
				errCode = 0x31; // requestOutOfRange
		}
		break;
	case 0x36: // TransferData
		if(programmingSessionStatus != downloadRequested)
		{
			errCode = 0x24; // requestSequenceError
		}else
		{
			if(blockSequenceCounter == dataBuf[1])
			{
				// Write to flash
				CANRxCurrDataPtr = &dataBuf[2];
				writeLen = len-2;
			}else
				errCode = 0x73; // wrongBlockSequenceCounter
		}
		break;
	case 0x37: // RequestTransferExit
		if(programmingSessionStatus != downloadRequested)
		{
			errCode = 0x24; // requestSequenceError
		}else
		{
			endFlag = 1;
		}
		break;
	case 0x3E: // TesterPresent
		sendBuf[0] = 0x00;
		CANAnswer(0x7E, sendBuf, 1);
		break;
	default:
		errCode = 0x12; // subFunctionNotSupported
		break;
	}
	if(errCode) // Negative
	{
		sendBuf[0] = serviceId;
		sendBuf[1] = errCode;
		CANAnswer(0x7F, sendBuf, 2);
	}
}

void CANAnswer(uint8_t serviceId, uint8_t *dataBuf, uint8_t len)
{
	if(len>6)
		return;

	TxMessage.Data[0] = len+1;
	TxMessage.Data[1] = serviceId;
	TxMessage.Data[2] = 0x00;
	TxMessage.Data[3] = 0x00;
	TxMessage.Data[4] = 0x00;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;

	for(int i=2;i<len+2;i++)
		TxMessage.Data[i] = dataBuf[i-2];

    can_transmit(CAN1,TxMessage.Id,false,false,TxMessage.DLC,TxMessage.Data);
}

void CANRxPacket(uint8_t *dataBuf, uint8_t currlen)
{
	for(int i=0;i<currlen;i++)
	{
		CANRxDataBuf[CANRxCurrWr++] = dataBuf[i];
	}
	if(CANRxCurrWr >= CANRxCurrLen)
		CANProcessPacket(CANRxDataBuf,CANRxCurrLen);
}

inline void CANRxError()
{
	CANRxCurrLen = 0;
}

void CANSendFC(uint8_t fs, uint8_t bs, uint8_t stmin)
{
	TxMessage.Data[0] = 0x30 + (fs & 0xF);
	TxMessage.Data[1] = bs;
	TxMessage.Data[2] = stmin;
	TxMessage.Data[3] = 0x00;
	TxMessage.Data[4] = 0x00;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
    can_transmit(CAN1,TxMessage.Id,false,false,TxMessage.DLC,TxMessage.Data);
}

void usb_lp_can_rx0_isr(void)
{
    bool ext, rtr;
    uint8_t fmi;

    can_receive(CAN1, 0, true, (uint32_t*)&RxMessage.Id, &ext, &rtr, &fmi, &RxMessage.DLC, RxMessage.Data, NULL);

	static uint16_t RxLen = 0;
	static uint8_t CF_SN = 0;
	static uint8_t sendBuf[8];

	switch((RxMessage.Data[0]) >> 4) // PCIType
	{
	case 2: // ConsecutiveFrame
		if(CF_SN == (RxMessage.Data[0] & 0xF) && CANRxCurrLen)
		{
			CANRxPacket(&RxMessage.Data[1], 7);
			if(++CF_SN>=0x10)
				CF_SN = 0;
		}else
		{
			CANRxError();
		}
		break;
	case 0: // SingleFrame
		RxLen = RxMessage.Data[0];
		CANProcessPacket(&RxMessage.Data[1], RxLen);
		break;
	case 1: // FirstFrame
		RxLen = (RxMessage.Data[0] & 0xF) << 8;
		RxLen += RxMessage.Data[1];
		if(writeLen) // Need wait
		{
			sendBuf[0] = RxMessage.Data[2];
			sendBuf[1] = 0x78;
			CANAnswer(0x7F, sendBuf, 2);
			CANRxError();
			break;
		}
		CANRxCurrLen = RxLen;
		CANRxCurrWr = 0;
		CF_SN = 1;
		CANRxPacket(&RxMessage.Data[2], 6);
		if(CANRxCurrLen > 0xff)
		{
			CANRxError();
			CANSendFC(FC_FS_OVFLW, 0, 0);
		}else
			CANSendFC(FC_FS_CTS, 30, 8);
		break;
	case 3: // FlowControl
		break;
	default:
		CANRxError();
		break;
	}
}