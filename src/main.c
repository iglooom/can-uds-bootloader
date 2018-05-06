/*
 * main.c
 *
 *  Created on: Feb 2, 2014
 *      Author: gl
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>
#include "can.h"
#include "main.h"

#define LED1_Pin GPIO12
#define LED1_GPIO_Port GPIOC

uint8_t tmpBuf[10];
uint8_t tmpBufLen;

extern uint16_t writeCrc;
extern volatile uint8_t bootloaderActive;
extern volatile uint8_t eraseFlag;
extern volatile uint8_t writeLen;
extern volatile uint8_t endFlag;
extern volatile uint8_t blockSequenceCounter;
extern uint8_t *CANRxCurrDataPtr;
extern uint32_t CANRxCurrFlash;

void erase_flash()
{
	uint32_t address = PROG_START_ADDR;
	flash_erase_page(address);
	for(int i=0; i<FLASH_PAGES_COUNT; i++)
	{
		address += FLASH_PAGE_SIZE;
		flash_erase_page(address);
	}
}

void flash_write(uint32_t address,uint32_t data)
{
    flash_program_half_word(address,(uint16_t)data);
    address+=2;
    data>>=16;
    flash_program_half_word(address,(uint16_t)data);
}

int main()
{
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_HSE25_72MHZ]);

    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOB);

    can_setup();

    gpio_set_mode(LED1_GPIO_Port,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,LED1_Pin);

    gpio_clear(LED1_GPIO_Port,LED1_Pin);

	tmpBufLen = 0;
	writeCrc = 0;

	uint32_t t = 6500000;
	while(--t && !bootloaderActive){
		gpio_toggle(LED1_GPIO_Port,LED1_Pin);
	}

	gpio_clear(LED1_GPIO_Port,LED1_Pin);

	if ((*(volatile uint32_t *)APPLICATION_ADDRESS & 0x2FFE0000) != 0x20000000) {
		bootloaderActive = 1;
	}

	if(bootloaderActive){
		gpio_set(LED1_GPIO_Port,LED1_Pin);
	}

	while (bootloaderActive)
	{
		if(eraseFlag)
		{
			eraseFlag = 0;
			erase_flash();
		}
		if(writeLen)
		{
			uint32_t data;
			uint8_t *startPtr;

			startPtr = CANRxCurrDataPtr;

			if(tmpBufLen)
			{
				uint8_t shift = 0;
				data = 0;
				for(int i=0;i<tmpBufLen;i++)
				{
					data += (tmpBuf[i]) << shift;
					shift+=8;
					if(shift>24)
						shift = 0;
				}
				for(int i=tmpBufLen;i<4;i++)
				{
					data += (*CANRxCurrDataPtr) << shift;
					CANRxCurrDataPtr++;
					shift+=8;
					if(shift>24)
						shift = 0;
				}
				flash_write(CANRxCurrFlash,data);
				CANRxCurrFlash +=4;
				tmpBufLen = 0;
			}

			while(writeLen - (CANRxCurrDataPtr-startPtr) >= 4)
			{
				data = (*CANRxCurrDataPtr);
				CANRxCurrDataPtr++;
				data += (*CANRxCurrDataPtr) << 8;
				CANRxCurrDataPtr++;
				data += (*CANRxCurrDataPtr) << 16;
				CANRxCurrDataPtr++;
				data += (*CANRxCurrDataPtr) << 24;
				CANRxCurrDataPtr++;
				flash_write(CANRxCurrFlash,data);
				CANRxCurrFlash += 4;
			}
			if((CANRxCurrDataPtr-startPtr) < writeLen)
			{
				tmpBufLen = writeLen - (CANRxCurrDataPtr-startPtr);
				for(int i=0;i<tmpBufLen;i++)
				{
					tmpBuf[i] = (*CANRxCurrDataPtr);
					CANRxCurrDataPtr++;
				}
			}
			writeLen = 0;
			uint8_t t = blockSequenceCounter;
			CANAnswer(0x76, &t, 1);
			blockSequenceCounter++;
		}
		if(endFlag)
		{
			uint32_t data;
			if(tmpBufLen)
			{
				uint8_t shift = 0;
				data = 0;
				for(int i=0;i<tmpBufLen;i++)
				{
					data += (tmpBuf[i]) << shift;
					shift+=8;
					if(shift>24)
						shift = 0;
				}
				for(int i=tmpBufLen;i<4;i++)
				{
					data += 0xFF << shift;
					shift+=8;
					if(shift>24)
						shift = 0;
				}
				flash_write(CANRxCurrFlash,data);
				CANRxCurrFlash +=4;
				tmpBufLen = 0;
			}

			uint8_t sendBuf[2];
			sendBuf[0] = writeCrc>>8;
			sendBuf[1] = writeCrc & 0xff;
			CANAnswer(0x77, sendBuf, 2);
			endFlag = 0;
		}
	}

	// Go to main app
	if(!bootloaderActive)
	{
		/* Set vector table base address. */
		SCB_VTOR = APPLICATION_ADDRESS & 0xFFFF;
		/* Initialise master stack pointer. */
		asm volatile("msr msp, %0"::"g"
					(*(volatile uint32_t *)APPLICATION_ADDRESS));
		/* Jump to application. */
		(*(void (**)())(APPLICATION_ADDRESS + 4))();
	}
}