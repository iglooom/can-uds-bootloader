/*
 * can.h
 *
 *  Created on: 2 Jan 2017
 *      Author: gl
 */

#ifndef CAN_H_
#define CAN_H_

#include <stdint.h>

#define CAN CAN1

typedef struct {
    uint32_t CanPort;
    uint16_t Id;
    uint8_t DLC;
    uint8_t Data[8];
    uint8_t Delay;
} canMsg;

#define CANID_RX 0x707
#define CANID_TX 0x70F

#define FC_FS_CTS 0 // ContinueToSend
#define FC_FS_WT 1 // Wait
#define FC_FS_OVFLW 2 // Overflow

void CANAnswer(uint8_t serviceId, uint8_t *dataBuf, uint8_t len);
void CAN_Bus_Init(void);
void CAN_Driver_Wake(void);
void CAN_Driver_Sleep(void);
void CANRxError(void);

void usb_lp_can_rx0_isr(void);
void can_setup(void);

#endif /* CAN_H_ */