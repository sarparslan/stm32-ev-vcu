/*
 * can_bus.h
 *
 *  CAN transport layer (RTOS):
 *   - RX frames are posted from the ISR into a queue, then decoded
 *     by CanRxTask (CanBus_ProcessRx).
 *   - The decoded status structs are protected by a mutex
 *     (CanBus_LockStatus / CanBus_UnlockStatus).
 *   - TX command frames are sent with CanBus_SendFrame.
 */

#ifndef INC_CAN_BUS_H_
#define INC_CAN_BUS_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Create the RX queue + status mutex and save the handle.
   Call after osKernelInitialize() (creates RTOS objects). */
void CanBus_Init(CAN_HandleTypeDef *hcan);

/* Configure the RX filter, start CAN1, enable the RX FIFO0 interrupt.
   Call from CanRxTask (after the queue exists). */
void CanBus_Start(void);

/* CanRxTask body helper: block up to 'timeout' on the RX queue, then
   decode the frame into the status structs (under the mutex). */
void CanBus_ProcessRx(uint32_t timeout);

/* Lock/unlock the shared status structs (for readers like ControlTask). */
void CanBus_LockStatus(void);
void CanBus_UnlockStatus(void);

/* Transmit one standard data frame. 1 = queued, 0 = no free mailbox. */
uint8_t CanBus_SendFrame(uint32_t canId, const uint8_t *data, uint8_t len);

#endif /* INC_CAN_BUS_H_ */
