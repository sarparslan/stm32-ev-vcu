
#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include "ev_types.h"
#include <stdint.h>

void MotorCan_Init(void);

uint8_t MotorCan_HandleRx(uint32_t canId, const uint8_t *data, uint8_t len);
MotorStatus_t MotorCan_GetStatus(void);

void MotorCan_BuildCommandPayload(const MotorCommand_t *command, uint8_t *data, uint8_t *len);

#endif /* MOTOR_CAN_H */
