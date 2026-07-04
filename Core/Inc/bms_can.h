
#ifndef BMS_CAN_H
#define BMS_CAN_H

#include "ev_types.h"
#include <stdint.h>

void BmsCan_Init(void);

uint8_t BmsCan_HandleRx(uint32_t canId, const uint8_t *data, uint8_t len);
BmsStatus_t BmsCan_GetStatus(void);

void BmsCan_BuildCommandPayload(const BmsCommand_t *command, uint8_t *data, uint8_t *len);

#endif /* BMS_CAN_H */
