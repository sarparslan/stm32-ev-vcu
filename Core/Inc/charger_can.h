
#ifndef CHARGER_CAN_H
#define CHARGER_CAN_H

#include "ev_types.h"
#include <stdint.h>

void ChargerCan_Init(void);

uint8_t ChargerCan_HandleRx(uint32_t canId, const uint8_t *data, uint8_t len);
ChargerStatus_t ChargerCan_GetStatus(void);

void ChargerCan_BuildCommandPayload(const ChargerCommand_t *command, uint8_t *data, uint8_t *len);

#endif /* CHARGER_CAN_H */

