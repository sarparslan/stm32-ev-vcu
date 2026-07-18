
#include "charger_can.h"
#include "ev_config.h"
#include "stm32f4xx_hal.h"

static ChargerStatus_t chargerStatus;

static uint16_t ReadU16LE(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void WriteU16LE(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

void ChargerCan_Init(void)
{
  chargerStatus.chargerVoltage_dV = 0U;
  chargerStatus.chargerCurrent_dA = 0U;
  chargerStatus.chargerState = CHARGER_STATE_DISCONNECTED;

  chargerStatus.faultActive = 0U;

  chargerStatus.plugDetected = 0U;
  chargerStatus.aliveCounter = 0U;
  chargerStatus.lastUpdateTime_ms = 0U;
  chargerStatus.isAlive = 0U;
}

/*
 *
 * CAN_ID_CHARGER_STATUS_1:
 *   Byte 0-1: chargerVoltage_dV   (0.1 V)
 *   Byte 2-3: chargerCurrent_dA   (0.1 A)
 *   Byte 4-7: reserved
 *
 * CAN_ID_CHARGER_STATUS_2:
 *   Byte 0:   chargerState
 *   Byte 1:   plugDetected
 *   Byte 2-7: reserved
 *
 * CAN_ID_CHARGER_STATUS_3:
 *   Byte 0:   packed flags
 *             bit 0: fault active
 *   Byte 1:   aliveCounter
 *   Byte 2-7: reserved
 *
 */
uint8_t ChargerCan_HandleRx(uint32_t canId, const uint8_t *data, uint8_t len)
{
  if (data == 0 || len != 8U)
  {
    return 0U;
  }

  switch (canId)
  {
    case CAN_ID_CHARGER_STATUS_1:
      chargerStatus.chargerVoltage_dV = ReadU16LE(&data[0]); // 0.1 V scale
      chargerStatus.chargerCurrent_dA = ReadU16LE(&data[2]); // 0.1 A scale
      break;

    case CAN_ID_CHARGER_STATUS_2:
      chargerStatus.chargerState = data[0];
      chargerStatus.plugDetected = data[1];
      break;

    case CAN_ID_CHARGER_STATUS_3:
    {
      uint8_t flags = data[0];

      chargerStatus.faultActive = ((flags & (1U << 0)) != 0U) ? 1U : 0U;
      chargerStatus.aliveCounter = data[1];
      break;
    }

    default:
      return 0U;
  }

  chargerStatus.lastUpdateTime_ms = HAL_GetTick();
  chargerStatus.isAlive = 1U;

  return 1U;
}

ChargerStatus_t ChargerCan_GetStatus(void)
{
  return chargerStatus;
}

void ChargerCan_BuildCommandPayload(const ChargerCommand_t *command, uint8_t *data, uint8_t *len)
{
  if (command == 0 || data == 0 || len == 0)
  {
    return;
  }

  data[0] = command->chargeEnable;

  WriteU16LE(&data[1], command->targetVoltage_dV);
  WriteU16LE(&data[3], command->targetCurrent_dA);

  data[5] = command->aliveCounter;
  data[6] = 0U;
  data[7] = 0U;

  *len = 8U;
}
