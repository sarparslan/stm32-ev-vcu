
#include "bms_can.h"
#include "ev_config.h"
#include "stm32f4xx_hal.h"

static BmsStatus_t bmsStatus;

static uint16_t ReadU16LE(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void WriteU16LE(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

void BmsCan_Init(void)
{
  bmsStatus.batteryVoltage_dV = 0U;
  bmsStatus.batteryCurrent_dA = 0;
  bmsStatus.batteryTemp_C = 0;
  bmsStatus.soc_percent = 0U;
  bmsStatus.bmsState = BMS_STATE_OFF;

  bmsStatus.faultActive = 0U;
  bmsStatus.chargeAllowed = 0U;
  bmsStatus.dischargeAllowed = 0U;
  bmsStatus.contactorClosed = 0U;

  bmsStatus.lastUpdateTime_ms = 0U;
  bmsStatus.isAlive = 0U;
}

/*
 *
 * CAN_ID_BMS_STATUS_1:
 *   Byte 0-1: batteryVoltage_dV   (0.1 V)
 *   Byte 2-3: batteryCurrent_dA   (0.1 A, offset : 32000, signed: + discharge / - charge)
 *   Byte 4-7: reserved
 *
 * CAN_ID_BMS_STATUS_2:
 *   Byte 0:   batteryTemp_C       (offset : 40)
 *   Byte 1:   soc_percent
 *   Byte 2:   bmsState
 *   Byte 3-7: reserved
 *
 * CAN_ID_BMS_STATUS_3:
 *   Byte 0:   packed flags
 *             bit 0: fault active
 *             bit 1: charge allowed
 *             bit 2: discharge allowed
 *             bit 3: contactor closed
 *   Byte 1-7: reserved
 */
uint8_t BmsCan_HandleRx(uint32_t canId, const uint8_t *data, uint8_t len)
{
  if (data == 0 || len != 8U)
  {
    return 0U;
  }

  switch (canId)
  {
    case CAN_ID_BMS_STATUS_1:
      bmsStatus.batteryVoltage_dV = ReadU16LE(&data[0]);  // 0.1 V scale, unsigned
      bmsStatus.batteryCurrent_dA =
          (int16_t)((int32_t)ReadU16LE(&data[2]) - 32000);
      break;

    case CAN_ID_BMS_STATUS_2:
      bmsStatus.batteryTemp_C = (int8_t)((int16_t)data[0] - 40);
      bmsStatus.soc_percent = data[1];
      bmsStatus.bmsState = data[2];
      break;

    case CAN_ID_BMS_STATUS_3:
    {
      uint8_t flags = data[0];

      bmsStatus.faultActive = ((flags & (1U << 0)) != 0U) ? 1U : 0U;
      bmsStatus.chargeAllowed = ((flags & (1U << 1)) != 0U) ? 1U : 0U;
      bmsStatus.dischargeAllowed = ((flags & (1U << 2)) != 0U) ? 1U : 0U;
      bmsStatus.contactorClosed = ((flags & (1U << 3)) != 0U) ? 1U : 0U;
      break;
    }

    default:
      return 0U;
  }

  bmsStatus.lastUpdateTime_ms = HAL_GetTick();
  bmsStatus.isAlive = 1U;

  return 1U;
}

BmsStatus_t BmsCan_GetStatus(void)
{
  return bmsStatus;
}

void BmsCan_BuildCommandPayload(const BmsCommand_t *command, uint8_t *data, uint8_t *len)
{
  if (command == 0 || data == 0 || len == 0)
  {
    return;
  }

  data[0] = command->contactorRequest;
  data[1] = command->chargeRequest;
  data[2] = command->dischargeRequest;

  WriteU16LE(&data[3], command->requestedChargeCurrent_dA);

  data[5] = command->aliveCounter;
  data[6] = 0U;
  data[7] = 0U;

  *len = 8U;
}
