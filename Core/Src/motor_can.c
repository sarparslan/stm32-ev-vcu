
#include "motor_can.h"
#include "ev_config.h"
#include "stm32f4xx_hal.h"

static MotorStatus_t motorStatus;

static uint16_t ReadU16LE(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void WriteI16LE(uint8_t *data, int16_t value)
{
  data[0] = (uint8_t)((uint16_t)value & 0xFFU);
  data[1] = (uint8_t)(((uint16_t)value >> 8) & 0xFFU);
}

void MotorCan_Init(void)
{
  motorStatus.motorRpm = 0;
  motorStatus.motorCurrent_dA = 0;
  motorStatus.motorTemp_C = 0;
  motorStatus.motorState = MOTOR_STATE_OFF;
  motorStatus.motorDirection = MOTOR_DIR_NEUTRAL;

  motorStatus.faultActive = 0U;

  motorStatus.aliveCounter = 0U;
  motorStatus.lastUpdateTime_ms = 0U;
  motorStatus.isAlive = 0U;
}

/*
 *
 * CAN_ID_MOTOR_STATUS_1:
 *   Byte 0-1: motorRpm           (offset : 32000)
 *   Byte 2-3: motorCurrent_dA     (0.1 A, offset : 32000)
 *   Byte 4-7: reserved
 *
 * CAN_ID_MOTOR_STATUS_2:
 *   Byte 0:   motorTemp_C         (offset : 40)
 *   Byte 1:   motorState
 *   Byte 2:   motorDirection      (0 neutral, 1 forward, 2 reverse)
 *   Byte 3-7: reserved
 *
 * CAN_ID_MOTOR_STATUS_3:
 *   Byte 0:   packed flags
 *             bit 0: fault active
 *   Byte 1:   aliveCounter
 *   Byte 2-7: reserved
 *
 */
uint8_t MotorCan_HandleRx(uint32_t canId, const uint8_t *data, uint8_t len)
{
  if (data == 0 || len != 8U)
  {
    return 0U;
  }

  switch (canId)
  {
    case CAN_ID_MOTOR_STATUS_1:
      motorStatus.motorRpm =
          (int16_t)((int32_t)ReadU16LE(&data[0]) - 32000);
      motorStatus.motorCurrent_dA =
          (int16_t)((int32_t)ReadU16LE(&data[2]) - 32000);
      break;

    case CAN_ID_MOTOR_STATUS_2:
      motorStatus.motorTemp_C = (int8_t)((int16_t)data[0] - 40);
      motorStatus.motorState = data[1];
      motorStatus.motorDirection = data[2];
      break;

    case CAN_ID_MOTOR_STATUS_3:
    {
      uint8_t flags = data[0];

      motorStatus.faultActive = ((flags & (1U << 0)) != 0U) ? 1U : 0U;
      motorStatus.aliveCounter = data[1];
      break;
    }

    default:
      return 0U;
  }

  motorStatus.lastUpdateTime_ms = HAL_GetTick();
  motorStatus.isAlive = 1U;

  return 1U;
}

MotorStatus_t MotorCan_GetStatus(void)
{
  return motorStatus;
}

void MotorCan_BuildCommandPayload(const MotorCommand_t *command, uint8_t *data, uint8_t *len)
{
  if (command == 0 || data == 0 || len == 0)
  {
    return;
  }

  data[0] = command->enableRequest;
  data[1] = command->direction;

  WriteI16LE(&data[2], command->torqueRequest_Nm);

  data[4] = command->regenRequest_percent;
  data[5] = command->aliveCounter;
  data[6] = 0U;
  data[7] = 0U;

  *len = 8U;
}
