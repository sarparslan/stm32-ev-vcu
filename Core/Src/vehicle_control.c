
#include "vehicle_control.h"
#include "ev_config.h"
#include "stm32f4xx_hal.h"

static VehicleState_t currentState;
static uint32_t faultFlags;

static BmsCommand_t bmsCommand;
static MotorCommand_t motorCommand;
static ChargerCommand_t chargerCommand;

static void ClearCommands(void)
{
  bmsCommand.contactorRequest = 0U;
  bmsCommand.chargeRequest = 0U;
  bmsCommand.dischargeRequest = 0U;
  bmsCommand.requestedChargeCurrent_dA = 0U;

  motorCommand.enableRequest = 0U;
  motorCommand.direction = 0U;
  motorCommand.torqueRequest_Nm = 0;
  motorCommand.regenRequest_percent = 0U;

  chargerCommand.chargeEnable = 0U;
  chargerCommand.targetVoltage_dV = 0U;
  chargerCommand.targetCurrent_dA = 0U;
}

static void SetFault(uint32_t fault)
{
  faultFlags |= fault;
  currentState = VEHICLE_STATE_FAULT;

  /*
   * Safe state:
   * - stop charging
   * - disable motor
   * - remove BMS requests
   */
  ClearCommands();
}

static uint8_t IsNodeTimedOut(uint8_t isAlive, uint32_t lastUpdateTime_ms)
{
  uint32_t now = HAL_GetTick();

  if (isAlive == 0U)
  {
    return 1U;
  }

  if ((now - lastUpdateTime_ms) > CAN_NODE_TIMEOUT_MS)
  {
    return 1U;
  }

  return 0U;
}

static void UpdateFaultDetection(
    BmsStatus_t bmsStatus,
    MotorStatus_t motorStatus,
    ChargerStatus_t chargerStatus
)
{
  if (IsNodeTimedOut(bmsStatus.isAlive, bmsStatus.lastUpdateTime_ms) != 0U)
  {
    SetFault(FAULT_COMM_TIMEOUT);
    return;
  }

  if (IsNodeTimedOut(motorStatus.isAlive, motorStatus.lastUpdateTime_ms) != 0U)
  {
    SetFault(FAULT_COMM_TIMEOUT);
    return;
  }

  if (IsNodeTimedOut(chargerStatus.isAlive, chargerStatus.lastUpdateTime_ms) != 0U)
  {
    SetFault(FAULT_COMM_TIMEOUT);
    return;
  }

  if (bmsStatus.batteryTemp_C >= BATTERY_TEMP_FAULT_THRESHOLD_C)
  {
    SetFault(FAULT_OVER_TEMPERATURE);
    return;
  }

  if (motorStatus.motorTemp_C >= MOTOR_TEMP_FAULT_THRESHOLD_C)
  {
    SetFault(FAULT_OVER_TEMPERATURE);
    return;
  }

  if (bmsStatus.batteryVoltage_dV <= BATTERY_UNDER_VOLTAGE_THRESHOLD_dV)
  {
    SetFault(FAULT_UNDER_VOLTAGE);
    return;
  }

  if (bmsStatus.batteryVoltage_dV >= BATTERY_OVER_VOLTAGE_THRESHOLD_dV)
  {
    SetFault(FAULT_OVER_VOLTAGE);
    return;
  }

  if (bmsStatus.faultActive != 0U || bmsStatus.bmsState == BMS_STATE_FAULT)
  {
    SetFault(FAULT_BMS_FAULT);
    return;
  }

  if (motorStatus.faultActive != 0U || motorStatus.motorState == MOTOR_STATE_FAULT)
  {
    SetFault(FAULT_MOTOR_FAULT);
    return;
  }

  if (chargerStatus.faultActive != 0U || chargerStatus.chargerState == CHARGER_STATE_FAULT)
  {
    SetFault(FAULT_CHARGER_FAULT);
    return;
  }
}

static void UpdateChargingControl(
    BmsStatus_t bmsStatus,
    ChargerStatus_t chargerStatus
)
{
  /*
   * Charging scenario:
   *
   * Charger says:
   * - plug detected
   * - charger ready
   *
   * BMS says:
   * - charge allowed
   *
   * ECU then:
   * - asks BMS for contactor + charge
   * - enables charger
   */
  if (chargerStatus.plugDetected != 0U)
  {
    bmsCommand.contactorRequest = 1U;
    bmsCommand.chargeRequest = 1U;
    bmsCommand.dischargeRequest = 0U;
    bmsCommand.requestedChargeCurrent_dA = DEFAULT_CHARGE_TARGET_CURRENT_dA;

    motorCommand.enableRequest = 0U;
    motorCommand.direction = 0U;
    motorCommand.torqueRequest_Nm = 0;
    motorCommand.regenRequest_percent = 0U;

    if (bmsStatus.chargeAllowed != 0U &&
        (chargerStatus.chargerState == CHARGER_STATE_READY ||
         chargerStatus.chargerState == CHARGER_STATE_CHARGING) &&
        bmsStatus.soc_percent < BATTERY_SOC_FULL_PERCENT)
    {
      chargerCommand.chargeEnable = 1U;
      chargerCommand.targetVoltage_dV = DEFAULT_CHARGE_TARGET_VOLTAGE_dV;
      chargerCommand.targetCurrent_dA = DEFAULT_CHARGE_TARGET_CURRENT_dA;

      if (chargerStatus.chargerState == CHARGER_STATE_CHARGING)
      {
        currentState = VEHICLE_STATE_CHARGING;
      }
      else
      {
        currentState = VEHICLE_STATE_READY;
      }
    }
    else
    {
      chargerCommand.chargeEnable = 0U;
      chargerCommand.targetVoltage_dV = 0U;
      chargerCommand.targetCurrent_dA = 0U;

      currentState = VEHICLE_STATE_READY;
    }

    return;
  }
}

static void UpdateDriveControl(
    BmsStatus_t bmsStatus,
    MotorStatus_t motorStatus,
    ChargerStatus_t chargerStatus
)
{
  /*
   * Drive scenario:
   *
   * No charger plug.
   * BMS says discharge allowed.
   * Motor says ready.
   *
   * ECU enables motor.
   */
  if (chargerStatus.plugDetected == 0U)
  {
    bmsCommand.contactorRequest = 1U;
    bmsCommand.chargeRequest = 0U;
    bmsCommand.dischargeRequest = 1U;
    bmsCommand.requestedChargeCurrent_dA = 0U;

    chargerCommand.chargeEnable = 0U;
    chargerCommand.targetVoltage_dV = 0U;
    chargerCommand.targetCurrent_dA = 0U;

    if (bmsStatus.dischargeAllowed != 0U &&
        (motorStatus.motorState == MOTOR_STATE_READY ||
         motorStatus.motorState == MOTOR_STATE_RUNNING))
    {
      motorCommand.enableRequest = 1U;
      motorCommand.direction = 1U;
      motorCommand.torqueRequest_Nm = 50;
      motorCommand.regenRequest_percent = 0U;

      if (motorStatus.motorState == MOTOR_STATE_RUNNING)
      {
        currentState = VEHICLE_STATE_DRIVE;
      }
      else
      {
        currentState = VEHICLE_STATE_READY;
      }
    }
    else
    {
      motorCommand.enableRequest = 0U;
      motorCommand.direction = 0U;
      motorCommand.torqueRequest_Nm = 0;
      motorCommand.regenRequest_percent = 0U;

      currentState = VEHICLE_STATE_IDLE;
    }

    return;
  }
}

void VehicleControl_Init(void)
{
  currentState = VEHICLE_STATE_IDLE;
  faultFlags = FAULT_NONE;

  ClearCommands();
}

void VehicleControl_UpdateFromCan(
    BmsStatus_t bmsStatus,
    MotorStatus_t motorStatus,
    ChargerStatus_t chargerStatus
)
{
  /* Zero all commands for this cycle. The alive counter is owned by
     the TX path (app.c), so it only advances when frames are sent. */
  ClearCommands();

  /*
   * Latched fault: once faulted we never auto-recover. Commands were
   * already zeroed above, so just stay in the safe state and skip
   * all control logic.
   */
  if (currentState == VEHICLE_STATE_FAULT)
  {
    return;
  }

  UpdateFaultDetection(bmsStatus, motorStatus, chargerStatus);

  /* If a fault was just detected, SetFault() already cleared commands. */
  if (currentState == VEHICLE_STATE_FAULT)
  {
    return;
  }

  UpdateChargingControl(bmsStatus, chargerStatus);

  if (chargerStatus.plugDetected != 0U)
  {
    return;
  }

  UpdateDriveControl(bmsStatus, motorStatus, chargerStatus);
}

VehicleState_t VehicleControl_GetState(void)
{
  return currentState;
}

uint32_t VehicleControl_GetFaultFlags(void)
{
  return faultFlags;
}

BmsCommand_t VehicleControl_GetBmsCommand(void)
{
  return bmsCommand;
}

MotorCommand_t VehicleControl_GetMotorCommand(void)
{
  return motorCommand;
}

ChargerCommand_t VehicleControl_GetChargerCommand(void)
{
  return chargerCommand;
}
