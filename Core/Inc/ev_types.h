
#ifndef EV_TYPES_H
#define EV_TYPES_H

#include <stdint.h>

/*
 * Main ECU state.
 */
typedef enum
{
  VEHICLE_STATE_IDLE = 0,
  VEHICLE_STATE_READY,
  VEHICLE_STATE_DRIVE,
  VEHICLE_STATE_CHARGING,
  VEHICLE_STATE_FAULT
} VehicleState_t;

/*
 * ECU-level fault flags.
 */
typedef enum
{
  FAULT_NONE              = 0x00000000U,
  FAULT_OVER_TEMPERATURE  = 0x00000001U,
  FAULT_UNDER_VOLTAGE     = 0x00000002U,
  FAULT_OVER_VOLTAGE      = 0x00000004U,
  FAULT_COMM_TIMEOUT      = 0x00000008U,
  FAULT_BMS_FAULT         = 0x00000010U,
  FAULT_MOTOR_FAULT       = 0x00000020U,
  FAULT_CHARGER_FAULT     = 0x00000040U,
  FAULT_CHARGE_NOT_ALLOWED = 0x00000080U,
  FAULT_DISCHARGE_NOT_ALLOWED = 0x00000100U
} FaultFlags_t;

/*
 * BMS state values received from CAN.
 */
typedef enum
{
  BMS_STATE_OFF = 0,
  BMS_STATE_STANDBY,
  BMS_STATE_DISCHARGE,
  BMS_STATE_CHARGE,
  BMS_STATE_FAULT
} BmsState_t;

/*
 * Motor controller state values received from CAN.
 */
typedef enum
{
  MOTOR_STATE_OFF = 0,
  MOTOR_STATE_READY,
  MOTOR_STATE_RUNNING,
  MOTOR_STATE_FAULT
} MotorState_t;

/*
 * Motor direction (DNR), shared by motor command and status.
 */
typedef enum
{
  MOTOR_DIR_NEUTRAL = 0,
  MOTOR_DIR_FORWARD = 1,   // Drive
  MOTOR_DIR_REVERSE = 2
} MotorDirection_t;

/*
 * Charger state values received from CAN.
 */
typedef enum
{
  CHARGER_STATE_DISCONNECTED = 0,
  CHARGER_STATE_CONNECTED,
  CHARGER_STATE_READY,
  CHARGER_STATE_CHARGING,
  CHARGER_STATE_FAULT
} ChargerState_t;

typedef struct
{
  uint16_t batteryVoltage_dV;
  int16_t  batteryCurrent_dA;     // signed: + discharge, - charge
  int16_t  batteryTemp_C;         // wire range -40..215 C: does not fit in int8_t
  uint8_t  soc_percent;
  uint8_t  bmsState;

  uint8_t  faultActive;
  uint8_t  chargeAllowed;
  uint8_t  dischargeAllowed;
  uint8_t  contactorClosed;

  uint32_t lastUpdateTime_ms;
  uint8_t  isAlive;
} BmsStatus_t;

typedef struct
{
  int16_t  motorRpm;
  int16_t  motorCurrent_dA;       // signed: + drive, - regen
  int16_t  motorTemp_C;           // wire range -40..215 C: does not fit in int8_t
  uint8_t  motorState;
  uint8_t  motorDirection;       // MotorDirection_t: 0 neutral, 1 forward, 2 reverse

  uint8_t  faultActive;

  uint8_t  aliveCounter;
  uint32_t lastUpdateTime_ms;
  uint8_t  isAlive;
} MotorStatus_t;

typedef struct
{
  uint16_t chargerVoltage_dV;
  uint16_t chargerCurrent_dA;
  uint8_t  chargerState;

  uint8_t  faultActive;

  uint8_t  plugDetected;
  uint8_t  aliveCounter;
  uint32_t lastUpdateTime_ms;
  uint8_t  isAlive;
} ChargerStatus_t;

typedef struct
{
  uint8_t  contactorRequest;
  uint8_t  chargeRequest;
  uint8_t  dischargeRequest;
  uint16_t requestedChargeCurrent_dA;
  uint8_t  aliveCounter;
} BmsCommand_t;

typedef struct
{
  uint8_t enableRequest;
  uint8_t direction;             // 0 neutral, 1 forward, 2 reverse
  int16_t torqueRequest_Nm;
  uint8_t regenRequest_percent;
  uint8_t aliveCounter;
} MotorCommand_t;

typedef struct
{
  uint8_t  chargeEnable;
  uint16_t targetVoltage_dV;
  uint16_t targetCurrent_dA;
  uint8_t  aliveCounter;
} ChargerCommand_t;

#endif
