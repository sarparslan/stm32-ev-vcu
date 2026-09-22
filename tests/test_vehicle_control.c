/*
 * State machine and fault-handling tests for vehicle_control.c.
 *
 * Every test starts from a healthy bench: all three nodes alive and
 * fresh, battery at 52.0 V / 25 C / 60 % SOC, motor READY, charger
 * unplugged. Each test then changes only what it is checking.
 */

#include "test_framework.h"
#include "fake_hal.h"

#include "vehicle_control.h"
#include "ev_config.h"

#define NOW_MS 10000U

typedef struct
{
  BmsStatus_t bms;
  MotorStatus_t motor;
  ChargerStatus_t charger;
} Bench_t;

static Bench_t HealthyBench(void)
{
  Bench_t b = { 0 };

  b.bms.batteryVoltage_dV = 520U;
  b.bms.batteryTemp_C = 25;
  b.bms.soc_percent = 60U;
  b.bms.bmsState = BMS_STATE_STANDBY;
  b.bms.chargeAllowed = 1U;
  b.bms.dischargeAllowed = 1U;
  b.bms.isAlive = 1U;
  b.bms.lastUpdateTime_ms = NOW_MS;

  b.motor.motorTemp_C = 30;
  b.motor.motorState = MOTOR_STATE_READY;
  b.motor.isAlive = 1U;
  b.motor.lastUpdateTime_ms = NOW_MS;

  b.charger.chargerState = CHARGER_STATE_DISCONNECTED;
  b.charger.isAlive = 1U;
  b.charger.lastUpdateTime_ms = NOW_MS;

  return b;
}

static Bench_t PluggedInBench(ChargerState_t chargerState)
{
  Bench_t b = HealthyBench();
  b.charger.plugDetected = 1U;
  b.charger.chargerState = chargerState;
  return b;
}

static void Step(const Bench_t *b)
{
  VehicleControl_UpdateFromCan(b->bms, b->motor, b->charger);
}

static void Reset(void)
{
  FakeHal_SetTick(NOW_MS);
  VehicleControl_Init();
}

static void CheckAllCommandsOff(void)
{
  BmsCommand_t bms = VehicleControl_GetBmsCommand();
  MotorCommand_t motor = VehicleControl_GetMotorCommand();
  ChargerCommand_t charger = VehicleControl_GetChargerCommand();

  CHECK_EQ(bms.contactorRequest, 0);
  CHECK_EQ(bms.chargeRequest, 0);
  CHECK_EQ(bms.dischargeRequest, 0);
  CHECK_EQ(motor.enableRequest, 0);
  CHECK_EQ(motor.torqueRequest_Nm, 0);
  CHECK_EQ(charger.chargeEnable, 0);
}

static void CheckFault(Bench_t b, uint32_t expectedFlag)
{
  Reset();
  Step(&b);
  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_FAULT);
  CHECK((VehicleControl_GetFaultFlags() & expectedFlag) != 0U);
  CheckAllCommandsOff();
}

static void CheckNoFault(Bench_t b)
{
  Reset();
  Step(&b);
  CHECK(VehicleControl_GetState() != VEHICLE_STATE_FAULT);
  CHECK_EQ(VehicleControl_GetFaultFlags(), FAULT_NONE);
}

/* ------------------------------------------------------- Drive path */

static void test_starts_idle_with_no_faults(void)
{
  Reset();
  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_IDLE);
  CHECK_EQ(VehicleControl_GetFaultFlags(), FAULT_NONE);
  CheckAllCommandsOff();
}

static void test_motor_ready_goes_ready_and_enables_motor(void)
{
  Reset();
  Bench_t b = HealthyBench();
  Step(&b);

  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_READY);
  CHECK_EQ(VehicleControl_GetBmsCommand().contactorRequest, 1);
  CHECK_EQ(VehicleControl_GetBmsCommand().dischargeRequest, 1);
  CHECK_EQ(VehicleControl_GetBmsCommand().chargeRequest, 0);
  CHECK_EQ(VehicleControl_GetMotorCommand().enableRequest, 1);
  CHECK_EQ(VehicleControl_GetMotorCommand().direction, MOTOR_DIR_FORWARD);
  CHECK_EQ(VehicleControl_GetMotorCommand().torqueRequest_Nm, 50);
  CHECK_EQ(VehicleControl_GetChargerCommand().chargeEnable, 0);
}

static void test_motor_running_goes_drive(void)
{
  Reset();
  Bench_t b = HealthyBench();
  b.motor.motorState = MOTOR_STATE_RUNNING;
  Step(&b);

  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_DRIVE);
  CHECK_EQ(VehicleControl_GetMotorCommand().enableRequest, 1);
}

static void test_discharge_not_allowed_stays_idle_with_motor_off(void)
{
  Reset();
  Bench_t b = HealthyBench();
  b.bms.dischargeAllowed = 0U;
  Step(&b);

  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_IDLE);
  CHECK_EQ(VehicleControl_GetMotorCommand().enableRequest, 0);
  CHECK_EQ(VehicleControl_GetMotorCommand().torqueRequest_Nm, 0);
}

static void test_drive_drops_back_to_ready_when_motor_stops(void)
{
  Reset();
  Bench_t b = HealthyBench();
  b.motor.motorState = MOTOR_STATE_RUNNING;
  Step(&b);
  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_DRIVE);

  b.motor.motorState = MOTOR_STATE_READY;
  Step(&b);
  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_READY);
}

/* ---------------------------------------------------- Charging path */

static void test_plug_with_charger_ready_enables_charging(void)
{
  Reset();
  Bench_t b = PluggedInBench(CHARGER_STATE_READY);
  Step(&b);

  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_READY);
  CHECK_EQ(VehicleControl_GetBmsCommand().contactorRequest, 1);
  CHECK_EQ(VehicleControl_GetBmsCommand().chargeRequest, 1);
  CHECK_EQ(VehicleControl_GetBmsCommand().dischargeRequest, 0);
  CHECK_EQ(VehicleControl_GetChargerCommand().chargeEnable, 1);
  CHECK_EQ(VehicleControl_GetChargerCommand().targetVoltage_dV, DEFAULT_CHARGE_TARGET_VOLTAGE_dV);
  CHECK_EQ(VehicleControl_GetChargerCommand().targetCurrent_dA, DEFAULT_CHARGE_TARGET_CURRENT_dA);
}

static void test_charger_charging_goes_charging(void)
{
  Reset();
  Bench_t b = PluggedInBench(CHARGER_STATE_CHARGING);
  Step(&b);

  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_CHARGING);
  CHECK_EQ(VehicleControl_GetChargerCommand().chargeEnable, 1);
}

static void test_plug_in_always_disables_motor(void)
{
  Reset();
  Bench_t b = PluggedInBench(CHARGER_STATE_CHARGING);
  b.motor.motorState = MOTOR_STATE_RUNNING;
  Step(&b);

  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_CHARGING);
  CHECK_EQ(VehicleControl_GetMotorCommand().enableRequest, 0);
  CHECK_EQ(VehicleControl_GetMotorCommand().torqueRequest_Nm, 0);
}

static void test_full_battery_does_not_charge(void)
{
  Reset();
  Bench_t b = PluggedInBench(CHARGER_STATE_READY);
  b.bms.soc_percent = BATTERY_SOC_FULL_PERCENT;
  Step(&b);

  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_READY);
  CHECK_EQ(VehicleControl_GetChargerCommand().chargeEnable, 0);
  CHECK_EQ(VehicleControl_GetChargerCommand().targetCurrent_dA, 0);
}

static void test_charge_not_allowed_by_bms_does_not_charge(void)
{
  Reset();
  Bench_t b = PluggedInBench(CHARGER_STATE_READY);
  b.bms.chargeAllowed = 0U;
  Step(&b);

  CHECK_EQ(VehicleControl_GetChargerCommand().chargeEnable, 0);
}

/* ---------------------------------------------------------- Faults */

static void test_node_never_heard_from_is_comm_timeout(void)
{
  Bench_t b = HealthyBench();
  b.motor.isAlive = 0U;
  CheckFault(b, FAULT_COMM_TIMEOUT);
}

static void test_comm_timeout_boundary(void)
{
  Bench_t b = HealthyBench();

  b.charger.lastUpdateTime_ms = NOW_MS - CAN_NODE_TIMEOUT_MS;
  CheckNoFault(b);

  b.charger.lastUpdateTime_ms = NOW_MS - CAN_NODE_TIMEOUT_MS - 1U;
  CheckFault(b, FAULT_COMM_TIMEOUT);
}

static void test_comm_timeout_survives_tick_wraparound(void)
{
  /* HAL_GetTick() wraps after ~49.7 days; unsigned subtraction must
     still see a frame received 512 ms ago as fresh. */
  Bench_t b = HealthyBench();
  b.bms.lastUpdateTime_ms = 0xFFFFFF00U;
  b.motor.lastUpdateTime_ms = 0xFFFFFF00U;
  b.charger.lastUpdateTime_ms = 0xFFFFFF00U;

  FakeHal_SetTick(0x00000100U);
  VehicleControl_Init();
  Step(&b);

  CHECK(VehicleControl_GetState() != VEHICLE_STATE_FAULT);
}

static void test_battery_over_temperature_boundary(void)
{
  Bench_t b = HealthyBench();

  b.bms.batteryTemp_C = BATTERY_TEMP_FAULT_THRESHOLD_C - 1;
  CheckNoFault(b);

  b.bms.batteryTemp_C = BATTERY_TEMP_FAULT_THRESHOLD_C;
  CheckFault(b, FAULT_OVER_TEMPERATURE);
}

static void test_motor_over_temperature_boundary(void)
{
  Bench_t b = HealthyBench();

  b.motor.motorTemp_C = MOTOR_TEMP_FAULT_THRESHOLD_C - 1;
  CheckNoFault(b);

  b.motor.motorTemp_C = MOTOR_TEMP_FAULT_THRESHOLD_C;
  CheckFault(b, FAULT_OVER_TEMPERATURE);
}

static void test_battery_voltage_window(void)
{
  Bench_t b = HealthyBench();

  b.bms.batteryVoltage_dV = BATTERY_UNDER_VOLTAGE_THRESHOLD_dV;
  CheckFault(b, FAULT_UNDER_VOLTAGE);

  b.bms.batteryVoltage_dV = BATTERY_UNDER_VOLTAGE_THRESHOLD_dV + 1U;
  CheckNoFault(b);

  b.bms.batteryVoltage_dV = BATTERY_OVER_VOLTAGE_THRESHOLD_dV - 1U;
  CheckNoFault(b);

  b.bms.batteryVoltage_dV = BATTERY_OVER_VOLTAGE_THRESHOLD_dV;
  CheckFault(b, FAULT_OVER_VOLTAGE);
}

static void test_node_reported_faults(void)
{
  Bench_t b;

  b = HealthyBench();
  b.bms.faultActive = 1U;
  CheckFault(b, FAULT_BMS_FAULT);

  b = HealthyBench();
  b.bms.bmsState = BMS_STATE_FAULT;
  CheckFault(b, FAULT_BMS_FAULT);

  b = HealthyBench();
  b.motor.motorState = MOTOR_STATE_FAULT;
  CheckFault(b, FAULT_MOTOR_FAULT);

  b = HealthyBench();
  b.charger.faultActive = 1U;
  CheckFault(b, FAULT_CHARGER_FAULT);
}

static void test_fault_is_latched_after_cause_clears(void)
{
  Reset();
  Bench_t b = HealthyBench();
  b.bms.batteryTemp_C = BATTERY_TEMP_FAULT_THRESHOLD_C + 5;
  Step(&b);
  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_FAULT);

  /* Battery cools down and everything looks healthy again. */
  b = HealthyBench();
  b.motor.motorState = MOTOR_STATE_RUNNING;
  for (int i = 0; i < 100; i++)
  {
    Step(&b);
  }

  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_FAULT);
  CheckAllCommandsOff();
}

static void test_only_init_clears_a_latched_fault(void)
{
  Reset();
  Bench_t b = HealthyBench();
  b.motor.isAlive = 0U;
  Step(&b);
  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_FAULT);

  VehicleControl_Init();
  CHECK_EQ(VehicleControl_GetState(), VEHICLE_STATE_IDLE);
  CHECK_EQ(VehicleControl_GetFaultFlags(), FAULT_NONE);
}

void run_vehicle_control_tests(void)
{
  RUN_TEST(test_starts_idle_with_no_faults);
  RUN_TEST(test_motor_ready_goes_ready_and_enables_motor);
  RUN_TEST(test_motor_running_goes_drive);
  RUN_TEST(test_discharge_not_allowed_stays_idle_with_motor_off);
  RUN_TEST(test_drive_drops_back_to_ready_when_motor_stops);

  RUN_TEST(test_plug_with_charger_ready_enables_charging);
  RUN_TEST(test_charger_charging_goes_charging);
  RUN_TEST(test_plug_in_always_disables_motor);
  RUN_TEST(test_full_battery_does_not_charge);
  RUN_TEST(test_charge_not_allowed_by_bms_does_not_charge);

  RUN_TEST(test_node_never_heard_from_is_comm_timeout);
  RUN_TEST(test_comm_timeout_boundary);
  RUN_TEST(test_comm_timeout_survives_tick_wraparound);
  RUN_TEST(test_battery_over_temperature_boundary);
  RUN_TEST(test_motor_over_temperature_boundary);
  RUN_TEST(test_battery_voltage_window);
  RUN_TEST(test_node_reported_faults);
  RUN_TEST(test_fault_is_latched_after_cause_clears);
  RUN_TEST(test_only_init_clears_a_latched_fault);
}
