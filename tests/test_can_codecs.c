/*
 * Wire-format tests for the BMS, motor and charger CAN codecs.
 *
 * Frames are built byte by byte exactly as the node simulator
 * (Arduino/can_node_sim) puts them on the bus: little-endian
 * multi-byte values, temp +40, current/rpm +32000.
 */

#include "test_framework.h"
#include "fake_hal.h"

#include "bms_can.h"
#include "motor_can.h"
#include "charger_can.h"
#include "ev_config.h"

#include <string.h>

/* ---------------------------------------------------------------- BMS */

static void test_bms_status1_decodes_voltage_and_discharge_current(void)
{
  BmsCan_Init();
  /* 52.0 V, +12.5 A (32125 = 0x7D7D) */
  const uint8_t frame[8] = { 0x08, 0x02, 0x7D, 0x7D, 0, 0, 0, 0 };

  CHECK_EQ(BmsCan_HandleRx(CAN_ID_BMS_STATUS_1, frame, 8), 1);
  CHECK_EQ(BmsCan_GetStatus().batteryVoltage_dV, 520);
  CHECK_EQ(BmsCan_GetStatus().batteryCurrent_dA, 125);
}

static void test_bms_status1_decodes_negative_charge_current(void)
{
  BmsCan_Init();
  /* -10.0 A while charging (31900 = 0x7C9C) */
  const uint8_t frame[8] = { 0x08, 0x02, 0x9C, 0x7C, 0, 0, 0, 0 };

  BmsCan_HandleRx(CAN_ID_BMS_STATUS_1, frame, 8);
  CHECK_EQ(BmsCan_GetStatus().batteryCurrent_dA, -100);
}

static void test_bms_status2_decodes_temp_soc_and_state(void)
{
  BmsCan_Init();
  const uint8_t warm[8] = { 25 + 40, 60, BMS_STATE_DISCHARGE, 0, 0, 0, 0, 0 };

  BmsCan_HandleRx(CAN_ID_BMS_STATUS_2, warm, 8);
  CHECK_EQ(BmsCan_GetStatus().batteryTemp_C, 25);
  CHECK_EQ(BmsCan_GetStatus().soc_percent, 60);
  CHECK_EQ(BmsCan_GetStatus().bmsState, BMS_STATE_DISCHARGE);

  const uint8_t cold[8] = { 30, 60, BMS_STATE_STANDBY, 0, 0, 0, 0, 0 };
  BmsCan_HandleRx(CAN_ID_BMS_STATUS_2, cold, 8);
  CHECK_EQ(BmsCan_GetStatus().batteryTemp_C, -10);
}

static void test_bms_status3_unpacks_each_flag_bit(void)
{
  BmsCan_Init();
  uint8_t frame[8] = { 0 };

  frame[0] = 0x01;
  BmsCan_HandleRx(CAN_ID_BMS_STATUS_3, frame, 8);
  CHECK_EQ(BmsCan_GetStatus().faultActive, 1);
  CHECK_EQ(BmsCan_GetStatus().chargeAllowed, 0);
  CHECK_EQ(BmsCan_GetStatus().dischargeAllowed, 0);
  CHECK_EQ(BmsCan_GetStatus().contactorClosed, 0);

  frame[0] = 0x0E;
  BmsCan_HandleRx(CAN_ID_BMS_STATUS_3, frame, 8);
  CHECK_EQ(BmsCan_GetStatus().faultActive, 0);
  CHECK_EQ(BmsCan_GetStatus().chargeAllowed, 1);
  CHECK_EQ(BmsCan_GetStatus().dischargeAllowed, 1);
  CHECK_EQ(BmsCan_GetStatus().contactorClosed, 1);
}

static void test_bms_rx_marks_node_alive_with_timestamp(void)
{
  BmsCan_Init();
  CHECK_EQ(BmsCan_GetStatus().isAlive, 0);

  FakeHal_SetTick(4242U);
  const uint8_t frame[8] = { 0x08, 0x02, 0x00, 0x7D, 0, 0, 0, 0 };
  BmsCan_HandleRx(CAN_ID_BMS_STATUS_1, frame, 8);

  CHECK_EQ(BmsCan_GetStatus().isAlive, 1);
  CHECK_EQ(BmsCan_GetStatus().lastUpdateTime_ms, 4242);
}

static void test_bms_rx_rejects_bad_length_unknown_id_and_null(void)
{
  BmsCan_Init();
  const uint8_t frame[8] = { 0x08, 0x02, 0x00, 0x7D, 0, 0, 0, 0 };

  CHECK_EQ(BmsCan_HandleRx(CAN_ID_BMS_STATUS_1, frame, 7), 0);
  CHECK_EQ(BmsCan_HandleRx(CAN_ID_MOTOR_STATUS_1, frame, 8), 0);
  CHECK_EQ(BmsCan_HandleRx(CAN_ID_BMS_STATUS_1, NULL, 8), 0);

  /* Rejected frames must not touch the status or keep the node alive. */
  CHECK_EQ(BmsCan_GetStatus().batteryVoltage_dV, 0);
  CHECK_EQ(BmsCan_GetStatus().isAlive, 0);
}

static void test_bms_command_payload_layout(void)
{
  BmsCommand_t cmd = { 0 };
  cmd.contactorRequest = 1U;
  cmd.chargeRequest = 1U;
  cmd.dischargeRequest = 0U;
  cmd.requestedChargeCurrent_dA = 0x1234U;
  cmd.aliveCounter = 7U;

  uint8_t data[8];
  uint8_t len = 0U;
  memset(data, 0xAA, sizeof(data));
  BmsCan_BuildCommandPayload(&cmd, data, &len);

  const uint8_t expected[8] = { 1, 1, 0, 0x34, 0x12, 7, 0, 0 };
  CHECK_EQ(len, 8);
  CHECK(memcmp(data, expected, sizeof(expected)) == 0);
}

/* -------------------------------------------------------------- Motor */

static void test_motor_status1_decodes_signed_rpm_and_current(void)
{
  MotorCan_Init();
  /* -1500 rpm (30500 = 0x7724), +25.0 A (32250 = 0x7DFA) */
  const uint8_t frame[8] = { 0x24, 0x77, 0xFA, 0x7D, 0, 0, 0, 0 };

  CHECK_EQ(MotorCan_HandleRx(CAN_ID_MOTOR_STATUS_1, frame, 8), 1);
  CHECK_EQ(MotorCan_GetStatus().motorRpm, -1500);
  CHECK_EQ(MotorCan_GetStatus().motorCurrent_dA, 250);
}

static void test_motor_status2_and_3_decode_state_direction_flags(void)
{
  MotorCan_Init();
  const uint8_t s2[8] = { 90 + 40, MOTOR_STATE_RUNNING, MOTOR_DIR_REVERSE, 0, 0, 0, 0, 0 };
  const uint8_t s3[8] = { 0x01, 200, 0, 0, 0, 0, 0, 0 };

  MotorCan_HandleRx(CAN_ID_MOTOR_STATUS_2, s2, 8);
  MotorCan_HandleRx(CAN_ID_MOTOR_STATUS_3, s3, 8);

  CHECK_EQ(MotorCan_GetStatus().motorTemp_C, 90);
  CHECK_EQ(MotorCan_GetStatus().motorState, MOTOR_STATE_RUNNING);
  CHECK_EQ(MotorCan_GetStatus().motorDirection, MOTOR_DIR_REVERSE);
  CHECK_EQ(MotorCan_GetStatus().faultActive, 1);
  CHECK_EQ(MotorCan_GetStatus().aliveCounter, 200);
}

static void test_temperature_decode_covers_full_wire_range(void)
{
  /* The temp byte carries -40..215 C. Values above 127 C used to wrap
     negative in an int8_t, hiding a hot motor from the fault check. */
  BmsCan_Init();
  MotorCan_Init();
  uint8_t frame[8] = { 0 };

  frame[0] = 130 + 40;
  MotorCan_HandleRx(CAN_ID_MOTOR_STATUS_2, frame, 8);
  CHECK_EQ(MotorCan_GetStatus().motorTemp_C, 130);

  frame[0] = 0xFF;
  MotorCan_HandleRx(CAN_ID_MOTOR_STATUS_2, frame, 8);
  CHECK_EQ(MotorCan_GetStatus().motorTemp_C, 215);

  frame[0] = 0x00;
  MotorCan_HandleRx(CAN_ID_MOTOR_STATUS_2, frame, 8);
  CHECK_EQ(MotorCan_GetStatus().motorTemp_C, -40);

  frame[0] = 0xFF;
  BmsCan_HandleRx(CAN_ID_BMS_STATUS_2, frame, 8);
  CHECK_EQ(BmsCan_GetStatus().batteryTemp_C, 215);
}

static void test_motor_command_payload_encodes_negative_torque(void)
{
  MotorCommand_t cmd = { 0 };
  cmd.enableRequest = 1U;
  cmd.direction = MOTOR_DIR_FORWARD;
  cmd.torqueRequest_Nm = -50;
  cmd.regenRequest_percent = 30U;
  cmd.aliveCounter = 9U;

  uint8_t data[8];
  uint8_t len = 0U;
  MotorCan_BuildCommandPayload(&cmd, data, &len);

  const uint8_t expected[8] = { 1, 1, 0xCE, 0xFF, 30, 9, 0, 0 };
  CHECK_EQ(len, 8);
  CHECK(memcmp(data, expected, sizeof(expected)) == 0);
}

/* ------------------------------------------------------------ Charger */

static void test_charger_status_decodes_all_frames(void)
{
  ChargerCan_Init();
  /* 56.0 V, 10.0 A (no offset: charger current is never negative) */
  const uint8_t s1[8] = { 0x30, 0x02, 0x64, 0x00, 0, 0, 0, 0 };
  const uint8_t s2[8] = { CHARGER_STATE_CHARGING, 1, 0, 0, 0, 0, 0, 0 };
  const uint8_t s3[8] = { 0x00, 42, 0, 0, 0, 0, 0, 0 };

  ChargerCan_HandleRx(CAN_ID_CHARGER_STATUS_1, s1, 8);
  ChargerCan_HandleRx(CAN_ID_CHARGER_STATUS_2, s2, 8);
  ChargerCan_HandleRx(CAN_ID_CHARGER_STATUS_3, s3, 8);

  CHECK_EQ(ChargerCan_GetStatus().chargerVoltage_dV, 560);
  CHECK_EQ(ChargerCan_GetStatus().chargerCurrent_dA, 100);
  CHECK_EQ(ChargerCan_GetStatus().chargerState, CHARGER_STATE_CHARGING);
  CHECK_EQ(ChargerCan_GetStatus().plugDetected, 1);
  CHECK_EQ(ChargerCan_GetStatus().faultActive, 0);
  CHECK_EQ(ChargerCan_GetStatus().aliveCounter, 42);
}

static void test_charger_command_payload_layout(void)
{
  ChargerCommand_t cmd = { 0 };
  cmd.chargeEnable = 1U;
  cmd.targetVoltage_dV = DEFAULT_CHARGE_TARGET_VOLTAGE_dV;
  cmd.targetCurrent_dA = DEFAULT_CHARGE_TARGET_CURRENT_dA;
  cmd.aliveCounter = 3U;

  uint8_t data[8];
  uint8_t len = 0U;
  ChargerCan_BuildCommandPayload(&cmd, data, &len);

  const uint8_t expected[8] = { 1, 0x30, 0x02, 0x64, 0x00, 3, 0, 0 };
  CHECK_EQ(len, 8);
  CHECK(memcmp(data, expected, sizeof(expected)) == 0);
}

void run_can_codec_tests(void)
{
  RUN_TEST(test_bms_status1_decodes_voltage_and_discharge_current);
  RUN_TEST(test_bms_status1_decodes_negative_charge_current);
  RUN_TEST(test_bms_status2_decodes_temp_soc_and_state);
  RUN_TEST(test_bms_status3_unpacks_each_flag_bit);
  RUN_TEST(test_bms_rx_marks_node_alive_with_timestamp);
  RUN_TEST(test_bms_rx_rejects_bad_length_unknown_id_and_null);
  RUN_TEST(test_bms_command_payload_layout);

  RUN_TEST(test_motor_status1_decodes_signed_rpm_and_current);
  RUN_TEST(test_motor_status2_and_3_decode_state_direction_flags);
  RUN_TEST(test_temperature_decode_covers_full_wire_range);
  RUN_TEST(test_motor_command_payload_encodes_negative_torque);

  RUN_TEST(test_charger_status_decodes_all_frames);
  RUN_TEST(test_charger_command_payload_layout);
}
