
#include "app.h"

#include "main.h"
#include "ev_config.h"

#include "bms_can.h"
#include "motor_can.h"
#include "charger_can.h"
#include "vehicle_control.h"
#include "led_diag.h"
#include "can_bus.h"

#include <stdint.h>
#include <stdio.h>

/*
 * Retarget printf to USART2 (115200 8N1) on PA2 (TX) / PA3 (RX).
 * syscalls.c's _write() calls __io_putchar() for each byte; we
 * push that byte out over the UART here.
 *
 * To view the output: connect a USB-TTL adapter (PA2 -> adapter RX,
 * GND -> GND) and open a 115200 8N1 serial terminal on the host.
 */
extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;

int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart2, &c, 1U, HAL_MAX_DELAY);
  return ch;
}

/*
 * ============================================================
 *  ECU outgoing command payloads (TX)
 * ============================================================
 *
 * Built every loop from the vehicle controller and transmitted
 * on CAN (one frame per node).
 */
static uint8_t latestBmsCommandPayload[8];
static uint8_t latestMotorCommandPayload[8];
static uint8_t latestChargerCommandPayload[8];

static uint8_t latestBmsCommandLen = 0U;
static uint8_t latestMotorCommandLen = 0U;
static uint8_t latestChargerCommandLen = 0U;

/*
 * Dump a single CAN frame to the console, e.g.:
 *   TX 0x280 [8] 01 00 01 00 00 05 00 00
 */
static void App_PrintCanFrame(const char *tag, uint32_t id, const uint8_t *data, uint8_t len)
{
  printf("%s 0x%03lX [%u]", tag, (unsigned long)id, (unsigned)len);

  for (uint8_t i = 0U; i < len; i++)
  {
    printf(" %02X", data[i]);
  }

  printf("\r\n");
}

/*
 * Print a 0.1-unit fixed-point value as a decimal, e.g.
 *   520 -> "52.0",  -125 -> "-12.5",  -5 -> "-0.5"
 */
static void App_PrintDeci(int32_t tenths)
{
  int32_t whole = tenths / 10;
  int32_t frac  = tenths % 10;

  if (frac < 0)
  {
    frac = -frac;
  }

  if (tenths < 0 && whole == 0)
  {
    printf("-%ld.%ld", (long)whole, (long)frac);  /* keep the sign for -0.x */
  }
  else
  {
    printf("%ld.%ld", (long)whole, (long)frac);
  }
}

/*
 * Dump the decoded status of every node in human-readable form.
 * The status structs are filled by the CAN RX interrupt.
 */
static void App_TraceDecoded(void)
{
  CanBus_LockStatus();
  BmsStatus_t b = BmsCan_GetStatus();
  MotorStatus_t m = MotorCan_GetStatus();
  ChargerStatus_t c = ChargerCan_GetStatus();
  CanBus_UnlockStatus();

  printf("  BMS: ");
  App_PrintDeci(b.batteryVoltage_dV);
  printf("V ");
  App_PrintDeci(b.batteryCurrent_dA);
  printf("A %dC SOC%u%% st%u alive%u [F%u Ch%u Dis%u K%u]\r\n",
         (int)b.batteryTemp_C, (unsigned)b.soc_percent, (unsigned)b.bmsState,
         (unsigned)b.isAlive, (unsigned)b.faultActive, (unsigned)b.chargeAllowed,
         (unsigned)b.dischargeAllowed, (unsigned)b.contactorClosed);

  printf("  MOT: %d rpm ", (int)m.motorRpm);
  App_PrintDeci(m.motorCurrent_dA);
  printf("A %dC st%u dir%u alive%u [F%u]\r\n",
         (int)m.motorTemp_C, (unsigned)m.motorState,
         (unsigned)m.motorDirection, (unsigned)m.isAlive, (unsigned)m.faultActive);

  printf("  CHG: ");
  App_PrintDeci((int32_t)c.chargerVoltage_dV);
  printf("V ");
  App_PrintDeci((int32_t)c.chargerCurrent_dA);
  printf("A st%u plug%u alive%u [F%u]\r\n",
         (unsigned)c.chargerState, (unsigned)c.plugDetected,
         (unsigned)c.isAlive, (unsigned)c.faultActive);
}

/*
 * Diagnostics step (run by DiagTask): dump the TX command frames and
 * the decoded RX status. No internal throttle anymore -- the task's
 * osDelay() sets the rate.
 */
void App_DiagStep(void)
{
  printf("---- CAN @ %lu ms ----\r\n", (unsigned long)HAL_GetTick());

  /* TX command frames (ECU -> node) */
  App_PrintCanFrame("TX", CAN_ID_BMS_COMMAND,     latestBmsCommandPayload,     latestBmsCommandLen);
  App_PrintCanFrame("TX", CAN_ID_MOTOR_COMMAND,   latestMotorCommandPayload,   latestMotorCommandLen);
  App_PrintCanFrame("TX", CAN_ID_CHARGER_COMMAND, latestChargerCommandPayload, latestChargerCommandLen);

  /* Decoded view of what we received */
  App_TraceDecoded();
}

/*
 * Build the ECU -> node command payloads and transmit them on CAN.
 *
 * The control loop runs every 10 ms, but commands only go out every
 * CAN_TX_PERIOD_MS: the frames always carry the latest decision
 * without flooding the bus. The alive counter advances once per
 * transmitted set, so a node can detect missing/stale frames.
 */
static void App_SendEcuCommands(void)
{
  static uint32_t lastTxTime_ms = 0U;
  static uint8_t txAliveCounter = 0U;
  static uint8_t hasSent = 0U;

  uint32_t now = HAL_GetTick();

  /* First call sends right away, then once per CAN_TX_PERIOD_MS. */
  if (hasSent != 0U && (now - lastTxTime_ms) < CAN_TX_PERIOD_MS)
  {
    return;
  }
  lastTxTime_ms = now;
  hasSent = 1U;

  BmsCommand_t bmsCommand = VehicleControl_GetBmsCommand();
  MotorCommand_t motorCommand = VehicleControl_GetMotorCommand();
  ChargerCommand_t chargerCommand = VehicleControl_GetChargerCommand();

  bmsCommand.aliveCounter = txAliveCounter;
  motorCommand.aliveCounter = txAliveCounter;
  chargerCommand.aliveCounter = txAliveCounter;
  txAliveCounter++;

  BmsCan_BuildCommandPayload(&bmsCommand, latestBmsCommandPayload, &latestBmsCommandLen);
  MotorCan_BuildCommandPayload(&motorCommand, latestMotorCommandPayload, &latestMotorCommandLen);
  ChargerCan_BuildCommandPayload(&chargerCommand, latestChargerCommandPayload, &latestChargerCommandLen);

  CanBus_SendFrame(CAN_ID_BMS_COMMAND,     latestBmsCommandPayload,     latestBmsCommandLen);
  CanBus_SendFrame(CAN_ID_MOTOR_COMMAND,   latestMotorCommandPayload,   latestMotorCommandLen);
  CanBus_SendFrame(CAN_ID_CHARGER_COMMAND, latestChargerCommandPayload, latestChargerCommandLen);
}

void App_Init(void)
{
  /*
   * Unbuffered stdout so each byte is pushed straight to the UART
   * (newlib-nano line buffering does not reliably flush on '\n').
   */
  setvbuf(stdout, NULL, _IONBF, 0);

  BmsCan_Init();
  MotorCan_Init();
  ChargerCan_Init();

  VehicleControl_Init();
  LedDiag_Init();

  /* NOTE: CAN (queue/mutex/start) is set up after the RTOS kernel is
     initialized -- see CanBus_Init() in main and CanBus_Start() in
     CanRxTask. App_Init runs before the scheduler, so we don't create
     RTOS objects here. */

  LedDiag_Update(VehicleControl_GetState());
}

void App_ControlStep(void)
{
  /*
   * 1. Read latest decoded CAN status (filled by CanRxTask). Take the
   *    mutex so we copy a consistent snapshot, not a half-updated one.
   */
  CanBus_LockStatus();
  BmsStatus_t bmsStatus = BmsCan_GetStatus();
  MotorStatus_t motorStatus = MotorCan_GetStatus();
  ChargerStatus_t chargerStatus = ChargerCan_GetStatus();
  CanBus_UnlockStatus();

  /*
   * 2. ECU makes control decision and updates command structs.
   */
  VehicleControl_UpdateFromCan(
      bmsStatus,
      motorStatus,
      chargerStatus
  );

  /*
   * 3. Build command payloads and transmit them on CAN
   *    (rate-limited to CAN_TX_PERIOD_MS inside).
   */
  App_SendEcuCommands();

  /*
   * 4. LED diagnostics follow ECU state.
   */
  LedDiag_Update(VehicleControl_GetState());
}
