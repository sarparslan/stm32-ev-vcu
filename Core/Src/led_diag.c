
#include "led_diag.h"
#include "ev_config.h"
#include "main.h"

static uint32_t lastFaultBlinkTime = 0U;
static GPIO_PinState faultLedState = GPIO_PIN_RESET;

static void TurnOffAllLeds(void)
{
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Green
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Orange
  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Red
  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Blue
}

static void ShowSolidState(VehicleState_t state)
{
  TurnOffAllLeds();

  switch (state)
  {
    case VEHICLE_STATE_IDLE:
      HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET); // Green
      break;

    case VEHICLE_STATE_READY:
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); // Orange
      break;

    case VEHICLE_STATE_DRIVE:
      HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET); // Blue
      break;

    case VEHICLE_STATE_CHARGING:
      HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET); // Green
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); // Orange
      break;

    default:
      break;
  }
}

static void HandleFaultBlink(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - lastFaultBlinkTime) >= FAULT_LED_BLINK_PERIOD_MS)
  {
    lastFaultBlinkTime = now;

    faultLedState = (faultLedState == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, faultLedState); // Red
  }
}

void LedDiag_Init(void)
{
  TurnOffAllLeds();

  lastFaultBlinkTime = HAL_GetTick();
  faultLedState = GPIO_PIN_RESET;

  ShowSolidState(VEHICLE_STATE_IDLE);
}
void LedDiag_Update(VehicleState_t state)
{
  static VehicleState_t previousState = VEHICLE_STATE_IDLE;

  if (state == VEHICLE_STATE_FAULT)
  {
    if (previousState != VEHICLE_STATE_FAULT)
    {
      TurnOffAllLeds();
      faultLedState = GPIO_PIN_RESET;
      lastFaultBlinkTime = HAL_GetTick();
    }

    HandleFaultBlink();
  }
  else
  {
    if (state != previousState)
    {
      ShowSolidState(state);
    }
  }

  previousState = state;
}
