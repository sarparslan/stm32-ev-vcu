#include "stm32f4xx_hal.h"
#include "fake_hal.h"

static uint32_t fakeTick_ms = 0U;

uint32_t HAL_GetTick(void)
{
  return fakeTick_ms;
}

void FakeHal_SetTick(uint32_t tick_ms)
{
  fakeTick_ms = tick_ms;
}
