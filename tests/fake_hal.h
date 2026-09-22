#ifndef FAKE_HAL_H
#define FAKE_HAL_H

#include <stdint.h>

/* Set the value returned by HAL_GetTick(). */
void FakeHal_SetTick(uint32_t tick_ms);

#endif /* FAKE_HAL_H */
