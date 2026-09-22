/*
 * Host-side stand-in for the STM32 HAL.
 *
 * The CAN codecs and the vehicle controller only need HAL_GetTick()
 * from the HAL, so the tests provide a controllable fake clock
 * instead of the real driver.
 */

#ifndef STM32F4XX_HAL_H
#define STM32F4XX_HAL_H

#include <stdint.h>

uint32_t HAL_GetTick(void);

#endif /* STM32F4XX_HAL_H */
