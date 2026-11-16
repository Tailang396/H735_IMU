#ifndef BSP_RGB_H
#define BSP_RGB_H

#include "stdint.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void bsp_rgb_set(uint8_t r, uint8_t g, uint8_t b);
void bsp_rgb_set_dma(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif //BSP_RGB_H
