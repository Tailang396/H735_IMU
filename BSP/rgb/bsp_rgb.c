#include "bsp_rgb.h"
#include "bsp_def.h"
#include "spi.h"

#define WS2812_HIGH    0xf0
#define WS2812_LOW     0xc0
#define WS2812_PORT    &hspi6

static uint8_t buf[24] RAM_D3;
static uint8_t dma_buf[24] __attribute__((section(".ram_d3")));

// __attribute__((section(".itcm_text"), noinline))
void bsp_rgb_set(uint8_t r, uint8_t g, uint8_t b) {
    for(int i = 0; i < 8; i++) {
        buf[7-i]  = (((g >> i) & 1) ? WS2812_HIGH : WS2812_LOW) >> 1;
        buf[15-i] = (((r >> i) & 1) ? WS2812_HIGH : WS2812_LOW) >> 1;
        buf[23-i] = (((b >> i) & 1) ? WS2812_HIGH : WS2812_LOW) >> 1;
    }
    HAL_SPI_Transmit(WS2812_PORT, buf, 24, 10);
}

void bsp_rgb_set_dma(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < 8; i++) {
        dma_buf[7-i]  = (((g >> i) & 1) ? WS2812_HIGH : WS2812_LOW) >> 1;
        dma_buf[15-i] = (((r >> i) & 1) ? WS2812_HIGH : WS2812_LOW) >> 1;
        dma_buf[23-i] = (((b >> i) & 1) ? WS2812_HIGH : WS2812_LOW) >> 1;
    }
    HAL_SPI_Transmit_DMA(WS2812_PORT, dma_buf, sizeof(dma_buf));
}
