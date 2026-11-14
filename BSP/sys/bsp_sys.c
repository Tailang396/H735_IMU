#include "bsp_sys.h"
#include "tim.h"
#include <stdint.h>

void delay_init(void) {
    if ((htim7.Instance->CR1 & TIM_CR1_CEN) != 0U) {
        return;
    }
    if (HAL_TIM_Base_Start(&htim7) != HAL_OK) {
        Error_Handler();
    }
}

void delay_us(uint16_t us) {
    if (us == 0) {
        return;
    }
    uint16_t previous = (uint16_t)__HAL_TIM_GET_COUNTER(&htim7);
    uint32_t elapsed = 0;
    while (elapsed < us) {
        uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(&htim7);
        elapsed += (uint16_t)(current - previous);
        previous = current;
    }
}

void delay_u32_us(uint32_t us) {
    uint32_t remaining = us;
    const uint16_t BOUNDARY = 65000;
    while (remaining > BOUNDARY) {
        delay_us(BOUNDARY);
        remaining -= BOUNDARY;
    }
    if (remaining > 0) {
        delay_us((uint16_t)remaining);
    }
}

void delay_ms(uint16_t ms) {
    while (ms--) {
        delay_us(1000);
    }
}
