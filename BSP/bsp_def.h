//
// Created by fish on 2024/9/2.
//

#pragma once

#include "bsp_rgb.h"
// #include "FreeRTOS.h"
// #include "task.h"
#include "stdint.h"
#include "bsp_uart.h"
// #include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

// #define BSP_USE_FREERTOS

#define RAM_D1 __attribute__((section(".ram_d1"), aligned(4)))
#define RAM_D2 __attribute__((section(".ram_d2"), aligned(4)))
#define RAM_D3 __attribute__((section(".ram_d3"), aligned(4)))

__attribute__((unused)) static void bsp_assert_err(const char *file, uint32_t line) {
    static uint8_t printing = 0;
    if (!printing) {
        printing = 1;
        bsp_uart_printf(E_UART_DEBUG, "[ASSERT] %s:%lu\r\n", file, (unsigned long)line);
    }
    // 开启 rtos 调度锁，强行停止其他任务，便于调试。
    // vTaskSuspendAll();
    __disable_irq();
    bsp_rgb_set(255, 0, 0);
    while(1)
        __NOP();
}

#define BSP_ASSERT(arg)                                                                                                \
    do {                                                                                                               \
        if(!(arg)) bsp_assert_err(__FILE__, __LINE__);                                                                 \
    } while(0)

typedef enum {
    BSP_OK  = 0,
    BSP_ERR = 1,
} bsp_status_t;

#ifdef __cplusplus
}
#endif
