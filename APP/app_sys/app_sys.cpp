//
// Created by asus on 2026/11/4.
//

#include "app_sys.h"
#include "bsp_rgb.h"
#include "bsp_sys.h"
#include "bsp_uart.h"
#include <cstring>
#include <cstdio>

#include "bsp_flash.h"
#include "bsp_imu.h"
#include "icm45686.h"
#include "tim.h"
#include "tmp117.h"

uint8_t sn[32] = {};
uint8_t reset_flag = 0;

/* 从 Flash 复制函数代码到 ITCM，必须早于第一次调用。 */
static void itcm_init();
static void uart_debug_callback(bsp_uart_e e, uint8_t *s, uint16_t l);
static void rgb_update();


void sys_main_init(void) {
    itcm_init();
    delay_init();
    bsp_uart_init(E_UART_DEBUG, &huart1);
    bsp_uart_set_callback(E_UART_DEBUG, uart_debug_callback);
    bsp_uart_set_callback(E_UART_CDC, uart_debug_callback);
    bsp_flash_init();
    // bsp_flash_SetDefault();
    bsp_flash_print_status();
    bsp_flash_read("SN", sn, sizeof(sn) - 1);
    bsp_uart_printf(E_UART_DEBUG, "[BSP] SN: %s\r\n", sn);

    bsp_imu_init(&htim6);

    bsp_rgb_set(0,50,0);
    HAL_Delay(2000);
}

void sys_main_while(void) {
    rgb_update();

    const imu_data_t *imu = bsp_imu_GetData();
    bsp_uart_printf(E_UART_DEBUG,
                    "ANGLE(deg),TEMP(C),GYRO(dps): %.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\r\n",
                    imu->angle.roll, imu->angle.pitch, imu->angle.yaw,
                    imu->temp.imu_temp,
                    imu->raw_data.raw_gyro.gyro_x, imu->raw_data.raw_gyro.gyro_y,
                    imu->raw_data.raw_gyro.gyro_z);
    if (reset_flag == 1) bsp_imu_request_gyro_calibration(6000);
    delay_ms(10);
}

void uart_debug_callback(bsp_uart_e e, uint8_t *s, uint16_t l) {
    if (memcmp(s, "imu cali", 8) == 0) {
        reset_flag = 1;
    }
    bsp_uart_send(e, s, l);
}

static void rgb_update() {
    static uint32_t last_tick = HAL_GetTick();
    static const uint32_t start_tick = last_tick;
    static const uint8_t colors[][3] = {
        {1, 0, 0},  // 红
        {1, 1, 0},  // 黄
        {0, 1, 0},  // 绿
        {0, 1, 1},  // 青
        {0, 0, 1},  // 蓝
        {1, 0, 1},  // 紫
    };

    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_tick) >= 10U) {
        last_tick = now;

        /* 每种颜色呼吸 2 秒：渐亮 1 秒、渐暗 1 秒，在熄灭时换色。 */
        const uint32_t elapsed = now - start_tick;
        const uint32_t color = (elapsed / 2000U) % 6U;
        const uint32_t phase = elapsed % 2000U;
        const uint32_t level = (phase <= 1000U ? phase : 2000U - phase) / 10U;

        /* 平滑曲线使亮暗转折更柔和，保持原先的最大通道亮度 50。 */
        const uint8_t brightness = static_cast<uint8_t>(
            50U * level * level * (300U - 2U * level) / 1000000U);
        bsp_rgb_set(colors[color][0] * brightness,
                    colors[color][1] * brightness,
                    colors[color][2] * brightness);

    }
}



/* 链接脚本导出的 ITCM 代码地址，使用 C 链接名称。 */
extern "C" {
extern uint8_t _siitcm[];
extern uint8_t _sitcm[];
extern uint8_t _eitcm[];
}

static void itcm_init() {
    SCB->ITCMCR |= SCB_ITCMCR_EN_Msk;
    __DSB();
    __ISB();
    /* 从 Flash 复制函数代码到 ITCM，必须早于第一次调用。 */
    const size_t size = reinterpret_cast<uintptr_t>(_eitcm) -
                        reinterpret_cast<uintptr_t>(_sitcm);
    memcpy(_sitcm, _siitcm, size);
    __DSB();
    __ISB();
}
