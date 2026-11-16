//
// Created by asus on 2026/11/8.
//

#include "app_imu.h"
#include "bsp_imu.h"

static uint8_t count = 0;

void app_imu_update(void) {
    fusion_update();
    if (++count == 3) {
        count = 0;
        tempctrl_update();

    }
}
