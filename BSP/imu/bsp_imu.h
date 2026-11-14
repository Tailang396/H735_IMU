//
// Created by Yu_Jie on 2025/12/20.
//

#ifndef BSP_IMU_H
#define BSP_IMU_H
#include "main.h"
#include "bsp_def.h"

#define IMU_DISABLE 0
#define IMU_ENABLE  1

#define BSP_IMU_DEBUG
#define BSP_IMU_INFO
#define IMU_DEBUG_UART E_UART_DEBUG
#define IMU_INFO_UART E_UART_DEBUG

typedef struct {
    float mag_temp;
    float imu_temp;
    float bsp_temp;
} imu_temp_data_t;

typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
} imu_acc_data_t;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} imu_gyro_data_t;


typedef struct {
    float mag_x;
    float mag_y;
    float mag_z;
} imu_mag_data_t;

typedef struct {
    imu_acc_data_t raw_acc;
    imu_gyro_data_t raw_gyro;
    imu_mag_data_t raw_mag;
} imu_raw_data_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} imu_angle_data_t;

typedef struct {
    float x;
    float y;
    float z;
} imu_earth_acc_data_t;

typedef struct {
    uint8_t MAG_STATE;            //是否启用磁力计
    uint16_t CALI_SAMPLE_TIMES;   //校准采样次数 0是不校准  500够用
    uint16_t TMEP_CTRL_STATE;     //温度控制状态
} IMU_InitTypeDef;

typedef struct {
    imu_angle_data_t angle;
    imu_earth_acc_data_t earth_acc;
    imu_raw_data_t raw_data;
    imu_temp_data_t temp;
} imu_data_t;

#ifdef __cplusplus
extern "C" {
#endif

void bsp_imu_init(TIM_HandleTypeDef *htim);
// 在非中断上下文调用：写入成功后等待 5 秒并重启。
void bsp_imu_request_gyro_calibration(uint16_t sample_times);
imu_data_t* bsp_imu_GetData(void);
// void bsp_imu_set_mode(void);
// void bsp_tempctrl_init(void);
// void bsp_tempctrl_update(void);
void fusion_update(void);

#ifdef __cplusplus
}
#endif

#endif
