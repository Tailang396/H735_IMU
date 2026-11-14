/**
 * @file bsp_imu.c
 * @brief BSP_IMU库 包含hscdtd008磁力计 icm45686陀螺仪驱动
 * 可以配置是否使用磁力计，磁力计校准参数请在此处设置
 *
 * @author Yu_Jie
 * @date 2026/3/17
 */

#include "bsp_imu.h"
#include <stdbool.h>
#include "Fusion.h"
#include "hscdtd008a.h"
#include "icm45686.h"
#include "tmp117.h"
#include "bsp_flash.h"
#include "bsp_sys.h"

// 接入 CMSIS-RTOS2 后，在编译选项中定义 BSP_IMU_USE_RTOS=1。
#ifdef BSP_IMU_USE_RTOS
#include "cmsis_os2.h"
#define BSP_IMU_RESTART_DELAY(ms) osDelay(ms)
#else
#define BSP_IMU_RESTART_DELAY(ms) HAL_Delay(ms)
#endif

#ifdef BSP_IMU_DEBUG
#define BSP_IMU_DEBUG_PRINT(...) bsp_uart_printf(IMU_DEBUG_UART, __VA_ARGS__)
#else
#define BSP_IMU_DEBUG_PRINT(fmt, ...) do { } while (0)
#endif

#ifdef BSP_IMU_INFO
#define BSP_IMU_INFO_PRINT(...) bsp_uart_printf(IMU_INFO_UART, __VA_ARGS__)
#else
#define BSP_IMU_INFO_PRINT(fmt, ...) do { } while (0)
#endif


/**
 * @brief IMU 初始化配置结构体
 * 该结构体用于配置 IMU 设备的初始化参数，包括校准采样次数、磁力计状态和温度控制状态。
 * 作为静态常量，该配置在程序运行期间保持不变。
 */
static IMU_InitTypeDef imu_init = {
    .CALI_SAMPLE_TIMES = 600,
    .MAG_STATE = IMU_DISABLE,
    .TMEP_CTRL_STATE = IMU_DISABLE
};

#define abs(x) ((x) > 0 ? (x) : -(x))

static imu_parameters_t fusion_imu_param;
static bool fusion_init_finsh_flag = false;
static uint8_t gyro_calibration_requested = 0;

// uint8_t imu_tempctrl_finsh_flag = 1;
static imu_data_raw_t imu_data_raw;
static HSCDTD008A_data_t mag_data;
static imu_data_t imu_data = {0};

// static float tmp117_temp;
// static float aim_temp = 40.0f;
// static float temp_stable_threshold = 0.1f;
// static uint16_t stable_count_required = 200;
// PID_TypeDef pid_temp;

static void fusion_init(void);
static bsp_status_t imu_load_calibration(void);


// Calibration parameters (replace with actual calibration data)
static const FusionMatrix gyroscopeMisalignment = {{1.0f, 0.0f, 0.0f,
                                                            0.0f, 1.0f, 0.0f,
                                                            0.0f, 0.0f, 1.0f}};
static const FusionVector gyroscopeSensitivity = {{1.0f, 1.0f, 1.0f}};
static FusionVector gyroscopeOffset = {{0.0f, 0.0f, 0.0f}};

static const FusionMatrix softIronMatrix = {{0.9744618f,  0.0301350f, 0.00280765f,
                                                     0.0301350f,  1.0356935f, -0.0542281f,
                                                     0.0028076f,  -0.054228f,  0.9945922f}};
static const FusionVector hardIronOffset = {{7.6314566f,  -3.095440f, -7.6259514f}};

// Initialise structures
static FusionBias bias;
static FusionAhrs ahrs;
static FusionAhrsFlags flags = {
    .initialising = true
};


/**
 * @brief BSP IMU 初始化函数
 * 必须在OS开启之前调用，定时器中断优先级最高，初始化时间较长
 */
void bsp_imu_init(TIM_HandleTypeDef *htim) {
    imu_status_e imu_st =  ICM45686_Init();
    BSP_IMU_INFO_PRINT("[IMU] ICM45686 Init %s\r\n",
                        imu_st == ICM45686_INIT_OK ? "SUCCESS" : "FAIL");
    HSCDTD008A_status_t mag_st = hscdtd008a_Init();
    BSP_IMU_INFO_PRINT("[IMU] HSCDTD008A Init %s\r\n",
                        mag_st == HSCDTD008A_NO_ERR ? "SUCCESS" : "FAIL");
    TMP117_Status_t temp_st = tmp117_Init();
    BSP_IMU_INFO_PRINT("[IMU] TMP117 Init %s\r\n",
                        temp_st == TMP117_NO_ERR ? "SUCCESS" : "FAIL");
    (void)imu_st; (void)mag_st; (void)temp_st;

    bsp_status_t imu_cali_st = imu_load_calibration();
    BSP_ASSERT(imu_cali_st == BSP_OK);
    fusion_init();
    BSP_IMU_INFO_PRINT("[IMU] Fusion algorithm Init %s\r\n",
                        fusion_init_finsh_flag == true ? "SUCCESS" : "FAIL");
    HAL_TIM_Base_Start_IT(htim);
}


imu_data_t* bsp_imu_GetData(void) {
    return &imu_data;
}

/**
 * @brief 保存陀螺仪零偏校准请求，在下次启动时执行校准。
 * @param sample_times 校准采样次数，必须大于 0。
 * @note 写入成功后等待 5 秒并重启；写入失败时返回，不重启。
 *       请在非中断上下文调用，并在重启后校准期间保持设备静止。
 */
void bsp_imu_request_gyro_calibration(uint16_t sample_times) {
    if (sample_times == 0) {
        BSP_IMU_INFO_PRINT("[IMU] Gyroscope calibration sample count must be greater than zero\r\n");
        return;
    }
    if (bsp_flash_write("imu_cali_time", &sample_times, sizeof(sample_times)) != BSP_OK) {
        BSP_IMU_INFO_PRINT("[IMU] Gyroscope calibration sample count write FAIL\r\n");
    }
    gyro_calibration_requested = 0xff;
    if (bsp_flash_write("imu_cali_req", &gyro_calibration_requested,
                        sizeof(gyro_calibration_requested)) != BSP_OK) {
        BSP_IMU_INFO_PRINT("[IMU] Gyroscope calibration request write FAIL\r\n");
        gyro_calibration_requested = 0;
        return;
    }
    BSP_IMU_INFO_PRINT("[IMU] Gyroscope calibration request write SUCCESS, samples: %u\r\n", (unsigned int)sample_times);
    BSP_IMU_INFO_PRINT("[IMU] Restart manually, or the board will restart in 5 seconds\r\n");
    BSP_IMU_RESTART_DELAY(5000);
    bsp_sys_reset();
}

/**
 * @brief 根据 Flash 中的请求执行陀螺仪零偏校准，或读取已保存的校准参数。
 * @return BSP_OK 表示校准或读取成功；BSP_ERR 表示读取、校准或写入失败。
 * @note 执行校准期间需要保持设备静止。
 */
static bsp_status_t imu_load_calibration(void) {
    uint32_t request_len = bsp_flash_read("imu_cali_req", &gyro_calibration_requested,sizeof(gyro_calibration_requested));
    if (request_len == sizeof(gyro_calibration_requested) && gyro_calibration_requested == 0xff) {
        uint16_t sample_times = 0;
        uint32_t sample_len = bsp_flash_read("imu_cali_time", &sample_times, sizeof(sample_times));
        if (sample_len != sizeof(sample_times)) {
            BSP_IMU_INFO_PRINT("[IMU] Gyroscope calibration parameters invalid\r\n");
            return BSP_ERR;
        }
        BSP_IMU_INFO_PRINT("[IMU] Calibrating gyroscope, keep the board stationary (%u samples)\r\n", (unsigned int)sample_times);
        if (ICM45686_SoftCalibrate(sample_times) != ICM45686_CALI_OK) {
            BSP_IMU_INFO_PRINT("[IMU] Gyroscope calibration FAIL\r\n");
            return BSP_ERR;
        }
        fusion_imu_param = ICM45686_GetParameter();
        if (bsp_flash_write("imu_offset", &fusion_imu_param.offset_data, sizeof(fusion_imu_param.offset_data)) != BSP_OK) {
            BSP_IMU_INFO_PRINT("[IMU] Gyroscope offset write FAIL\r\n");
            return BSP_ERR;
        }
        gyro_calibration_requested = 0;
        if (bsp_flash_write("imu_cali_req", &gyro_calibration_requested, sizeof(gyro_calibration_requested)) != BSP_OK) {
            BSP_IMU_INFO_PRINT("[IMU] Gyroscope calibration flag clear FAIL\r\n");
            return BSP_ERR;
        }
        BSP_IMU_INFO_PRINT("[IMU] Gyroscope calibration and offset save SUCCESS\r\n");
    } else {
        fusion_imu_param = ICM45686_GetParameter();
        uint32_t len = bsp_flash_read("imu_offset", &fusion_imu_param.offset_data, sizeof(fusion_imu_param.offset_data));
        if (len == sizeof(fusion_imu_param.offset_data) &&
            fusion_imu_param.offset_data.gyro_offset_checksum == imu_offset_checksum(&fusion_imu_param.offset_data)) {
            BSP_IMU_INFO_PRINT("[IMU] Gyroscope Offset data read SUCCESS\r\n");
        } else {
            fusion_imu_param.offset_data = (imu_offset_data_t){0};
            BSP_IMU_INFO_PRINT("[IMU] Gyroscope Offset data read FAIL. Please run the calibration command manually.\r\n");
            return BSP_ERR;
        }
    }
    return BSP_OK;
}

// void bsp_tempctrl_init(void) {
//     PID_Init(&pid_temp, 150.0f, 0.4f, 0.0f, 400, 500);
//     pid_temp.polarity = PID_UNIPOLAR;
//     HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
// }
//
// void bsp_tempctrl_update(void) {
//     if (imu_init.TMEP_CTRL_STATE == IMU_ENABLE) {
//         if (app_sensor_GetTMP117OSFlag() == 0)
//             tmp117_temp = tmp117_GetData();
//         else
//             tmp117_temp = app_sensor_GetTMP117Data();
//         if (tmp117_temp == 0) tmp117_temp = aim_temp;
//         PID_Calculate(&pid_temp, aim_temp, tmp117_temp);
//         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint16_t)pid_temp.PIDout);
//         if (imu_tempctrl_finsh_flag == 0) {
//             static uint16_t count = 0;
//             bsp_uart_printf(E_UART_DEBUG, "%.1f, %.1f, %f\r\n", tmp117_temp, aim_temp, pid_temp.PIDout);
//             if (abs(tmp117_temp - aim_temp) <= temp_stable_threshold) {
//                 count++;
//                 if (count >= stable_count_required) {
//                     imu_tempctrl_finsh_flag = 1;
//                 }
//             }
//             else {
//                 count = 0;
//             }
//         }
//     }
// }

static void fusion_init(void) {
    FusionBiasInitialise(&bias, (uint32_t)fusion_imu_param.sample_freq);
    FusionAhrsInitialise(&ahrs);
    gyroscopeOffset.array[0] = fusion_imu_param.offset_data.gyro_offset[0];
    gyroscopeOffset.array[1] = fusion_imu_param.offset_data.gyro_offset[1];
    gyroscopeOffset.array[2] = fusion_imu_param.offset_data.gyro_offset[2];
    // Set AHRS settings
    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,               // 地球坐标系：北(X)、西(Y)、上(Z)
        .gain = 0.5f,                                    // 加速度计和磁力计对姿态的反馈校正增益
        .gyroscopeRange = fusion_imu_param.gyro_range,   // 陀螺仪量程，单位：°/s
        .accelerationRejection = 10.0f,                  // 加速度计拒绝阈值：与估计重力方向的夹角，单位：度
        .magneticRejection = 10.0f,                      // 磁力计拒绝阈值：磁场方向误差角，单位：度
        .recoveryTriggerPeriod = 5 * (uint32_t)fusion_imu_param.sample_freq, // 异常恢复触发周期，单位：采样次数
    };
    FusionAhrsSetSettings(&ahrs, &settings);
    fusion_init_finsh_flag = true;
}


void fusion_update(void) {
    if (fusion_init_finsh_flag == true) {
        ICM45686_GetRawData(&imu_data_raw);
        hscdtd008a_GetStatus(&mag_data);
        if (mag_data.status.update_state)
            hscdtd008a_GetData(&mag_data);
        FusionVector gyroscope = {{imu_data_raw.gyro_x, imu_data_raw.gyro_y, imu_data_raw.gyro_z}};
        FusionVector accelerometer = {{imu_data_raw.acc_x, imu_data_raw.acc_y, imu_data_raw.acc_z}};
        FusionVector magnetometer = {{mag_data.mag_x, mag_data.mag_y, mag_data.mag_z}};
        // Apply calibration
        gyroscope = FusionModelInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        // accelerometer = FusionModelInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
        if (imu_init.MAG_STATE == IMU_ENABLE) {
            magnetometer = FusionModelMagnetic(magnetometer, softIronMatrix, hardIronOffset);
        }
        // Update bias algorithm
        gyroscope = FusionBiasUpdate(&bias, gyroscope);
        // Update AHRS algorithm
        if (imu_init.MAG_STATE == IMU_ENABLE) {
            FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, fusion_imu_param.deltaTime);
        } else {
            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, fusion_imu_param.deltaTime);
        }
        // Print AHRS outputs
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
        if (flags.initialising == true) {
            flags = FusionAhrsGetFlags(&ahrs);
        } else {
            imu_data.raw_data.raw_acc.acc_x = imu_data_raw.acc_x;
            imu_data.raw_data.raw_acc.acc_y = imu_data_raw.acc_y;
            imu_data.raw_data.raw_acc.acc_z = imu_data_raw.acc_z;
            imu_data.raw_data.raw_gyro.gyro_x = imu_data_raw.gyro_x;
            imu_data.raw_data.raw_gyro.gyro_y = imu_data_raw.gyro_y;
            imu_data.raw_data.raw_gyro.gyro_z = imu_data_raw.gyro_z;
            imu_data.raw_data.raw_mag.mag_x = mag_data.mag_x;
            imu_data.raw_data.raw_mag.mag_y = mag_data.mag_y;
            imu_data.raw_data.raw_mag.mag_z = mag_data.mag_z;

            imu_data.angle.roll = euler.angle.roll;
            imu_data.angle.pitch = euler.angle.pitch;
            imu_data.angle.yaw = euler.angle.yaw;
            imu_data.earth_acc.x = earth.axis.x;
            imu_data.earth_acc.y = earth.axis.y;
            imu_data.earth_acc.z = earth.axis.z;

            imu_data.temp.imu_temp = imu_data_raw.temp;
            imu_data.temp.mag_temp = mag_data.temperature;
            imu_data.temp.bsp_temp = 0;
        }
    }
}

