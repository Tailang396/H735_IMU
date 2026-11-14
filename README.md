# H735_IMU

基于 STM32H735 的自设计惯性测量与机器人控制板固件。板载 IMU、磁力计、温度传感器和外部 Flash，提供 CAN、RS-485 等通信接口，面向后续机器人姿态感知、电机控制及设备通信应用。

工程使用 STM32CubeMX、STM32 HAL 和 C/C++ 开发，通过 CMake 构建。姿态解算使用开源的 [Fusion](https://github.com/xioTechnologies/Fusion) 库，目前传感器驱动和解算框架正在逐步联调。

## 硬件组成

| 模块 | 器件或接口 | 当前软件连接 |
| --- | --- | --- |
| 主控 | STM32H735RGV6，Cortex-M7 | 系统时钟配置为 480 MHz |
| IMU | ICM45686，三轴加速度计和三轴陀螺仪 | SPI1 |
| 磁力计 | HSCDTD008A，三轴磁力计 | I2C1 |
| 温度传感器 | TMP117 | I2C3，用于温度监测和后续温度补偿 |
| 外部存储 | W25Q128JV NOR Flash | SPI2，使用 SFUD 和 EasyFlash |
| CAN | 板载 CAN 接口 | 已配置 FDCAN1、FDCAN2，具体引脚和物理连接以板卡资料为准 |
| RS-485 | 板载 RS-485 接口 | 工程配置了 USART2 及 DE 引脚 |
| 调试与指示 | USART1、USB CDC、RGB 指示灯 | 调试串口、USB 虚拟串口和状态指示 |

详细引脚与接口定义请参考仓库根目录的 [咪板管脚配置图与说明](<咪板_管脚配置图与说明_V1.pdf>)、`H735_IMU.ioc` 和实际硬件原理图。芯片资料位于本地 `数据手册/` 目录。

## 当前实现与开发状态

- **传感器**：已包含 ICM45686、HSCDTD008A、TMP117 驱动。ICM45686 当前配置为加速度 ±4 g、陀螺仪 ±1000 °/s，加速度计和陀螺仪输出数据率均为 200 Hz；具体以 `icm45686.c` 的配置为准。
- **磁力计**：当前配置为 Normal 状态、100 Hz、FIFO 关闭；磁力计数据是否参与 Fusion 由 `BSP/imu/bsp_imu.c` 中的 `MAG_STATE` 控制，目前为关闭。
- **温度**：主循环读取 TMP117 和 ICM45686 温度并用于调试输出。温度控制及补偿相关代码尚未完整接入，不能视为已完成的闭环温控或温漂补偿。
- **姿态解算**：已集成 Fusion 的 AHRS、陀螺仪偏置和校准模型接口，`fusion_init()` 设置 NWU 坐标约定，`fusion_update()` 包含有磁力计与无磁力计两种更新路径。当前 `app_imu_update()` 中的更新调用被注释，`imu_init_finsh_flag` 初始为 0，解算尚未形成完整运行链路。
- **校准与存储**：IMU 初始化流程尝试从 Flash 读取 `imu_offset` 陀螺仪偏置，并进行校验；校准数据产生、写入及整机效果仍需结合实际调试验证。Flash 已封装环境变量读写、删除与擦除接口。
- **通信与控制**：FDCAN 外设、USART2/RS-485 和 USB CDC 已有底层配置；CAN/RS-485 应用协议及电机控制逻辑仍待开发和联调。



## Flash 存储

板载 W25Q128JV 通过 SPI2 连接主控，Flash 软件分为三层：

- **[SFUD](https://github.com/armink/SFUD)**：负责识别和驱动 SPI NOR Flash，提供底层读、写、擦除功能。
- **[EasyFlash](https://github.com/armink/EasyFlash)**：通过 SFUD 移植接口管理 Flash 上的环境变量，提供键值数据存储；当前启用了 ENV 功能。
- **BSP Flash**：在 `BSP/flash/bsp_flash.c` 中封装初始化、键值读写和删除、擦除及状态打印，供应用层使用。

默认环境变量包含设备序列号 `SN`；IMU 初始化时还会尝试读取 `imu_offset` 陀螺仪偏置数据。存储区及功能开关可在 `BSP/flash/easyflash/inc/ef_cfg.h` 中查看，Flash 设备表配置位于 `BSP/flash/sfud/inc/sfud_cfg.h`。

## 构建

需要准备 CMake 3.22 或更高版本、Ninja 和 Arm GNU Toolchain，并确保 `arm-none-eabi-gcc`、`arm-none-eabi-g++` 等工具可在 `PATH` 中找到。也可以使用 CLion 配置上述工具链。
