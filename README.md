# EDCstm32 电赛项目

本仓库是基于 STM32F103C8T6 的电子设计竞赛（电赛）项目代码，用于整车控制、
传感器采集、电机驱动、视觉模块通信和人机交互等功能的开发与联调。

> [!IMPORTANT]
> **修改任何 GPIO、定时器、串口、I2C 或外设驱动前，必须先完整阅读
> [`docs/broad.md`](docs/broad.md)。**
>
> 该文件记录了已经确认的底板实际引脚映射，是本项目的硬件连接依据。不要根据
> STM32 默认复用功能、排针位置或旧原理图自行推断引脚，也不要只修改驱动层而
> 忽略 `.ioc`、GPIO、AFIO 和定时器底层配置。

## 硬件平台

- 主控：STM32F103C8T6
- 电机驱动：TB6612，两路 PWM 和两路正交编码器
- 姿态与磁传感器：MPU6050 及共享 I2C 总线设备
- 视觉模块：K230，使用 USART1 通信
- 测距：HC-SR04
- 显示：OLED 软件 I2C
- 其他输入输出：七路循迹、两个按键和蜂鸣器

完整引脚和外设资源分配见 [`docs/broad.md`](docs/broad.md)。当前必须遵守的约束：

- `PA2`、`PA3` 不配置、不使用。
- 电机 A：`AIN1=PB12`、`AIN2=PB13`。
- 修改引脚映射时必须同时检查定时器通道和 AFIO 重映射。
- PA15、PB3、PB4 用作普通外设引脚，调试接口保留 SWD、关闭 JTAG。
- STM32 GPIO 为 3.3 V 逻辑，连接 5 V 供电模块前必须确认输入输出电平兼容。

## 当前外设资源

| 功能 | 底层资源 |
| :--- | :--- |
| 电机 PWM | TIM1_CH1 / PA8，TIM1_CH4 / PA11，20 kHz |
| 电机 A 编码器 | TIM4_CH1 / PB6，TIM4_CH2 / PB7 |
| 电机 B 编码器 | TIM2 全重映射，CH1 / PA15，CH2 / PB3 |
| 超声波回波 | PB10 双边沿 EXTI，TIM3 提供 1 MHz 时间基准 |
| MPU6050 / I2C 设备 | I2C1 重映射至 PB8/PB9，400 kHz |
| K230 | USART1 / PA9、PA10，115200-8-N-1 |
| OLED | PC14 / PC15 软件 I2C |

## 目录结构

```text
Core/Inc/                         应用与外设驱动头文件
Core/Src/                         主程序、驱动及中断/MSP 实现
Drivers/                          STM32 HAL 与 CMSIS
cmake/                            STM32 CMake 和工具链配置
docs/                             引脚映射、器件说明与第三方许可
EDCstm32.ioc                      STM32CubeMX 工程配置
CMakeLists.txt                    CMake 工程入口
CMakePresets.json                 Debug/Release 构建预设
```

## 构建

需要安装 CMake、Ninja 和 Arm GNU Toolchain（`arm-none-eabi-gcc`）。在仓库根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Release 构建：

```powershell
cmake --preset Release
cmake --build --preset Release
```

默认输出位于 `build/Debug/` 或 `build/Release/`，主要固件文件为
`EDCstm32.elf`。

## 修改流程

1. 阅读 [`docs/broad.md`](docs/broad.md)，确认信号、MCU 引脚和定时器通道。
2. 修改 `EDCstm32.ioc`，同步外设模式、GPIO 属性、NVIC 和 AFIO 配置。
3. 同步检查 `Core/Inc/main.h`、`Core/Src/main.c`、
   `Core/Src/stm32f1xx_hal_msp.c` 和 `Core/Src/stm32f1xx_it.c`。
4. 驱动中使用 `main.h` 的板级宏或显式传入端口/通道，避免重新硬编码引脚。
5. 更新相关文档并完成 Debug 构建。
6. 上板后依次检查默认电平、PWM、编码器方向、通信波形和传感器电平。

## 相关文档

- [`docs/broad.md`](docs/broad.md)：底板实际引脚映射，修改前必读
- [`docs/TB6612.md`](docs/TB6612.md)：电机驱动与闭环控制说明
- [`docs/JGA25_370.md`](docs/JGA25_370.md)：编码器与 PID 说明
- [`docs/MPU6050.md`](docs/MPU6050.md)：MPU6050 驱动说明

部分外设驱动来源于第三方项目，修改或分发前请同时检查 `docs/` 中对应的来源与
许可证文件。
