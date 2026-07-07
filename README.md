# Zeta&nbsp;ζ

> *A compact servo controller firmware for STM32 — tuned by damping, driven by code.*

**Zeta** 是一款面向 STM32 的紧凑型伺服控制器固件与配套 C++ 库，提供位置/速度/电流三环控制、磁编码器与 IMU 驱动，以及一套仿 Dynamixel 的寄存器表（Regmap）从机协议。

## 核心特性

- 多模式控制

  - 位置伺服模式：精确的位置控制
  - 恒速模式：稳定的速度控制
  - PWM 开环调速模式：简单的速度控制
  - 步进伺服模式：步进电机控制
- 丰富的保护功能

  - 过流保护
  - 过温保护
  - 过压保护
  - 堵转保护
  - 位置限位保护
- 精确的控制算法

  - PID 位置控制
  - 速度闭环控制
  - 前馈控制
  - 死区控制
  - 输出限幅
- 多种传感器支持

  - MT6701（I2C 接口）
  - MA330（SPI 接口）
  - 支持 14 位分辨率
- 多种驱动芯片支持

  - DRV8231A
  - MP6515

## 开发环境

本项目使用 [PlatformIO](https://platformio.org/) 构建系统，支持以下 MCU：

- STM32G431CB

C++ 代码统一位于 `zeta::` 命名空间下，库目录：`lib/zeta/`。

## 许可证

本项目采用 Apache License 2.0 许可证 - 详情请参阅 [LICENSE](LICENSE) 文件。

## 致谢

感谢以下开源项目对本项目的启发：

- [Arduino-FOC](https://github.com/simplefoc/Arduino-FOC)
- [Arduino_LSM6DSOX](https://github.com/arduino-libraries/Arduino_LSM6DSOX)
- [SMSMod](https://github.com/pat92fr/SMSMod)
- [STS_servos](https://github.com/matthieuvigne/STS_servos)

## 贡献指南

欢迎提交 Issue 和 Pull Request 来帮助改进这个项目。
