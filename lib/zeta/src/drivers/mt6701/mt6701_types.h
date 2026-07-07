// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file mt6701_types.h
 * @brief MT6701 类型与寄存器定义
 */

#pragma once

#include <Arduino.h>

#include "regmap/reg_field.h"

namespace zeta::drivers::MT6701 {
/**
 * @brief MT6701工作模式枚举
 *
 * 定义MT6701传感器支持的不同工作模式，用于配置传感器的输出方式。
 */
enum class Mode {
  kABZ = 0x0,  // ABZ模式，提供增量式编码器输出
  kUVW = 0x1,  // UVW模式，适用于无刷电机控制
};

/**
 * @brief 迟滞设置枚举
 *
 * 定义MT6701传感器的迟滞设置选项，用于抑制角度输出的抖动。
 * 较大的迟滞值可以减少噪声，但会降低灵敏度。
 */
enum class Hyst : uint8_t {
  kHyst1    = 0x0,  // 1 LSB迟滞
  kHyst2    = 0x1,  // 2 LSB迟滞
  kHyst4    = 0x2,  // 4 LSB迟滞
  kHyst8    = 0x3,  // 8 LSB迟滞
  kHyst0_25 = 0x5,  // 0.25 LSB迟滞，最小设置
  kHyst0_5  = 0x6,  // 0.5 LSB迟滞
};

/**
 * @brief 旋转方向枚举
 *
 * 定义角度计数的方向设置，用于配置传感器的角度增长方向。
 */
enum class Direction : uint8_t {
  kCCW = 0x0,  // 逆时针方向，角度随逆时针旋转增加
  kCW  = 0x1,  // 顺时针方向，角度随顺时针旋转增加
};

/**
 * @brief PWM频率设置枚举
 *
 * 定义MT6701传感器PWM输出模式下的频率选项。
 */
enum class PwmFreq : uint8_t {
  kPWMFreq994_4 = 0x0,  // 994.4 Hz PWM频率
  kPWMFreq497_2 = 0x1,  // 497.2 Hz PWM频率
};

/**
 * @brief PWM极性设置枚举
 *
 * 定义MT6701传感器PWM输出的极性选项。
 */
enum class PwmPol : uint8_t {
  kHigh = 0x0,  // 高电平有效，占空比随角度增加而增加
  kLow  = 0x1,  // 低电平有效，占空比随角度增加而减少
};

/**
 * @brief 输出模式枚举
 *
 * 定义MT6701传感器的输出信号类型。
 */
enum class OutMode : uint8_t {
  kAnalog = 0x0,  // 模拟输出模式，输出电压与角度成正比
  kPWM    = 0x1,  // PWM输出模式，占空比与角度成正比
};

/**
 * @brief 脉冲宽度设置枚举
 *
 * 定义MT6701传感器在ABZ模式下Z信号的脉冲宽度选项。
 */
enum class PulseWidth : uint8_t {
  k1LSB  = 0x0,  // 1 LSB宽度，最窄脉冲
  k2LSB  = 0x1,  // 2 LSB宽度
  k4LSB  = 0x2,  // 4 LSB宽度
  k8LSB  = 0x3,  // 8 LSB宽度
  k12LSB = 0x4,  // 12 LSB宽度
  k16LSB = 0x5,  // 16 LSB宽度
  k180   = 0x6,  // 180度宽度，半圈脉冲
};

/**
 * @brief 磁场状态枚举
 *
 * 定义MT6701传感器检测到的磁场状态，用于诊断磁铁位置问题。
 */
enum class Status : uint8_t {
  kNormal      = 0x0,  // 正常状态，磁场强度适中
  kFieldStrong = 0x1,  // 磁场过强，磁铁可能过于靠近传感器
  kFieldWeak   = 0x2,  // 磁场过弱，磁铁可能距离传感器过远
  kFieldError  = 0x3,  // 磁场错误，无法检测到有效磁场
};

/// @brief SSI时钟频率，定义SPI通信的时钟速率（1MHz）
constexpr uint32_t kSSIClock = 1000000;
/// @brief I2C默认地址，MT6701传感器的7位I2C地址
constexpr uint8_t kI2CAddress = 0x06;

/**
 * @brief MT6701寄存器定义
 *
 * 定义MT6701传感器的寄存器地址和位域，用于通过I2C接口配置传感器。
 * 使用RegisterUtils工具类简化寄存器操作。
 */
namespace MT6701Regs {
template <uint8_t Address, uint8_t Shift, uint8_t Bits>
using FieldU08 = regmap::Field<uint8_t, Address, Shift, Bits>;

template <uint8_t Address, uint8_t Shift, uint8_t Bits>
using FieldB08 = regmap::Field<bool, Address, Shift, Bits>;

// 角度相关寄存器，用于读取当前角度值
struct kANGLE_6 : FieldU08<0x03, 0, 8> {};
struct kANGLE_0 : FieldU08<0x04, 2, 6> {};

// 模式选择寄存器，用于配置传感器工作模式
struct kUVM_MUX : FieldB08<0x25, 7, 1> {};
struct kABZ_MUX : FieldU08<0x29, 6, 1> {};
struct kDIR : FieldU08<0x29, 1, 1> {};

// 分辨率设置寄存器，用于配置UVW和ABZ模式的分辨率
struct kUVM_RES_0 : FieldU08<0x30, 4, 4> {};
struct kABZ_RES_8 : FieldU08<0x30, 0, 2> {};
struct kABZ_RES_0 : FieldU08<0x31, 0, 8> {};

// 零位和迟滞设置，用于配置零位偏移和迟滞参数
struct kZERO_8 : FieldU08<0x32, 0, 4> {};
struct kZERO_0 : FieldU08<0x33, 0, 8> {};
struct kHYST_2 : FieldU08<0x32, 7, 1> {};
struct kHYST_0 : FieldU08<0x34, 6, 2> {};

// 脉冲宽度设置，用于配置ABZ模式下Z信号的脉冲宽度
struct kPULSE_WIDTH : FieldU08<0x32, 4, 3> {};

// PWM相关设置，用于配置PWM输出的参数
struct kPWM_FREQ : FieldU08<0x38, 7, 1> {};
struct kPWM_POL : FieldU08<0x38, 6, 1> {};
struct kOUT_MODE : FieldU08<0x38, 5, 1> {};

// 模拟输出范围设置，用于配置模拟输出模式的起始和结束角度
struct kA_STOP_8 : FieldU08<0x3E, 4, 4> {};
struct kA_START_8 : FieldU08<0x3E, 0, 4> {};
struct kA_START_0 : FieldU08<0x3F, 0, 8> {};
struct kA_STOP_0 : FieldU08<0x40, 0, 8> {};
};  // namespace MT6701Regs

enum class BusType : uint8_t {
  kI2C,  // I2C总线通信
  kSPI,  // SPI总线通信
};
};  // namespace zeta::drivers::MT6701