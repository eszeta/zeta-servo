// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file control_table.h
 * @brief 从机控制表类型与单位转换（XL330 兼容）
 */

#pragma once

#include <Arduino.h>

#include "protocol/protocol_types.h"

namespace zeta::slave {

constexpr uint32_t kBaudRateTable[] = {9600,    57600,   115200,  1000000,
                                       2000000, 3000000, 4000000, 4500000};

namespace ControlBits {

union DriveModeBits {
  uint8_t value = 0;
  struct {
    bool motor_reverse_mode   : 1;  // 位0 0: 正转, 1: 反转
    bool encoder_reverse_mode : 1;  // 位1 0: 正转, 1: 反转
    bool reserved_bit2        : 1;  // 位2 保留
    bool reserved_bit3        : 1;  // 位3 保留
    bool reserved_bit4        : 1;  // 位4 保留
    bool reserved_bit5        : 1;  // 位5 保留
    bool reserved_bit6        : 1;  // 位6 保留
    bool reserved_bit7        : 1;  // 位7 保留
  };
};

union ShutdownBits {
  uint8_t value = 0;
  struct {
    bool input_voltage_error    : 1;  // 位0: 输入电压超出范围
    bool overheating_error      : 1;  // 位1: 温度超过上限
    bool motor_encoder_error    : 1;  // 位2: 编码器故障
    bool electrical_shock_error : 1;  // 位3: 电气冲击
    bool overload_error         : 1;  // 位4: 过载
  };
};

union HardwareErrorStatusBits {
  uint8_t value = 0;
  struct {
    bool input_voltage_error    : 1;  // 位0: 输入电压超出范围
    bool overheating_error      : 1;  // 位1: 温度超过上限
    bool motor_encoder_error    : 1;  // 位2: 编码器故障
    bool electrical_shock_error : 1;  // 位3: 电气冲击
    bool overload_error         : 1;  // 位4: 过载
    bool angle_limit_error      : 1;  // 位5: 角度超出范围
    bool range_error            : 1;  // 位6: 范围错误
  };
};

union MovingStatusBits {
  uint8_t value = 0;
  struct {
    bool    in_position      : 1;  // 位0: 已到达目标位置
    bool    profile_ongoing  : 1;  // 位1: Profile 执行中
    bool    reserved_bit2    : 1;  // 位2: 保留
    bool    following_error  : 1;  // 位3: 跟随误差
    uint8_t profile_type     : 2;  // 位4-5: Profile类型(0=Step,1=Rect,2=Tri,3=Trap)
    uint8_t reserved_bits6_7 : 2;  // 位6-7: 保留
  };
};

}  // namespace ControlBits

/** @name 单位转换辅助函数 */
namespace Converters {

// 统一使用语义化 *Converter 命名。
using VoltageCvt  = regmap::RatioConverter<float, 1, 10>;         // 0.1V / LSB
using PwmCvt      = regmap::RatioConverter<float, 113, 100000>;   // 0.00113 normalized / LSB
using VelocityCvt = regmap::RatioConverter<float, 229, 1000>;     // 0.229RPM / LSB
using MsCvt       = regmap::RatioConverter<uint16_t, 20>;         // 20ms / LSB
using UsCvt       = regmap::RatioConverter<uint16_t, 2>;          // 2us / LSB
using CurrentCvt  = regmap::RatioConverter<float, 1, 1000>;       // 0.001A / LSB
using FFGainCvt   = regmap::RatioConverter<float, 1, 4>;          // raw / 4
using PidPCvt     = regmap::RatioConverter<float, 1, 128>;        // raw / 128
using PidICvt     = regmap::RatioConverter<float, 1, 65536>;      // raw / 65536
using PidDCvt     = regmap::RatioConverter<float, 1, 16>;         // raw / 16
using AccCvt      = regmap::RatioConverter<float, 214577, 1000>;  // 214.577 rev/min² / LSB
}  // namespace Converters
/**
 * @brief 控制表
 * 仿照XL330的控制表
 */
namespace ControlTable {
//--------------------------------------------------------------------------
// 控制表项类型定义
//--------------------------------------------------------------------------

template <typename T, uint8_t Address>
using ControlTableItem = regmap::Field<T, Address, 0, sizeof(T) * 8>;

// 快捷别名
template <uint8_t Address>
using CTIU8 = ControlTableItem<uint8_t, Address>;
template <uint8_t Address>
using CTIU16 = ControlTableItem<uint16_t, Address>;
template <uint8_t Address>
using CTIU32 = ControlTableItem<uint32_t, Address>;
template <uint8_t Address>
using CTIS8 = ControlTableItem<int8_t, Address>;
template <uint8_t Address>
using CTIS16 = ControlTableItem<int16_t, Address>;
template <uint8_t Address>
using CTIS32 = ControlTableItem<int32_t, Address>;
template <uint8_t Address>
using CTIB8 = ControlTableItem<bool, Address>;

// 统一使用语义化 *Converter 命名。
using VoltageCvt  = Converters::VoltageCvt;
using PwmCvt      = Converters::PwmCvt;
using VelocityCvt = Converters::VelocityCvt;
using MsCvt       = Converters::MsCvt;
using UsCvt       = Converters::UsCvt;
using CurrentCvt  = Converters::CurrentCvt;
using FFGainCvt   = Converters::FFGainCvt;
using PidPCvt     = Converters::PidPCvt;
using PidICvt     = Converters::PidICvt;
using PidDCvt     = Converters::PidDCvt;
using AccCvt      = Converters::AccCvt;

//--------------------------------------------------------------------------
// EEPROM 区（掉电保存）
//--------------------------------------------------------------------------

/* 设备信息组 (0x00-0x0F, 16字节) */
/// @brief 固件版本 | 单位: - | 访问: R
struct kFirmwareVersion : CTIU8<0x00> {
  static constexpr uint8_t kDefault = 0;
};
/// @brief 舵机ID (0-252) | 单位: - | 访问: RW
struct kId : CTIU8<0x01> {
  static constexpr uint8_t kDefault = 1;
};
/* 0x08-0x0F: 保留，用于设备信息组扩展 */

/* 通信配置组 (0x10-0x1F, 16字节) */
/// @brief 波特率索引 (0-7) | 单位: - | 访问: RW
struct kBaudRate : CTIU8<0x10> {
  static constexpr uint8_t kDefault = 3;
};
/// @brief 返回延迟 | 单位: 2μs | 访问: RW
struct kReturnDelayTime : CTIU8<0x11> {
  static constexpr uint16_t kDefault = 500;
  using converter_t                  = UsCvt;
};
/// @brief 状态返回级别 (0-2) | 单位: - | 访问: RW
struct kStatusReturnLevel : CTIU8<0x12> {
  static constexpr uint8_t kDefault = 2;
};
/* 0x13-0x1F: 保留，用于通信配置组扩展 */

/* 运行模式组 (0x20-0x2F, 16字节) */
/// @brief 驱动模式 | 单位: - | 访问: RW
struct kDriveMode : CTIU8<0x20> {
  static constexpr uint8_t kDefault = 0;
};
/// @brief 工作模式 (0,1,3-5,16) | 单位: - | 访问: RW
struct kOperatingMode : CTIU8<0x21> {
  static constexpr uint8_t kDefault = 3;
};
/// @brief 关断条件 | 单位: - | 访问: RW
struct kShutdown : CTIU8<0x22> {
  static constexpr uint8_t kDefault = 52;
};
/* 0x23-0x2F: 保留，用于运行模式组扩展 */

/* 位置配置组 (0x30-0x3F, 16字节) */
/// @brief 归零偏移 | 单位: pulse | 访问: RW
struct kHomingOffset : CTIS32<0x30> {
  static constexpr int32_t kDefault = 0;
};
/// @brief 运动阈值 | 单位: 0.229 rev/min | 访问: RW
struct kMovingThreshold : CTIU32<0x34> {
  static constexpr float kDefault = 2.29f;
  using converter_t               = VelocityCvt;
};
/* 0x38-0x3F: 保留，用于位置配置组扩展 */

/* 保护限制组 (0x40-0x5F, 32字节) */
/// @brief 温度上限 | 单位: °C | 访问: RW
struct kTemperatureLimit : CTIU8<0x40> {
  static constexpr uint8_t kDefault = 72;
};
/// @brief 最高电压限制 | 单位: 0.1V | 访问: RW
struct kMaxVoltageLimit : CTIU16<0x41> {
  static constexpr float kDefault = 16.0f;
  using converter_t               = VoltageCvt;
};
/// @brief 最低电压限制 | 单位: 0.1V | 访问: RW
struct kMinVoltageLimit : CTIU16<0x43> {
  static constexpr float kDefault = 5.0f;
  using converter_t               = VoltageCvt;
};
/// @brief PWM上限 | 单位: normalized PWM (raw 0.113%/LSB) | 访问: RW
struct kPwmLimit : CTIU16<0x45> {
  static constexpr float kDefault = 1.0f;
  using converter_t               = PwmCvt;
};
/// @brief 电流上限 | 单位: 0.001A | 访问: RW
struct kCurrentLimit : CTIU16<0x47> {
  static constexpr float kDefault = 3.0f;
  using converter_t               = CurrentCvt;
};
/// @brief 速度上限 | 单位: 0.229 rev/min | 访问: RW
struct kVelocityLimit : CTIU32<0x49> {
  static constexpr float kDefault = 468.763f;
  using converter_t               = VelocityCvt;
};
/// @brief 位置上限 | 单位: pulse | 访问: RW
struct kMaxPositionLimit : CTIS32<0x4D> {
  static constexpr int32_t kDefault = 4095;
};
/// @brief 位置下限 | 单位: pulse | 访问: RW
struct kMinPositionLimit : CTIS32<0x51> {
  static constexpr int32_t kDefault = 0;
};
/// @brief 过载保护时间 | 单位: 20ms | 访问: RW
struct kProtectionTime : CTIU8<0x55> {
  static constexpr uint16_t kDefault = 0;
  using converter_t                  = MsCvt;
};
/* 0x56-0x5F: 保留，用于保护限制组扩展 */

/* PID 参数组 (0x60-0x6F, 16字节) */
/// @brief 速度积分增益 | 单位: - | 访问: RW
struct kVelocityIGain : CTIU16<0x60> {
  static constexpr float kDefault = 0.029296875f;
  using converter_t               = PidICvt;
};
/// @brief 速度比例增益 | 单位: - | 访问: RW
struct kVelocityPGain : CTIU16<0x62> {
  static constexpr float kDefault = 0.78125f;
  using converter_t               = PidPCvt;
};
/// @brief 位置微分增益 | 单位: - | 访问: RW
struct kPositionDGain : CTIU16<0x64> {
  static constexpr float kDefault = 250.0f;
  using converter_t               = PidDCvt;
};
/// @brief 位置积分增益 | 单位: - | 访问: RW
struct kPositionIGain : CTIU16<0x66> {
  static constexpr float kDefault = 0.0f;
  using converter_t               = PidICvt;
};
/// @brief 位置比例增益 | 单位: - | 访问: RW
struct kPositionPGain : CTIU16<0x68> {
  static constexpr float kDefault = 6.25f;
  using converter_t               = PidPCvt;
};
/// @brief 前馈二阶增益 | 单位: - | 访问: RW
struct kFeedforward2ndGain : CTIU16<0x6A> {
  static constexpr float kDefault = 0.0f;
  using converter_t               = FFGainCvt;
};
/// @brief 前馈一阶增益 | 单位: - | 访问: RW
struct kFeedforward1stGain : CTIU16<0x6C> {
  static constexpr float kDefault = 0.0f;
  using converter_t               = FFGainCvt;
};
/* 0x6E-0x6F: 保留，用于PID参数组扩展 */

/* 轮廓参数组 (0x70-0x7F, 16字节) */
/// @brief 轮廓加速度 | 单位: 214.577 rev/min² | 访问: RW
struct kProfileAcceleration : CTIU32<0x70> {
  static constexpr float kDefault = 0.0f;
  using converter_t               = AccCvt;
};
/// @brief 轮廓速度 | 单位: 0.229 rev/min | 访问: RW
struct kProfileVelocity : CTIU32<0x74> {
  static constexpr float kDefault = 0.0f;
  using converter_t               = VelocityCvt;
};
/* 0x78-0x7F: 保留，用于轮廓参数组扩展 */

//--------------------------------------------------------------------------
// RAM 区（掉电不保存）
//--------------------------------------------------------------------------
/* 控制命令组 (0x80-0x8F, 16字节) */
/// @brief 力矩使能 (0/1) | 单位: - | 访问: RW
struct kTorqueEnable : CTIU8<0x80> {
  static constexpr uint8_t kDefault = 0;
};
/// @brief LED开关 (0/1) | 单位: - | 访问: RW
struct kDxlLed : CTIU8<0x81> {
  static constexpr uint8_t kDefault = 0;
};
/// @brief 对齐到目标位置 | 单位: pulse | 访问: W
struct kAlignToPosition : CTIU16<0x82> {
  static constexpr uint16_t kDefault = 0;
};
/// @brief 硬件错误状态 | 单位: - | 访问: R
struct kHardwareErrorStatus : CTIU8<0x84> {
  static constexpr uint8_t kDefault = 0;
};
/// @brief 总线看门狗 | 单位: 20ms | 访问: RW
struct kBusWatchdog : CTIU8<0x85> {
  static constexpr uint16_t kDefault = 0;
  using converter_t                  = MsCvt;
};
/* 0x86-0x8F: 保留，用于控制命令组扩展 */

/* 目标值组 (0x90-0x9F, 16字节) */
/// @brief 目标PWM | 单位: normalized PWM (raw 0.113%/LSB) | 访问: RW
struct kGoalPwm : CTIS16<0x90> {
  static constexpr float kDefault = 0.0f;
  using converter_t               = PwmCvt;
};
/// @brief 目标电流 | 单位: 0.001A | 访问: RW
struct kGoalCurrent : CTIU16<0x92> {
  static constexpr float kDefault = 0.0f;
  using converter_t               = CurrentCvt;
};
/// @brief 目标速度 | 单位: 0.229 rev/min | 访问: RW
struct kGoalVelocity : CTIU32<0x94> {
  static constexpr float kDefault = 0.0f;
  using converter_t               = VelocityCvt;
};
/// @brief 目标位置 | 单位: pulse (0-4095=0-360°) | 访问: RW
struct kGoalPosition : CTIS32<0x98> {
  static constexpr int32_t kDefault = 0;
};
/* 0x9C-0x9F: 保留，用于目标值组扩展 */

/* 状态反馈组 (0xA0-0xBF, 32字节) */
/// @brief 实时时钟 | 单位: ms | 访问: R
struct kRealtimeTick : CTIU16<0xA0> {
  static constexpr uint16_t kDefault = 0;
};
/// @brief 运动状态 (0/1) | 单位: - | 访问: R
struct kMoving : CTIB8<0xA2> {
  static constexpr bool kDefault = false;
};
/// @brief 运动详细状态 | 单位: - | 访问: R
struct kMovingStatus : CTIU8<0xA3> {
  static constexpr uint8_t kDefault = 0;
};
/// @brief 当前PWM | 单位: normalized PWM (raw 0.113%/LSB) | 访问: R
struct kPresentPwm : CTIS16<0xA4> {
  static constexpr float kDefault = 0.0f;
  using converter_t               = PwmCvt;
};
/// @brief 当前电流 | 单位: 0.001A | 访问: R
struct kPresentCurrent : CTIU16<0xA6> {
  static constexpr float kDefault = 0.0f;
  using converter_t               = CurrentCvt;
};
/// @brief 当前速度 | 单位: 0.229 rev/min | 访问: R
struct kPresentVelocity : CTIU32<0xA8> {
  static constexpr float kDefault = 0.0f;
  using converter_t               = VelocityCvt;
};
/// @brief 当前位置 | 单位: pulse (0-4095=0-360°) | 访问: R
struct kPresentPosition : CTIS32<0xAC> {
  static constexpr int32_t kDefault = 0;
};
/// @brief 当前输入电压 | 单位: 0.1V | 访问: R
struct kPresentInputVoltage : CTIU16<0xB0> {
  static constexpr float kDefault = 0.0f;
  using converter_t               = VoltageCvt;
};
/// @brief 当前温度 | 单位: °C | 访问: R
struct kPresentTemperature : CTIU8<0xB2> {
  static constexpr uint8_t kDefault = 0;
};
/* 0xB3: gap */
/// @brief 期望速度轨迹 | 单位: 0.229 rev/min | 访问: R
struct kVelocityTrajectory : CTIS32<0xB4> {
  static constexpr float kDefault = 0.0f;
  using converter_t               = VelocityCvt;
};
/// @brief 期望位置轨迹 | 单位: pulse | 访问: R
struct kPositionTrajectory : CTIS32<0xB8> {
  static constexpr int32_t kDefault = 0;
};
/* 0xBC-0xBF: 保留，用于状态反馈组扩展 */

constexpr size_t kTotalSize = 0xC0;
};  // namespace ControlTable

namespace TableBlocks {
template <typename BeginType, typename EndType>
struct ControlTableBlock : public zeta::Noncopyable {
  static constexpr uint8_t kBegin = BeginType::kAddress;
  static constexpr uint8_t kEnd   = EndType::kAddress + EndType::kSize;

  static constexpr uint8_t size() { return kEnd - kBegin; }
  static constexpr bool    InBlock(const uint8_t address, const uint8_t access_size) {
    return address < kEnd && address + access_size > kBegin;
  }
};

struct kEeprom : ControlTableBlock<ControlTable::kFirmwareVersion, ControlTable::kProfileVelocity> {
};
struct kRam : ControlTableBlock<ControlTable::kTorqueEnable, ControlTable::kPositionTrajectory> {};
struct kAlign : ControlTableBlock<ControlTable::kAlignToPosition, ControlTable::kAlignToPosition> {
};
}  // namespace TableBlocks
}  // namespace zeta::slave