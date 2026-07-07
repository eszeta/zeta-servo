// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file slave_regmap.h
 * @brief 伺服从机寄存器映射
 *
 * 参考: DYNAMIXEL XL330 控制表
 * https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/
 */

#pragma once

#include <Arduino.h>

#include "regmap/reg_mmio.h"
#include "slave/control_table.h"
#include "slave/slave_regmap_impl.h"

namespace zeta::slave {

/**
 * @brief 伺服从机寄存器映射实现
 */
class Regmap : public regmap::Regmap<regmap::RegMmio>, public detail::EepromStore<Regmap> {
  friend class detail::EepromStore<Regmap>;

 public:
  Error Init() {
    CHECK(transport_.Init(table_, sizeof(table_)));
    CHECK(LoadEeprom());
    // 检查EEPROM是否为空
    if (IsEepromEmpty()) {
      CHECK(RecoveryEeprom());
      RestoreEeprom<ControlTable::kId>();
      return Error::kOk;
    }
    // 检查固件版本
    const auto firmware_version = ReadFirmwareVersion();
    if (firmware_version != ControlTable::kFirmwareVersion::kDefault) {
      CHECK(RecoveryEeprom());
      return Error::kOk;
    }
    return Error::kOk;
  }

  /** @name 设备信息组 (0x00-0x0F, EEPROM，只读) */
#pragma region "设备信息组"
  /**
   * @brief 获取固件版本 (R)
   * @return firmware_version 固件版本
   */
  uint8_t ReadFirmwareVersion() {
    uint8_t value;
    ReadField<ControlTable::kFirmwareVersion>(value);
    return value;
  }

  /**
   * @brief 写入固件版本 (R，仅内部同步)
   * @param[in] value 固件版本
   */
  void WriteFirmwareVersion(const uint8_t value) {
    WriteField<ControlTable::kFirmwareVersion>(value);
  }

  /**
   * @brief 获取舵机 ID (R/W)
   * @return id 舵机 ID
   *
   * 范围: 0-252
   *
   * 【功能说明】
   * - 舵机在总线上的唯一标识符
   * - 用于多舵机系统中的地址识别
   * - 同一总线上不能有重复的 ID
   */
  uint8_t ReadId() {
    uint8_t value;
    ReadField<ControlTable::kId>(value);
    return value;
  }

  /**
   * @brief 设置舵机 ID (R/W)
   * @param[in] value 舵机 ID (0-252)
   */
  void WriteId(const uint8_t value) { WriteField<ControlTable::kId>(value); }

#pragma endregion  // "设备信息组"

  /** @name 通信配置组 (0x10-0x1F, EEPROM) */
#pragma region "通信配置组"

  /**
   * @brief 获取波特率索引 (R/W)
   * @return baud_rate 波特率索引
   *
   * 范围: 0-7
   *
   * 【功能说明】
   * - 设置串口通信的波特率
   * - 通过索引值选择预定义的波特率
   * - 修改后需要重启生效
   *
   * 【波特率对照表】
   * | 索引 | 波特率 (bps) |
   * |------|--------------|
   * | 0    | 9600         |
   * | 1    | 57600        |
   * | 2    | 115200       |
   * | 3    | 1000000      |
   * | 4    | 2000000      |
   * | 5    | 3000000      |
   * | 6    | 4000000      |
   * | 7    | 4500000      |
   *
   */
  uint8_t ReadBaudRate() {
    uint8_t value;
    ReadField<ControlTable::kBaudRate>(value);
    return value;
  }

  /**
   * @brief 设置波特率索引 (R/W)
   * @param[in] value 波特率索引 (0-7)
   */
  void WriteBaudRate(const uint8_t value) { WriteField<ControlTable::kBaudRate>(value); }

  /**
   * @brief 获取返回延迟时间 (R/W)
   * @return microseconds 返回延迟时间（μs）
   *
   * 范围: 0-508 μs
   *
   * 【功能说明】
   * - 设置舵机响应指令后的延迟时间
   * - 用于多舵机串联时避免总线冲突
   * - 延迟时间 = 原始值 × 2μs
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 无延迟：0 μs（最快响应）
   * - 默认延迟：500 μs（平衡性能）
   * - 最大延迟：508 μs（最稳定）
   *
   * @note 在多舵机串联时，适当的延迟可避免总线冲突
   * @note 修改后需调用 StoreEeprom() 并重启生效
   */
  uint16_t ReadReturnDelayTime() {
    uint16_t value;
    ReadField<ControlTable::kReturnDelayTime>(value);
    return value;
  }

  /**
   * @brief 设置返回延迟时间 (R/W)
   * @param[in] value 返回延迟时间（μs）
   *
   * @note 在多舵机串联时，适当的延迟可避免总线冲突
   */
  void WriteReturnDelayTime(const uint16_t value) {
    WriteField<ControlTable::kReturnDelayTime>(value);
  }

  /**
   * @brief 获取状态返回级别 (R/W)
   * @return status_return_level 状态返回级别
   *
   * 范围: 0-2
   *
   * 【功能说明】
   * - 设置舵机返回状态包的级别
   * - 控制通信流量和响应行为
   * - 影响总线通信效率
   *
   * 【级别说明】
   * - 0: 不返回状态包（仅 PING 指令返回）
   * - 1: 仅读取指令返回状态包
   * - 2: 所有指令都返回状态包（默认）
   *
   */
  uint8_t ReadStatusReturnLevel() {
    uint8_t value;
    ReadField<ControlTable::kStatusReturnLevel>(value);
    return value;
  }

  /**
   * @brief 设置状态返回级别 (R/W)
   * @param[in] value 状态返回级别 (0-2)
   */
  void WriteStatusReturnLevel(const uint8_t value) {
    WriteField<ControlTable::kStatusReturnLevel>(value);
  }

#pragma endregion  // "通信配置组"

  /** @name 运行模式组 (0x20-0x2F, EEPROM) */
#pragma region "运行模式组"

  /**
   * @brief 获取驱动模式 (R/W)
   * @return drive_mode 驱动模式位域
   *
   * 【功能说明】
   * - 配置舵机的基本驱动行为
   * - 位域寄存器，每个 bit 控制不同功能
   * - 修改后需要重启生效
   *
   * 【位域定义】
   * | Bit | 名称                       | 说明                              |
   * |-----|----------------------------|-----------------------------------|
   * | 0   | Motor Reverse Mode         | 0: Normal, 1: Reverse             |
   * | 1   | Encoder Reverse Mode       | 0: Normal, 1: Reverse             |
   * | 2   | Reserved                   | 保留（未使用）                    |
   * | 3   | Reserved                   | 保留（未使用）                    |
   * | 4   | Reserved                   | 保留（未使用）                    |
   * | 5   | Reserved                   | 保留（未使用）                    |
   * | 6   | Reserved                   | 保留（未使用）                    |
   * | 7   | Reserved                   | 保留（未使用）                    |
   *
   * 【电机反转模式 (Bit 4)】
   * - 电机正反转模式，修正最终输出轴方向不一致问题
   *
   * 【编码器反转模式 (Bit 5)】
   * - 编码器正反转模式，修正最终输出轴方向不一致问题
   *
   */
  uint8_t ReadDriveMode() {
    uint8_t value;
    ReadField<ControlTable::kDriveMode>(value);
    return value;
  }

  /**
   * @brief 设置驱动模式 (R/W)
   * @param[in] value 驱动模式位域
   */
  void WriteDriveMode(const uint8_t value) { WriteField<ControlTable::kDriveMode>(value); }

  /**
   * @brief 获取工作模式 (R/W)
   * @return operating_mode 工作模式
   *
   * 【功能说明】
   * - 设置舵机的工作模式，决定控制目标和行为特性
   * - 不同模式使用不同的目标寄存器和控制策略
   * - 修改前必须先禁用力矩（Torque Enable = 0）
   * - 修改后需要重启生效
   *
   * 【模式对比表】
   * | 模式             | 控制目标      | 位置限位  | 多圈支持| 典型应用       |
   * |------------------|---------------|-----------|---------|----------------|
   * | 0: Current       | 电流/力矩     | ✗         | ✗       | 力控、柔顺     |
   * | 1: Velocity      | 速度          | ✗         | ✓       | 轮子、传送带   |
   * | 3: Position      | 位置(单圈)    | ✓         | ✗       | 关节、单圈定位 |
   * | 4: Extended      | 位置(多圈)    | ✗         | ✓       | 多圈、旋转计数 |
   * | 5: Current-based | 位置+电流限制 | ✓         | ✗       | 抓取、防撞     |
   * | 16: PWM          | PWM占空比     | ✗         | ✗       | 开环、调试     |
   *
   * 【详细模式说明】
   *
   * 【0: Current Control Mode（电流控制模式）】
   * - 使用 Goal Current 控制输出电流
   * - 输出力矩与电流成正比
   * - 不控制位置和速度
   * - 适用场景：力矩控制、柔顺控制
   *
   * 【1: Velocity Control Mode（速度控制模式）】
   * - 使用 Goal Velocity 控制转速
   * - 不控制位置
   * - 适用场景：轮式机器人、传送带
   *
   * 【3: Position Control Mode（位置控制模式，默认）】
   * - 使用 Goal Position 控制位置
   * - 位置范围: 0-4095 (0-360°)
   * - 受 Min/Max Position Limit 限制
   * - 适用场景：关节控制、单圈定位
   *
   * 【4: Extended Position Control Mode（扩展位置控制模式）】
   * - 支持多圈位置控制
   * - 位置范围: -1,048,575 ~ 1,048,575 (-256 ~ 256 圈)
   * - 不受 Min/Max Position Limit 限制
   * - 支持无限旋转累计
   * - 适用场景：多圈定位、旋转计数
   *
   * 【5: Current-based Position Control Mode（电流型位置控制）】
   * - 位置控制 + 电流限制
   * - Goal Current 作为最大电流限制
   * - 遇到阻力时电流不超过设定值
   * - 适用场景：柔性抓取、防撞控制
   *
   * 【16: PWM Control Mode（PWM 控制模式）】
   * - 直接控制 PWM 输出占空比
   * - 使用 Goal PWM 设置占空比
   * - 不进行位置/速度/电流控制
   * - 适用场景：开环控制、调试测试
   *
   * 【模式切换流程】
   * 1. WriteTorqueEnable(0) - 必须先关闭力矩
   * 2. WriteOperatingMode(new_mode) - 设置新模式
   * 3. 设置该模式的目标值（Goal Position/Velocity/Current/PWM）
   * 4. WriteTorqueEnable(1) - 重新使能力矩
   *
   * @warning 更改工作模式时，必须先将 Torque Enable 设置为 0
   * @note 不同模式下使用的控制寄存器不同，详见各 Goal 寄存器说明
   */
  uint8_t ReadOperatingMode() {
    uint8_t value;
    ReadField<ControlTable::kOperatingMode>(value);
    return value;
  }

  /**
   * @brief 设置工作模式 (R/W)
   * @param[in] value 工作模式
   */
  void WriteOperatingMode(const uint8_t value) { WriteField<ControlTable::kOperatingMode>(value); }

  /**
   * @brief 获取关断条件 (R/W)
   * @return shutdown 关断条件位域
   *
   * 范围: 0-255 (8位位域)
   *
   * 【功能说明】
   * - 设置舵机自动关断的条件
   * - 当对应错误发生时，舵机会自动关断力矩（Torque Enable = 0）
   * - 错误状态会记录在 Hardware Error Status
   * - 修改后需要重启生效
   *
   * 【位域定义】
   * | Bit | 错误类型               | 说明                          |
   * |-----|------------------------|-------------------------------|
   * | 0   | Input Voltage Error    | 输入电压超出范围              |
   * | 1   | Overheating Error      | 温度超过上限                  |
   * | 2   | Motor Encoder Error    | 编码器故障                    |
   * | 3   | Electrical Shock Error | 电气冲击                      |
   * | 4   | Overload Error         | 过载                          |
   *
   */
  uint8_t ReadShutdown() {
    uint8_t value;
    ReadField<ControlTable::kShutdown>(value);
    return value;
  }

  /**
   * @brief 设置关断条件 (R/W)
   * @param[in] value 关断条件位域
   */
  void WriteShutdown(const uint8_t value) { WriteField<ControlTable::kShutdown>(value); }

#pragma endregion  // "运行模式组"

  /** @name 位置配置组 (0x30-0x3F, EEPROM) */
#pragma region "位置配置组"

  /**
   * @brief 获取归零偏移 (R/W)
   * @return homing_offset 归零偏移（pulse）
   *
   * 范围: -1,048,575 ~ 1,048,575 pulse
   *
   * 【功能说明】
   * - 归零偏移用于定义舵机的零位参考点
   * - 影响所有位置指令的计算
   * - 实际目标位置 = Goal Position
   * - 修改后需要重启生效
   *
   * 【使用场景】
   * - 机械校准：补偿机械安装误差
   * - 零点调整：设置用户自定义的零位
   * - 多圈校准：Extended Position Control Mode 下的零点设置
   *
   * 【典型参数】
   * - 无偏移：0（默认）
   * - 小调整：±100 ~ ±1000 pulse（±2.2° ~ ±22°）
   * - 大调整：±10000 ~ ±50000 pulse（±220° ~ ±1100°）
   *
   */
  int32_t ReadHomingOffset() {
    int32_t value;
    ReadField<ControlTable::kHomingOffset>(value);
    return value;
  }

  /**
   * @brief 设置归零偏移 (R/W)
   * @param[in] value 归零偏移（pulse）
   */
  void WriteHomingOffset(const int32_t value) { WriteField<ControlTable::kHomingOffset>(value); }

  /**
   * @brief 获取运动阈值 (R/W)
   * @return rpm 运动阈值（RPM）
   *
   * 【功能说明】
   * - 运动阈值用于判断舵机是否在运动
   * - 影响 Moving Status 的更新
   * - 当速度 ≥ 此阈值时，Moving 置 1
   * - 当速度 < 此阈值时，Moving 清 0
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 高精度应用: 0.5-1.0 RPM（避免微小振动）
   * - 一般应用: 2-5 RPM（平衡精度和稳定性）
   * - 高速应用: 10-20 RPM（避免高速时的误判）
   *
   * @note 用于检测运动状态，避免微小振动导致的误判
   */
  float ReadMovingThreshold() {
    float value;
    ReadField<ControlTable::kMovingThreshold>(value);
    return value;
  }

  /**
   * @brief 设置运动阈值 (R/W)
   * @param[in] value 运动阈值（RPM）
   */
  void WriteMovingThreshold(const float value) {
    WriteField<ControlTable::kMovingThreshold>(value);
  }

#pragma endregion  // "位置配置组"

  /** @name 保护限制组 (0x40-0x5F, EEPROM) */
#pragma region "保护限制组"
  /**
   * @brief 获取温度上限 (R/W)
   * @return temperature_limit 温度上限（°C）
   *
   * 范围: 0-100°C
   *
   * 【功能说明】
   * - 设置舵机内部温度保护上限
   * - 当内部温度超过此值时触发保护机制
   * - Hardware Error Status 的 Overheating Error bit 置 1
   * - 如果 Shutdown 对应位启用，舵机会自动关断力矩
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 保守设置: 60-70°C（高安全裕度）
   * - 标准设置: 70-80°C（平衡性能和安全性）
   * - 极限设置: 80-90°C（最大性能，需谨慎）
   *
   */
  uint8_t ReadTemperatureLimit() {
    uint8_t value;
    ReadField<ControlTable::kTemperatureLimit>(value);
    return value;
  }

  /**
   * @brief 设置温度上限 (R/W)
   * @param[in] value 温度上限（°C）
   */
  void WriteTemperatureLimit(const uint8_t value) {
    WriteField<ControlTable::kTemperatureLimit>(value);
  }

  /**
   * @brief 获取最高电压限制 (R/W)
   * @return voltage 最高电压限制（V）
   *
   * 范围: 5.0-16.0V
   *
   * 【功能说明】
   * - 设置输入电压保护上限
   * - 当输入电压超过此值时触发保护机制
   * - Hardware Error Status 的 Input Voltage Error bit 置 1
   * - 如果 Shutdown 对应位启用，舵机会自动关断力矩
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 12V 系统: 13-14V（标准12V电源）
   * - 24V 系统: 26-28V（标准24V电源）
   * - 通用设置: 15-16V（兼容多种电源）
   *
   */
  float ReadMaxVoltageLimit() {
    float value;
    ReadField<ControlTable::kMaxVoltageLimit>(value);
    return value;
  }

  /**
   * @brief 设置最高电压限制 (R/W)
   * @param[in] value 最高电压限制（V）
   */
  void WriteMaxVoltageLimit(const float value) {
    WriteField<ControlTable::kMaxVoltageLimit>(value);
  }

  /**
   * @brief 获取最低电压限制 (R/W)
   * @return voltage 最低电压限制（V）
   *
   * 范围: 5.0-16.0V
   *
   * 【功能说明】
   * - 设置输入电压保护下限
   * - 当输入电压低于此值时触发保护机制
   * - Hardware Error Status 的 Input Voltage Error bit 置 1
   * - 如果 Shutdown 对应位启用，舵机会自动关断力矩
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 12V 系统: 10-11V（标准12V电源）
   * - 24V 系统: 20-22V（标准24V电源）
   * - 通用设置: 6-8V（兼容多种电源）
   *
   */
  float ReadMinVoltageLimit() {
    float value;
    ReadField<ControlTable::kMinVoltageLimit>(value);
    return value;
  }

  /**
   * @brief 设置最低电压限制 (R/W)
   * @param[in] value 最低电压限制（V）
   */
  void WriteMinVoltageLimit(const float value) {
    WriteField<ControlTable::kMinVoltageLimit>(value);
  }

  /**
   * @brief 获取 PWM 上限 (R/W)
   * @return PWM 上限（normalized 0.0-1.0）
   *
   * 范围: 0.0-1.0
   *
   * 【功能说明】
   * - 限制电机驱动器的 PWM 占空比
   * - 间接限制输出力矩和电流
   * - 所有控制模式都受此限制
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 保守设置: 0.5-0.7（安全运行）
   * - 标准设置: 0.8-0.9（平衡性能）
   * - 极限设置: 0.95-1.0（最大性能）
   *
   * @note Goal PWM 和 Present PWM 不会超过此值
   */
  float ReadPwmLimit() {
    float value;
    ReadField<ControlTable::kPwmLimit>(value);
    return value;
  }

  /**
   * @brief 设置 PWM 上限 (R/W)
   * @param[in] value PWM 上限（normalized 0.0-1.0）
   */
  void WritePwmLimit(const float value) { WriteField<ControlTable::kPwmLimit>(value); }

  /**
   * @brief 获取电流上限 (R/W)
   * @return current_limit 电流上限（A）
   *
   * 【功能说明】
   * - 限制电机输出电流
   * - 保护电机和驱动电路
   * - Current-based Position Control Mode 的最大电流
   * - 修改后需要重启生效
   *
   * @warning 长时间大电流运行会导致过热
   * @note Present Current 不会超过此值
   */
  float ReadCurrentLimit() {
    float value;
    ReadField<ControlTable::kCurrentLimit>(value);
    return value;
  }

  /**
   * @brief 设置电流上限 (R/W)
   * @param[in] value 电流上限（A）
   */
  void WriteCurrentLimit(const float value) { WriteField<ControlTable::kCurrentLimit>(value); }

  /**
   * @brief 获取速度上限 (R/W)
   * @return rpm 速度上限（RPM）
   *
   * 范围: 0-468.763 RPM
   *
   * 【功能说明】
   * - 限制舵机运动的最大速度
   * - 保护机械结构免受高速冲击
   * - 影响所有控制模式的速度限制
   * - 修改后需要重启生效
   *
   * 【典型参数】
   * - 高速应用：400-468 RPM（接近最大值）
   * - 一般应用：200-300 RPM（平衡速度和精度）
   * - 精密应用：50-100 RPM（低速高精度）
   * - 重载应用：100-200 RPM（保护机械结构）
   *
   * @note 实际速度还受 PWM Limit 和 Current Limit 限制
   */
  float ReadVelocityLimit() {
    float value;
    ReadField<ControlTable::kVelocityLimit>(value);
    return value;
  }

  /**
   * @brief 设置速度上限 (R/W)
   * @param[in] value 速度上限（RPM）
   */
  void WriteVelocityLimit(const float value) { WriteField<ControlTable::kVelocityLimit>(value); }

  /**
   * @brief 获取位置上限 (R/W)
   * @return max_position_limit 位置上限（pulse，有符号）
   *
   * 【功能说明】
   * - 限制 Position Control Mode 下的最大位置
   * - Extended Position Control Mode 不受此限制
   * - Goal Position 会被限制在 [Min Position Limit, Max Position Limit] 范围内
   * - 修改后需要重启生效
   *
   * @note 确保 Max Position Limit >= Min Position Limit
   */
  int32_t ReadMaxPositionLimit() {
    int32_t value;
    ReadField<ControlTable::kMaxPositionLimit>(value);
    return value;
  }

  /**
   * @brief 设置位置上限 (R/W)
   * @param[in] value 位置上限（pulse，有符号）
   *
   * @warning 如果设置不当，可能导致舵机无法移动
   * @note 修改后需调用 StoreEeprom() 并重启生效
   */
  void WriteMaxPositionLimit(const int32_t value) {
    WriteField<ControlTable::kMaxPositionLimit>(value);
  }

  /**
   * @brief 获取位置下限 (R/W)
   * @return min_position_limit 位置下限（pulse，有符号）
   *
   * 【功能说明】
   * - 限制 Position Control Mode 下的最小位置
   * - Extended Position Control Mode 不受此限制
   * - Goal Position 会被限制在 [Min Position Limit, Max Position Limit] 范围内
   * - 修改后需要重启生效
   *
   * @note 确保 Max Position Limit >= Min Position Limit
   */
  int32_t ReadMinPositionLimit() {
    int32_t value;
    ReadField<ControlTable::kMinPositionLimit>(value);
    return value;
  }

  /**
   * @brief 设置位置下限 (R/W)
   * @param[in] value 位置下限（pulse，有符号）
   */
  void WriteMinPositionLimit(const int32_t value) {
    WriteField<ControlTable::kMinPositionLimit>(value);
  }

  /**
   * @brief 获取保护时间 (R/W)
   * @return protection_time 保护时间（ms）
   */
  uint16_t ReadProtectionTime() {
    uint16_t value;
    ReadField<ControlTable::kProtectionTime>(value);
    return value;
  }

  /**
   * @brief 设置保护时间 (R/W)
   * @param[in] value 保护时间（ms）
   */
  void WriteProtectionTime(const uint16_t value) {
    WriteField<ControlTable::kProtectionTime>(value);
  }

#pragma endregion  // "保护限制组"

  /** @name PID 参数组 (0x60-0x6F, EEPROM) */
#pragma region "PID 参数组"
  /**
   * @brief 获取速度环 I 增益 (R/W)
   * @return velocity_igain I 增益
   *
   * 范围: 0.0-0.25
   *
   * 【功能说明】
   * - 速度环积分增益，用于消除速度稳态误差
   * - 提高恒速控制精度
   * - 仅在 Velocity Control Mode 和 Position Control Mode 有效
   * - 作为内环控制器的积分项
   *
   * 【控制结构】
   * XL330 采用级联控制结构：
   * Goal Position → [Position PID] → Goal Velocity → [Velocity PI] → Goal
   * Current → [Current PI] → PWM
   *
   * 【调试建议】
   * 1. 先调好 P 增益
   * 2. 如果存在稳态误差，缓慢增加 I 增益
   * 3. 观察是否出现低频振荡
   * 4. I 增益过大会导致系统不稳定
   *
   * 【典型参数】
   * - 保守设置: 0.01-0.05（稳定优先）
   * - 标准设置: 0.05-0.15（平衡性能）
   * - 高精度设置: 0.15-0.25（精度优先）
   *
   * @note 仅在 Velocity Control Mode 和 Position Control Mode 有效
   */
  float ReadVelocityIGain() {
    float value;
    ReadField<ControlTable::kVelocityIGain>(value);
    return value;
  }

  /**
   * @brief 设置速度环 I 增益 (R/W)
   * @param[in] value I 增益
   */
  void WriteVelocityIGain(const float value) { WriteField<ControlTable::kVelocityIGain>(value); }

  /**
   * @brief 获取速度环 P 增益 (R/W)
   * @return velocity_pgain P 增益
   *
   * 范围: 0.0-127.99
   *
   * 【功能说明】
   * - 速度环比例增益，决定速度响应速度
   * - 影响速度跟踪精度和稳定性
   * - 仅在 Velocity Control Mode 和 Position Control Mode 有效
   * - 作为内环控制器的主要增益
   *
   * 【调试建议】
   * 1. 从较小值开始调试
   * 2. 逐渐增加直到出现轻微振荡
   * 3. 回退到稳定值
   * 4. P 增益是系统稳定性的基础
   *
   * @note 仅在 Velocity Control Mode 和 Position Control Mode 有效
   */
  float ReadVelocityPGain() {
    float value;
    ReadField<ControlTable::kVelocityPGain>(value);
    return value;
  }

  /**
   * @brief 设置速度环 P 增益 (R/W)
   * @param[in] value P 增益
   */
  void WriteVelocityPGain(const float value) { WriteField<ControlTable::kVelocityPGain>(value); }

  /**
   * @brief 获取位置环 D 增益 (R/W)
   * @return position_dgain D 增益
   *
   * 范围: 0.0-1023.9
   *
   * 【功能说明】
   * - 位置环微分增益，用于抑制超调和振荡
   * - 提高位置控制稳定性
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * - 作为外环控制器的微分项
   *
   * 【调试建议】
   * 1. 先调好 P 和 I 增益
   * 2. 如果存在超调或振荡，增加 D 增益
   * 3. D 增益过大会导致高频噪声放大
   * 4. 通常 D 增益较小，0.1-1.0 范围
   *
   * 【典型参数】
   * - 保守设置: 50-100（稳定优先）
   * - 标准设置: 100-300（平衡性能）
   * - 高精度设置: 300-500（精度优先）
   *
   * @warning D 增益过大会放大高频噪声
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   */
  float ReadPositionDGain() {
    float value;
    ReadField<ControlTable::kPositionDGain>(value);
    return value;
  }

  /**
   * @brief 设置位置环 D 增益 (R/W)
   * @param[in] value D 增益
   */
  void WritePositionDGain(const float value) { WriteField<ControlTable::kPositionDGain>(value); }

  /**
   * @brief 获取位置环 I 增益 (R/W)
   * @return position_igain I 增益
   *
   * 范围: 0.0-0.25
   *
   * 【功能说明】
   * - 位置环积分增益，用于消除位置稳态误差
   * - 提高位置控制精度
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * - 作为外环控制器的积分项
   *
   * 【调试建议】
   * 1. 先调好 P 和 D 增益
   * 2. 如果存在稳态误差，缓慢增加 I 增益
   * 3. 观察是否出现低频振荡
   * 4. I 增益过大会导致系统不稳定
   *
   * 【典型参数】
   * - 保守设置: 0.01-0.05（稳定优先）
   * - 标准设置: 0.05-0.15（平衡性能）
   * - 高精度设置: 0.15-0.25（精度优先）
   *
   * @warning I 增益过大会导致系统不稳定
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   */
  float ReadPositionIGain() {
    float value;
    ReadField<ControlTable::kPositionIGain>(value);
    return value;
  }

  /**
   * @brief 设置位置环 I 增益 (R/W)
   * @param[in] value I 增益
   */
  void WritePositionIGain(const float value) { WriteField<ControlTable::kPositionIGain>(value); }

  /**
   * @brief 获取位置环 P 增益 (R/W)
   * @return position_pgain P 增益
   *
   * 范围: 0.0-127.99
   *
   * 【功能说明】
   * - 位置环比例增益，决定位置响应速度
   * - 影响位置跟踪精度和稳定性
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * - 作为外环控制器的主要增益
   *
   * 【调试建议】
   * 1. 从较小值开始调试
   * 2. 逐渐增加直到出现轻微振荡
   * 3. 回退到稳定值
   * 4. P 增益是系统稳定性的基础
   *
   * 【典型参数】
   * - 保守设置: 1-10（稳定优先）
   * - 标准设置: 10-50（平衡性能）
   * - 高响应设置: 50-127（响应优先）
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   */
  float ReadPositionPGain() {
    float value;
    ReadField<ControlTable::kPositionPGain>(value);
    return value;
  }

  /**
   * @brief 设置位置环 P 增益 (R/W)
   * @param[in] value P 增益
   */
  void WritePositionPGain(const float value) { WriteField<ControlTable::kPositionPGain>(value); }

  /**
   * @brief 获取前馈二阶增益 (R/W)
   * @return feedforward_2nd_gain 前馈二阶增益
   *
   * 范围: 0.0-127.99
   *
   * 【功能说明】
   * - 前馈控制器的二阶增益，用于改善动态响应
   * - 减少位置跟踪误差
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * - 作为前馈控制器的二阶项
   *
   * 【调试建议】
   * 1. 先调好 PID 参数
   * 2. 如果存在跟踪误差，增加前馈增益
   * 3. 前馈增益过大会导致系统不稳定
   * 4. 通常前馈增益较小，0.1-1.0 范围
   *
   * 【典型参数】
   * - 保守设置: 0.1-0.5（稳定优先）
   * - 标准设置: 0.5-2.0（平衡性能）
   * - 高精度设置: 2.0-5.0（精度优先）
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   */
  float ReadFeedforward2ndGain() {
    float value;
    ReadField<ControlTable::kFeedforward2ndGain>(value);
    return value;
  }

  /**
   * @brief 设置前馈二阶增益 (R/W)
   * @param[in] value 前馈二阶增益
   */
  void WriteFeedforward2ndGain(const float value) {
    WriteField<ControlTable::kFeedforward2ndGain>(value);
  }

  /**
   * @brief 获取前馈一阶增益 (R/W)
   * @return feedforward_1st_gain 前馈一阶增益
   *
   * 范围: 0.0-127.99
   *
   * 【功能说明】
   * - 前馈控制器的一阶增益，用于改善动态响应
   * - 减少位置跟踪误差
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * - 作为前馈控制器的一阶项
   *
   * 【调试建议】
   * 1. 先调好 PID 参数
   * 2. 如果存在跟踪误差，增加前馈增益
   * 3. 前馈增益过大会导致系统不稳定
   * 4. 通常前馈增益较小，0.1-1.0 范围
   *
   * 【典型参数】
   * - 保守设置: 0.1-0.5（稳定优先）
   * - 标准设置: 0.5-2.0（平衡性能）
   * - 高精度设置: 2.0-5.0（精度优先）
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   */
  float ReadFeedforward1stGain() {
    float value;
    ReadField<ControlTable::kFeedforward1stGain>(value);
    return value;
  }

  /**
   * @brief 设置前馈一阶增益 (R/W)
   * @param[in] value 前馈一阶增益
   */
  void WriteFeedforward1stGain(const float value) {
    WriteField<ControlTable::kFeedforward1stGain>(value);
  }

#pragma endregion  // "PID 参数组"

  /** @name 轮廓参数组 (0x70-0x7F, EEPROM) */
#pragma region "轮廓参数组"

  /**
   * @brief 获取轮廓加速度 (R/W)
   * @return rpm2 轮廓加速度（rev/min²）
   *
   * 【功能说明】
   * - 设置位置控制轨迹生成器的加速度限制
   * - 对应 Dynamixel Profile Acceleration (108)，单位 214.577 rev/min² / LSB
   * - 0: 不限制加速度（阶跃）
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   *
   * @note 作为 EEPROM 参数与 PID 增益同类，掉电保存
   */
  float ReadProfileAcceleration() {
    float value;
    ReadField<ControlTable::kProfileAcceleration>(value);
    return value;
  }

  /**
   * @brief 设置轮廓加速度 (R/W)
   * @param[in] value 轮廓加速度（rev/min/s）
   */
  void WriteProfileAcceleration(const float value) {
    WriteField<ControlTable::kProfileAcceleration>(value);
  }

  /**
   * @brief 获取轮廓速度 (R/W)
   * @return rpm 轮廓速度（rev/min）
   *
   * 【功能说明】
   * - 设置位置控制轨迹生成器的最大速度限制
   * - 对应 Dynamixel Profile Velocity (112)
   * - 0: 不限制速度（阶跃）
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   *
   * @note 作为 EEPROM 参数与 PID 增益同类，掉电保存
   */
  float ReadProfileVelocity() {
    float value;
    ReadField<ControlTable::kProfileVelocity>(value);
    return value;
  }

  /**
   * @brief 设置轮廓速度 (R/W)
   * @param[in] value 轮廓速度（rev/min）
   */
  void WriteProfileVelocity(const float value) {
    WriteField<ControlTable::kProfileVelocity>(value);
  }

#pragma endregion  // "轮廓参数组"

  /** @name 控制命令组 (0x80-0x8F, RAM) */
#pragma region "控制命令组"
  /**
   * @brief 获取力矩使能状态 (R/W)
   * @return torque_enable 力矩使能状态
   *
   * 范围: 0-1
   *
   * 【功能说明】
   * - 控制舵机力矩输出的使能状态
   * - 0: 禁用力矩，电机可以自由转动（关断电机驱动器）
   * - 1: 使能力矩，电机根据控制指令运动
   * - 修改 EEPROM 参数（如 Operating Mode）前必须设置为 0
   * - 上电后默认为 0，需要手动使能
   * - 发生 Shutdown 错误时会自动变为 0
   *
   * 【操作流程】
   * 1. 使能力矩：WriteTorqueEnable(1) - 电机会立即执行控制指令
   * 2. 禁用力矩：WriteTorqueEnable(0) - 电机可以自由转动
   * 3. 模式切换：必须先禁用，再切换模式，最后重新使能
   *
   * @warning 使能力矩后电机会立即执行控制指令，注意安全
   * @note 禁用力矩时，Present Position 等反馈值仍然有效
   */
  uint8_t ReadTorqueEnable() {
    uint8_t value;
    ReadField<ControlTable::kTorqueEnable>(value);
    return value;
  }

  /**
   * @brief 设置力矩使能 (R/W)
   * @param[in] value 力矩使能 (0: 禁用, 1: 使能)
   */
  void WriteTorqueEnable(const uint8_t value) { WriteField<ControlTable::kTorqueEnable>(value); }

  /**
   * @brief 获取 LED 状态 (R/W)
   * @return dxl_led LED 状态
   *
   * 范围: 0-1
   *
   * 【功能说明】
   * - 控制舵机上的 LED 指示灯
   * - 0: LED 关闭
   * - 1: LED 开启
   * - 用于状态指示和调试
   */
  uint8_t ReadDxlLed() {
    uint8_t value;
    ReadField<ControlTable::kDxlLed>(value);
    return value;
  }

  /**
   * @brief 设置 LED 开关 (R/W)
   * @param[in] value LED 状态 (0: 关, 1: 开)
   */
  void WriteDxlLed(const uint8_t value) { WriteField<ControlTable::kDxlLed>(value); }

  /**
   * @brief 获取对齐命令值 (W)
   * @return align_to_position 对齐到目标位置
   */
  uint16_t ReadAlignToPosition() {
    uint16_t value;
    ReadField<ControlTable::kAlignToPosition>(value);
    return value;
  }

  /**
   * @brief 设置对齐到目标位置 (W)
   * @param[in] value 对齐到目标位置
   * 范围: 0-1
   * 【功能说明】
   * - 调整 Homing Offset，使 Present Position 对齐目标位置
   */
  void WriteAlignToPosition(const uint16_t value) {
    WriteField<ControlTable::kAlignToPosition>(value);
  }

  /**
   * @brief 获取硬件错误状态 (R)
   * @return hardware_error_status 硬件错误状态位域
   *
   * 范围: 0-255 (8位位域)
   *
   * 【功能说明】
   * - 读取舵机硬件错误状态，提供详细的故障诊断信息
   * - 错误状态会持续存在，直到手动清除
   * - 位域寄存器，每个 bit 表示一种错误类型
   *
   * 【硬件错误位域】
   * | Bit | 错误类型               | 说明                          |
   * |-----|------------------------|-------------------------------|
   * | 0   | Input Voltage Error    | 输入电压超出范围              |
   * | 1   | Overheating Error      | 温度超过上限                  |
   * | 2   | Motor Encoder Error    | 编码器故障                    |
   * | 3   | Electrical Shock Error | 电气冲击                      |
   * | 4   | Overload Error         | 过载                          |
   * | 5   | Angle Limit Error      | 角度超出范围                  |
   * | 6   | Range Error            | 范围错误                      |
   *
   * @note 如果 Shutdown 对应位启用，发生错误会自动关断力矩
   */
  uint8_t ReadHardwareErrorStatus() {
    uint8_t value;
    ReadField<ControlTable::kHardwareErrorStatus>(value);
    return value;
  }

  /**
   * @brief 写入硬件错误状态 (R，仅内部同步)
   * @param[in] value 硬件错误状态位域
   */
  void WriteHardwareErrorStatus(const uint8_t value) {
    WriteField<ControlTable::kHardwareErrorStatus>(value);
  }

  /**
   * @brief 获取总线看门狗 (R/W)
   * @return milliseconds 看门狗时间（ms）
   *
   * 范围: 0-5080 ms
   *
   * 【功能说明】
   * - 总线看门狗超时时间，用于检测通信故障
   * - 当超过此时间未收到指令时，舵机会自动关断力矩
   * - 0: 禁用看门狗功能
   * - 非0: 启用看门狗功能
   *
   * 【典型参数】
   * - 禁用看门狗: 0（调试时使用）
   * - 快速检测: 100-500 ms（高响应要求）
   * - 标准检测: 1000-2000 ms（一般应用）
   * - 慢速检测: 3000-5080 ms（低频率通信）
   *
   * @note 看门狗超时会自动关断力矩，需要重新使能
   */
  uint16_t ReadBusWatchdog() {
    uint16_t value;
    ReadField<ControlTable::kBusWatchdog>(value);
    return value;
  }

  /**
   * @brief 设置总线看门狗 (R/W)
   * @param[in] value 看门狗时间（ms）
   */
  void WriteBusWatchdog(const uint16_t value) { WriteField<ControlTable::kBusWatchdog>(value); }

#pragma endregion  // "控制命令组"

  /** @name 目标值组 (0x90-0x9F, RAM) */
#pragma region "目标值组"

  /**
   * @brief 获取目标 PWM (R/W)
   * @return 目标 PWM（normalized -1.0-1.0）
   *
   * 范围: -1.0 ~ 1.0
   *
   * 【功能说明】
   * - 设置电机驱动器的 PWM 占空比
   * - 仅在 PWM Control Mode (Operating Mode = 16) 有效
   * - 负值表示反向旋转
   * - 受 PWM Limit 限制
   *
   * 【典型参数】
   * - 停止: 0.0（无输出）
   * - 低速: ±0.1-0.3（低速运行）
   * - 中速: ±0.3-0.7（中速运行）
   * - 高速: ±0.7-1.0（高速运行）
   *
   * @note 仅在 PWM Control Mode 有效
   */
  float ReadGoalPwm() {
    float value;
    ReadField<ControlTable::kGoalPwm>(value);
    return value;
  }

  /**
   * @brief 设置目标 PWM (R/W)
   * @param[in] value 目标 PWM（normalized -1.0-1.0）
   */
  void WriteGoalPwm(const float value) { WriteField<ControlTable::kGoalPwm>(value); }

  /**
   * @brief 获取目标电流 (R/W)
   * @return goal_current 目标电流（A）
   *
   * 范围: -65.535 ~ 65.535 A
   *
   * 【功能说明】
   * - 设置电机输出电流目标值
   * - 在 Current Control Mode (Operating Mode = 0) 中作为控制目标
   * - 在 Current-based Position Control Mode (Operating Mode = 5)
   * 中作为最大电流限制
   * - 负值表示反向电流
   * - 受 Current Limit 限制
   *
   * @note 在 Current Control Mode 中作为控制目标，在 Current-based Position
   * Control Mode 中作为限制
   */
  float ReadGoalCurrent() {
    float value;
    ReadField<ControlTable::kGoalCurrent>(value);
    return value;
  }

  /**
   * @brief 设置目标电流 (R/W)
   * @param[in] value 目标电流（0.001A）
   */
  void WriteGoalCurrent(const float value) { WriteField<ControlTable::kGoalCurrent>(value); }

  /**
   * @brief 获取目标速度 (R/W)
   * @return rpm 目标速度（RPM）
   *
   * 范围: -468.763 到 468.763 RPM
   *
   * 【功能说明】
   * - 设置电机转速目标值
   * - 在 Velocity Control Mode (Operating Mode = 1) 中作为控制目标
   * - 在 Position Control Mode 和 Extended Position Control Mode
   * 中由位置控制器自动计算
   * - 负值表示反向旋转
   * - 受 Velocity Limit 限制
   *
   * 【典型参数】
   * - 停止: 0 RPM（无旋转）
   * - 低速: ±10-50 RPM（低速运行）
   * - 中速: ±50-200 RPM（中速运行）
   * - 高速: ±200-468 RPM（高速运行）
   *
   * @note 仅在 Velocity Control Mode 中作为控制目标
   */
  float ReadGoalVelocity() {
    float value;
    ReadField<ControlTable::kGoalVelocity>(value);
    return value;
  }

  /**
   * @brief 设置目标速度 (R/W)
   * @param[in] value 目标速度（RPM）
   */
  void WriteGoalVelocity(const float value) { WriteField<ControlTable::kGoalVelocity>(value); }

  /**
   * @brief 获取目标位置 (RW)
   * @return goal_position 目标位置（pulse）
   *
   * 范围: 0-4095 (Position Mode) | -1,048,575 到 1,048,575 (Extended Mode)
   *
   * 【功能说明】
   * 设置舵机的目标位置，实际行为取决于当前 Operating Mode：
   * - Position Control Mode: 单圈位置控制，范围 0-4095 (0-360°)
   * - Extended Position Control Mode: 多圈位置控制，支持无限旋转
   *
   * 【典型参数】
   * - Position Mode: 0-4095 (0-360°)
   * - Extended Mode: ±1000000 (约±244圈)
   * - 连续旋转: 设置超出物理范围的值
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * @note 运动中更新 Goal Position 会平滑过渡到新轨迹
   */
  int32_t ReadGoalPosition() {
    int32_t value;
    ReadField<ControlTable::kGoalPosition>(value);
    return value;
  }

  /**
   * @brief 设置目标位置 (RW)
   * @param[in] value 目标位置（pulse）
   */
  void WriteGoalPosition(const int32_t value) { WriteField<ControlTable::kGoalPosition>(value); }

#pragma endregion  // "目标值组"

  /** @name 状态反馈组 (0xA0-0xBF, RAM，只读) */
#pragma region "状态反馈组"
  /**
   * @brief 获取实时时钟 (R)
   * @return realtime_tick 实时时钟（ms）
   *
   * 范围: 0-32767 ms
   *
   * 【功能说明】
   * - 舵机内部实时时钟计数器
   * - 以毫秒为单位递增
   * - 用于时间戳和定时功能
   * - 上电后从 0 开始计数
   * - 达到最大值后重新从 0 开始
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  uint16_t ReadRealtimeTick() {
    uint16_t value;
    ReadField<ControlTable::kRealtimeTick>(value);
    return value;
  }

  /**
   * @brief 写入实时时钟 (R，仅内部同步)
   * @param[in] value 实时时钟
   */
  void WriteRealtimeTick(const uint16_t value) { WriteField<ControlTable::kRealtimeTick>(value); }

  /**
   * @brief 获取运动状态 (R)
   * @return moving 运动状态
   *
   * 范围: 0-1
   *
   * 【功能说明】
   * - 简化的运动状态指示
   * - 0: 舵机静止
   * - 1: 舵机正在运动
   * - 基于 Moving Threshold 判断
   * - 实时更新，反映当前运动状态
   *
   * 【判断逻辑】
   * - 当 Present Velocity ≥ Moving Threshold 时，Moving = 1
   * - 当 Present Velocity < Moving Threshold 时，Moving = 0
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  bool ReadMoving() {
    bool value;
    ReadField<ControlTable::kMoving>(value);
    return value;
  }

  /**
   * @brief 写入运动状态 (R，仅内部同步)
   * @param[in] value 运动状态
   */
  void WriteMoving(const bool value) { WriteField<ControlTable::kMoving>(value); }

  /**
   * @brief 获取详细运动状态 (R)
   * @return moving_status 详细运动状态位域
   *
   * 范围: 0-255 (8位位域)
   *
   * 【功能说明】
   * - 提供详细的运动状态信息
   * - 位域寄存器，每个 bit 表示不同的状态
   * - 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   * - 实时更新，反映当前运动状态
   *
   * 【位域定义】
   * | Bit | 名称                   | 说明                          |
   * |-----|------------------------|-------------------------------|
   * | 0   | In-Position            | 0: 未到达目标位置, 1: 已到达  |
   * | 1   | Reserved               | 保留位                        |
   * | 2   | Reserved               | 保留位                        |
   * | 3   | Following Error        | 0: 正在跟随, 1: 未跟随轨迹    |
   * | 4-5 | Reserved               | 保留位                        |
   * | 6-7 | Reserved               | 保留位                        |
   *
   * 【状态说明】
   * - In-Position: 当前位置是否在目标位置容差范围内
   * - Following Error: 是否出现跟随误差
   *
   * @note 仅在 Position Control Mode 和 Extended Position Control Mode 有效
   */
  uint8_t ReadMovingStatus() {
    uint8_t value;
    ReadField<ControlTable::kMovingStatus>(value);
    return value;
  }

  /**
   * @brief 写入详细运动状态 (R，仅内部同步)
   * @param[in] value 详细运动状态位域
   */
  void WriteMovingStatus(const uint8_t value) { WriteField<ControlTable::kMovingStatus>(value); }

  /**
   * @brief 获取当前 PWM (R)
   * @return percent 当前 PWM（%）
   *
   * 范围: -100.0% 到 100.0%
   *
   * 【功能说明】
   * - 显示实际输出的 PWM 占空比
   * - 正值表示正向旋转
   * - 负值表示反向旋转
   * - 实时更新，反映当前输出状态
   *
   */
  float ReadPresentPwm() {
    float value;
    ReadField<ControlTable::kPresentPwm>(value);
    return value;
  }

  /**
   * @brief 写入当前 PWM (R，仅内部同步)
   * @param[in] value 当前 PWM（normalized -1.0-1.0）
   */
  void WritePresentPwm(const float value) { WriteField<ControlTable::kPresentPwm>(value); }

  /**
   * @brief 获取当前电流 (R)
   * @return present_current 当前电流（A）
   *
   * 【功能说明】
   * - 显示当前通过电机的电流
   * - 电流与负载力矩成正比
   * - 实时更新，反映当前电流状态
   * - 用于监控电机负载和功耗
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  float ReadPresentCurrent() {
    float value;
    ReadField<ControlTable::kPresentCurrent>(value);
    return value;
  }

  /**
   * @brief 写入当前电流 (R，仅内部同步)
   * @param[in] value 当前电流
   */
  void WritePresentCurrent(const float value) { WriteField<ControlTable::kPresentCurrent>(value); }

  /**
   * @brief 获取当前速度 (R)
   * @return rpm 当前速度（RPM）
   *
   * 范围: -468.763 到 468.763 RPM
   *
   * 【功能说明】
   * - 显示当前电机转速
   * - 通过编码器微分计算得到
   * - 正值表示正向旋转
   * - 负值表示反向旋转
   * - 实时更新，反映当前速度状态
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  float ReadPresentVelocity() {
    float value;
    ReadField<ControlTable::kPresentVelocity>(value);
    return value;
  }

  /**
   * @brief 写入当前速度 (R，仅内部同步)
   * @param[in] value 当前速度
   */
  void WritePresentVelocity(const float value) {
    WriteField<ControlTable::kPresentVelocity>(value);
  }

  /**
   * @brief 获取当前位置 (R)
   * @return present_position 当前位置（pulse）
   *
   * 范围: 0-4095 (Position Mode) | -1,048,575 到 1,048,575 (Extended Mode)
   *
   * 【功能说明】
   * - 读取舵机当前的实际位置
   * - 通过磁编码器实时反馈
   * - 位置范围取决于当前 Operating Mode
   * - 实时更新，反映当前位置状态
   *
   * 【模式差异】
   * - Position Control Mode: 0-4095 (单圈)
   * - Extended Position Control Mode: 多圈累计位置
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  int32_t ReadPresentPosition() {
    int32_t value;
    ReadField<ControlTable::kPresentPosition>(value);
    return value;
  }

  /**
   * @brief 写入当前位置 (R，仅内部同步)
   * @param[in] value 当前位置
   */
  void WritePresentPosition(const int32_t value) {
    WriteField<ControlTable::kPresentPosition>(value);
  }

  /**
   * @brief 获取当前输入电压 (R)
   * @return voltage 当前输入电压（V）
   *
   * 范围: 0-16.0 V
   *
   * 【功能说明】
   * - 显示当前输入电压
   * - 用于监控电源状态
   * - 实时更新，反映当前电压
   * - 电压异常会触发保护
   *
   * @note 此寄存器为只读，由系统自动更新
   */
  float ReadPresentInputVoltage() {
    float value;
    ReadField<ControlTable::kPresentInputVoltage>(value);
    return value;
  }

  /**
   * @brief 写入当前输入电压 (R，仅内部同步)
   * @param[in] value 当前输入电压
   */
  void WritePresentInputVoltage(const float value) {
    WriteField<ControlTable::kPresentInputVoltage>(value);
  }

  /**
   * @brief 获取当前温度 (R)
   * @return present_temperature 当前温度（°C）
   *
   * 范围: 0-100 °C
   *
   * 【功能说明】
   * - 显示当前电机温度
   * - 温度传感器位于电机内部
   * - 实时更新，反映当前温度
   * - 温度过高会触发保护
   */
  uint8_t ReadPresentTemperature() {
    uint8_t value;
    ReadField<ControlTable::kPresentTemperature>(value);
    return value;
  }

  /**
   * @brief 写入当前温度 (R，仅内部同步)
   * @param[in] value 当前温度
   */
  void WritePresentTemperature(const uint8_t value) {
    WriteField<ControlTable::kPresentTemperature>(value);
  }

  /**
   * @brief 获取期望速度轨迹 (R)
   * @return rpm 期望速度轨迹（rev/min）
   *
   * 【功能说明】
   * - 位置控制模式下，轨迹生成器输出的期望速度
   * - 对应 Dynamixel Velocity Trajectory (136)
   * - 用于前馈控制和运动监控
   * - 实时更新，反映当前轨迹速度
   *
   * @note 此寄存器为只读，由控制器自动更新
   */
  float ReadVelocityTrajectory() {
    float value;
    ReadField<ControlTable::kVelocityTrajectory>(value);
    return value;
  }

  /**
   * @brief 写入期望速度轨迹 (R，仅内部同步)
   * @param[in] value 期望速度轨迹（rev/min）
   */
  void WriteVelocityTrajectory(const float value) {
    WriteField<ControlTable::kVelocityTrajectory>(value);
  }

  /**
   * @brief 获取期望位置轨迹 (R)
   * @return pulse 期望位置轨迹（pulse）
   *
   * 【功能说明】
   * - 位置控制模式下，轨迹生成器输出的期望位置
   * - 对应 Dynamixel Position Trajectory (140)
   * - 用于跟踪误差监控和调试
   * - 实时更新，反映当前轨迹位置
   *
   * @note 此寄存器为只读，由控制器自动更新
   */
  int32_t ReadPositionTrajectory() {
    int32_t value;
    ReadField<ControlTable::kPositionTrajectory>(value);
    return value;
  }

  /**
   * @brief 写入期望位置轨迹 (R，仅内部同步)
   * @param[in] value 期望位置轨迹（pulse）
   */
  void WritePositionTrajectory(const int32_t value) {
    WriteField<ControlTable::kPositionTrajectory>(value);
  }

#pragma endregion  // "状态反馈组"

  // 控制表缓存，按 ControlTable 地址空间顺序线性存储。
  uint8_t table_[ControlTable::kTotalSize] = {};
};

}  // namespace zeta::slave
