// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file servo_types.h
 * @brief 舵机相关类型（工作模式、方向、位域状态）
 */

#pragma once

#include <Arduino.h>

namespace zeta::servo {

/**
 * @brief 伺服电机工作模式枚举
 *
 * 定义了伺服电机支持的各种控制模式，每种模式对应不同的控制策略和参数。
 * 更改工作模式前必须先禁用扭矩输出（Torque Enable = 0）。
 */
enum class OperatingMode : uint8_t {
  /**
   * @brief 电流控制模式
   *
   * 直接控制电机输出电流，实现力矩控制。
   * - 控制参数：Goal Current（目标电流）
   * - 输出特性：力矩与电流成正比
   * - 应用场景：力矩控制、柔顺控制、力反馈
   */
  kCurrent = 0,

  /**
   * @brief 速度控制模式
   *
   * 控制电机转速，实现速度伺服。
   * - 控制参数：Goal Velocity（目标速度）
   * - 输出特性：恒定转速输出
   * - 应用场景：轮式机器人、传送带、风扇控制
   */
  kVelocity = 1,

  /**
   * @brief 位置控制模式（单圈）
   *
   * 控制电机位置，实现位置伺服，限制在单圈范围内。
   * - 控制参数：Goal Position（目标位置）
   * - 位置范围：0-4095（对应0-360°）
   * - 限制条件：受 Min/Max Position Limit 限制
   * - 应用场景：关节控制、单圈定位、舵机应用
   */
  kPosition = 3,

  /**
   * @brief 扩展位置控制模式（多圈）
   *
   * 支持多圈位置控制，允许电机连续旋转。
   * - 控制参数：Goal Position（目标位置）
   * - 位置范围：-1,048,575 ~ 1,048,575（对应-256 ~ 256圈）
   * - 限制条件：不受 Min/Max Position Limit 限制
   * - 应用场景：多圈定位、旋转计数、连续旋转应用
   */
  kExtendedPosition = 4,

  /**
   * @brief 电流限制位置控制模式
   *
   * 位置控制与电流限制相结合，提供安全的位置控制。
   * - 控制参数：Goal Position（目标位置）、Goal Current（最大电流限制）
   * - 输出特性：位置控制 + 电流保护
   * - 安全特性：遇到阻力时电流不超过设定值
   * - 应用场景：柔性抓取、防撞控制、安全定位
   */
  kCurrentPosition = 5,

  /**
   * @brief PWM 开环控制模式
   *
   * 直接控制 PWM 占空比，不进行闭环控制。
   * - 控制参数：Goal PWM（PWM 占空比）
   * - 输出特性：开环控制，无反馈
   * - 应用场景：开环控制、调试测试、简单速度控制
   */
  kPwm = 16
};

/**
 * @brief 方向
 */
enum class Reverse : int8_t {
  kNormal  = 1,   ///< 顺时针
  kReverse = -1,  ///< 逆时针
};
}  // namespace zeta::servo