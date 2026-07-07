// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file monitor.h
 * @brief 舵机状态监控输出（Serial 打印目标/位置/速度/电流/PWM）
 */

#pragma once

#include <Arduino.h>

#include "noncopyable.h"

namespace zeta::utils {

/**
 * @brief 监控类，用于监控电机状态
 */
template <typename ServoType>
class Monitor : public zeta::Noncopyable {
 public:
  enum MonitorBitmap : uint8_t {
    kTarget   = 0b1000000,  ///< 监控目标值
    kPwm      = 0b0100000,  ///< 监控 PWM
    kCurrent  = 0b0001000,  ///< 监控电流
    kVelocity = 0b0000010,  ///< 监控速度
    kPosition = 0b0000001   ///< 监控位置
  };

  static constexpr uint8_t kDecimals = 4;  ///< 监控输出小数位数

  /**
   * @brief 初始化监控
   * @param servo 伺服电机
   */
  Error Init(ServoType* servo, Print* serial) {
    servo_       = servo;
    monitorPort_ = serial;
    return Error::kOk;
  }

  /**
   * @brief 处理监控
   * @param dt 时间间隔(秒)
   */
  Error Process(float dt);

 private:
  /**
   * @brief 监控变量
   */
  uint8_t variables_ = kTarget | kPwm | kCurrent | kVelocity | kPosition;
  /**
   * @brief 伺服电机
   */
  ServoType* servo_ = nullptr;
  /**
   * @brief 监控输出
   */
  Print* monitorPort_ = nullptr;
};

}  // namespace zeta::utils

namespace zeta::utils {

template <typename ServoType>
Error Monitor<ServoType>::Process(float dt) {
  (void)dt;
  if (!monitorPort_)
    return Error::kOk;

  if (variables_ & MonitorBitmap::kPosition) {
    monitorPort_->print(F(">position:"));
    monitorPort_->println(servo_->present_position());
  }

  if (variables_ & MonitorBitmap::kPosition) {
    monitorPort_->print(F(">raw_position:"));
    monitorPort_->println(servo_->encoder()->raw_pos());
  }

  return Error::kOk;
}

}  // namespace zeta::utils
