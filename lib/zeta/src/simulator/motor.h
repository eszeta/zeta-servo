// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file motor.h
 * @brief 仿真电机（PWM 由 plant 读取，供算法验证）
 */

#pragma once

#include "servo/motor.h"

namespace zeta::simulator {

/**
 * @brief 仿真用电机驱动器
 *
 * 不驱动真实硬件，仅保存 PWM 供仿真 plant 读取，用于算法验证。
 */
class SimulatorMotor final : public servo::Motor<SimulatorMotor> {
 public:
  /**
   * @brief 获取上一拍输出的 PWM（-1..1）
   */
  float last_pwm() const { return pwm_; }

  void SetPWMImpl(float pwm);
  void BrakeImpl();
  void CoastImpl();

 private:
  float pwm_ = 0.0f;
};

}  // namespace zeta::simulator

namespace zeta::simulator {

inline void SimulatorMotor::SetPWMImpl(float pwm) {
  pwm_ = pwm;
}

inline void SimulatorMotor::BrakeImpl() {
  pwm_ = 0.0f;
}

inline void SimulatorMotor::CoastImpl() {
  pwm_ = 0.0f;
}

}  // namespace zeta::simulator
