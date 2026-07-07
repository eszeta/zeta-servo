// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file plant.h
 * @brief 仿真被控对象（PWM→位置/电流，回写 encoder/current）
 */

#pragma once

#include "error.h"
#include "simulator/current.h"
#include "simulator/encoder.h"
#include "simulator/motor.h"

namespace zeta::simulator {

/**
 * @brief 仿真被控对象（Planter）
 *
 * 持有 SimulatorMotor / SimulatorEncoder / SimulatorCurrent motor 的 PWM 计算位置与电流，
 * 回写到 encoder 和 current，供 servo.Process 读取。
 */
class SimulatorPlant {
 public:
  void set_motor(SimulatorMotor* motor) { motor_ = motor; }
  void set_encoder(SimulatorEncoder* encoder) { encoder_ = encoder; }
  void set_current_sensor(SimulatorCurrent* current) { current_sensor_ = current; }

  /**
   * @brief 每拍更新：从 motor PWM 积分位置、比例电流，回写传感器
   */
  Error Process(float dt);

 private:
  static constexpr float kCpsPerPwm     = 1000.0f;  ///< counts/s per unit PWM
  static constexpr float kCurrentPerPwm = 1.0f;     ///< A per unit PWM

  SimulatorMotor*   motor_          = nullptr;
  SimulatorEncoder* encoder_        = nullptr;
  SimulatorCurrent* current_sensor_ = nullptr;
  float             sim_position_   = 0.0f;
};

}  // namespace zeta::simulator

namespace zeta::simulator {

inline Error SimulatorPlant::Process(float dt) {
  if (motor_ == nullptr || encoder_ == nullptr || current_sensor_ == nullptr) {
    return Error::kInvalidArg;
  }
  const float pwm = motor_->last_pwm();
  sim_position_ += kCpsPerPwm * pwm * dt;
  const auto pos_counts = static_cast<int32_t>(sim_position_);
  encoder_->SetRawPosition(pos_counts);
  current_sensor_->SetCurrent(kCurrentPerPwm * pwm);
  return Error::kOk;
}

}  // namespace zeta::simulator
