// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file drv8231a.h
 * @brief DRV8231A 双 H 桥电机驱动（GPIO/PWM）
 */

#pragma once

#include <Arduino.h>

#include "servo/motor.h"

namespace zeta::drivers::DRV8231A {

class Motor;
using Base = servo::Motor<Motor>;

/// @brief DRV8231A 电机驱动实现（双 H 桥，IN1/IN2 + 可选 nFAULT）
class Motor final : public Base {
 public:
  /// @brief 配置：引脚与低速衰减阈值
  struct Config {
    uint8_t pin_in1;                      ///< IN1 控制引脚
    uint8_t pin_in2;                      ///< IN2 控制引脚
    uint8_t pin_nfault           = 0;     ///< nFAULT 引脚（0 表示不使用）
    float   slow_decay_threshold = 0.3f;  ///< 低速阈值（低于此值使用慢速衰减）
  };

  /**
   * @brief 初始化引脚与参数
   * @param config 配置（引脚与低速衰减阈值）
   * @return 错误码
   */
  Error Init(const Config& config);
  void  SetPWMImpl(float pwm);
  void  BrakeImpl();
  void  CoastImpl();
  /**
   * @brief 是否检测到故障（nFAULT 低）
   * @return 有故障为 true，无 nFAULT 或未触发为 false
   */
  bool HasFault() const;

 private:
  void SetPWMFastDecay(float pwm);
  void SetPWMSlowDecay(float pwm);

  uint8_t pin_in1_              = 0;
  uint8_t pin_in2_              = 0;
  uint8_t pin_nfault_           = 0;
  float   slow_decay_threshold_ = 0.3f;
};

}  // namespace zeta::drivers::DRV8231A

namespace zeta::drivers::DRV8231A {

inline Error Motor::Init(const Config& config) {
  VERIFY(config.pin_in1 != 0 && config.pin_in2 != 0, Error::kInvalidArg);

  CHECK(Base::Init());

  pin_in1_              = config.pin_in1;
  pin_in2_              = config.pin_in2;
  pin_nfault_           = config.pin_nfault;
  slow_decay_threshold_ = config.slow_decay_threshold;

  pinMode(pin_in1_, OUTPUT);
  pinMode(pin_in2_, OUTPUT);

  if (pin_nfault_ != 0) {
    pinMode(pin_nfault_, INPUT_PULLUP);
  }

  digitalWrite(pin_in1_, LOW);
  digitalWrite(pin_in2_, LOW);

  return Error::kOk;
}

inline void Motor::SetPWMImpl(float pwm) {
  pwm = pwm * static_cast<int8_t>(reverse_);
  pwm = constrain(pwm, -1.0f, 1.0f);

  const float abs_pwm = abs(pwm);

  if (abs_pwm < slow_decay_threshold_) {
    SetPWMSlowDecay(pwm);
  } else {
    SetPWMFastDecay(pwm);
  }
}

inline void Motor::BrakeImpl() {
  digitalWrite(pin_in1_, HIGH);
  digitalWrite(pin_in2_, HIGH);
}

inline void Motor::CoastImpl() {
  digitalWrite(pin_in1_, LOW);
  digitalWrite(pin_in2_, LOW);
}

inline bool Motor::HasFault() const {
  if (pin_nfault_ == 0) {
    return false;
  }
  return digitalRead(pin_nfault_) == LOW;
}

inline void Motor::SetPWMFastDecay(float pwm) {
  if (pwm > 0.0f) {
    analogWrite(pin_in1_, static_cast<uint32_t>(255 * pwm));
    digitalWrite(pin_in2_, LOW);
  } else if (pwm < 0.0f) {
    digitalWrite(pin_in1_, LOW);
    analogWrite(pin_in2_, static_cast<uint32_t>(255 * (-pwm)));
  } else {
    digitalWrite(pin_in1_, LOW);
    digitalWrite(pin_in2_, LOW);
  }
}

inline void Motor::SetPWMSlowDecay(float pwm) {
  if (pwm > 0.0f) {
    digitalWrite(pin_in1_, HIGH);
    analogWrite(pin_in2_, static_cast<uint32_t>(255 * (1.0f - pwm)));
  } else if (pwm < 0.0f) {
    analogWrite(pin_in1_, static_cast<uint32_t>(255 * (1.0f + pwm)));
    digitalWrite(pin_in2_, HIGH);
  } else {
    digitalWrite(pin_in1_, LOW);
    digitalWrite(pin_in2_, LOW);
  }
}

}  // namespace zeta::drivers::DRV8231A