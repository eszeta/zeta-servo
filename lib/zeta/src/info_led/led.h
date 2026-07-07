// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file led.h
 * @brief LED 驱动（开漏/推挽，Turn/Toggle）
 */

#pragma once

#include <Arduino.h>

#include "noncopyable.h"

namespace zeta::info_led {

/**
 * @brief 模式枚举
 */
enum class LedMode {
  /**
   * @brief 开漏输出
   */
  kOpenDrain = 0,
  /**
   * @brief 推挽输出
   */
  kPushPull = 1,
};

/**
 * @brief LED类
 * @tparam Mode 输出模式（编译期确定，零运行时分支）
 */
template <LedMode Mode>
class LED final : public zeta::Noncopyable {
 public:
  /**
   * @brief 初始化
   * @param pin GPIO 引脚号
   */
  void Init(uint32_t pin);
  /**
   * @brief 设置状态
   * @param value true 亮 / false 灭
   */
  void Turn(bool value);
  /**
   * @brief 切换状态
   */
  void Toggle();

 private:
  bool     state_ = false;  ///< 当前亮/灭
  uint32_t pin_   = NC;     ///< GPIO 引脚
};

}  // namespace zeta::info_led

namespace zeta::info_led {

template <LedMode Mode>
void LED<Mode>::Init(uint32_t pin) {
  pin_ = pin;
  if constexpr (Mode == LedMode::kOpenDrain) {
    pinMode(pin_, OUTPUT_OPEN_DRAIN);
  } else {
    pinMode(pin_, OUTPUT);
  }
  Turn(false);
}

template <LedMode Mode>
void LED<Mode>::Turn(bool value) {
  if (value == state_) {
    return;
  }
  state_ = value;
  if constexpr (Mode == LedMode::kOpenDrain) {
    digitalWrite(pin_, value ? LOW : HIGH);
  } else {
    digitalWrite(pin_, value ? HIGH : LOW);
  }
}

template <LedMode Mode>
void LED<Mode>::Toggle() {
  Turn(!state_);
}

}  // namespace zeta::info_led
