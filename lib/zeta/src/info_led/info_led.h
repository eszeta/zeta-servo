// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file info_led.h
 * @brief 信息 LED（按状态闪烁：Ok/Warning/Error/FatalError）
 */

#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#include <iterator>

#include "error.h"
#include "info_led/led.h"
#include "noncopyable.h"
namespace zeta::info_led {

/**
 * @brief LED闪烁模式的基本单元（1字节紧凑编码）
 * bit7: state, bit0-6: duration in 20ms units
 */
struct BlinkUnit : public zeta::Noncopyable {
  uint8_t raw;

  /** @brief 持续时间 [ms] */
  uint16_t duration_ms() const { return static_cast<uint16_t>(raw & 0x7Fu) * 20u; }
  /** @brief 亮(true)/灭(false) */
  bool state() const { return (raw & 0x80u) != 0; }

  static constexpr uint8_t kDurationUnitMs = 20;
  /**
   * @brief 构造紧凑编码
   * @param duration_ms 持续时间 [ms]
   * @param state 亮 true / 灭 false
   * @return 1 字节编码
   */
  static constexpr uint8_t Make(uint16_t duration_ms, bool state) {
    return static_cast<uint8_t>((duration_ms / kDurationUnitMs) | (state ? 0x80u : 0u));
  }
  constexpr BlinkUnit() : raw(0) {}
  constexpr BlinkUnit(uint8_t r) : raw(r) {}
};

/**
 * @brief 信息LED
 * @tparam Mode LED 输出模式（编译期确定，默认开漏）
 */
template <LedMode Mode = LedMode::kOpenDrain>
class InfoLED : public zeta::Noncopyable {
 public:
  enum class InfoType { kOk, kWarning, kError, kFatalError, kMax };

  /**
   * @brief 初始化 LED 引脚
   * @param pin GPIO 引脚号
   */
  void Init(uint32_t pin);
  /**
   * @brief 设置信息类型（切换闪烁模式）
   * @param type 信息类型
   */
  void SetInfo(InfoType type);
  void Stop();
  /**
   * @brief 显示错误码闪烁（N 次快闪表示错误码 N）
   * @param code 错误码 (0–kMax)
   */
  void ShowErrorCode(uint8_t code);
  /**
   * @brief 每拍调用，推进闪烁状态
   * @param dt 时间间隔 [s]
   */
  void Process(float dt);

 private:
  size_t FillErrorCodePattern(uint8_t code);

  LED<Mode>        led_;
  const BlinkUnit* pattern_      = nullptr;
  size_t           pattern_size_ = 0;
  size_t           step_         = 0;

  uint32_t elapsed_ms_ = 0;
  // ShowErrorCode 动态模式缓冲区（2*N+1 单元，N=闪烁次数，与 Error 枚举同步）
  static constexpr uint8_t kMaxCode            = static_cast<uint8_t>(Error::kMax);
  static constexpr size_t  kMaxCodePatternSize = 2 * kMaxCode + 1;
  BlinkUnit                error_code_buffer_[kMaxCodePatternSize]{};

  static constexpr BlinkUnit kOkPattern[2] = {
      BlinkUnit(BlinkUnit::Make(500, true)),
      BlinkUnit(BlinkUnit::Make(500, false)),
  };
  static constexpr BlinkUnit kWarningPattern[2] = {
      BlinkUnit(BlinkUnit::Make(200, true)),
      BlinkUnit(BlinkUnit::Make(200, false)),
  };
  static constexpr BlinkUnit kErrorPattern[6] = {
      BlinkUnit(BlinkUnit::Make(1000, true)), BlinkUnit(BlinkUnit::Make(500, false)),
      BlinkUnit(BlinkUnit::Make(200, true)),  BlinkUnit(BlinkUnit::Make(200, false)),
      BlinkUnit(BlinkUnit::Make(200, true)),  BlinkUnit(BlinkUnit::Make(200, false)),
  };
  static constexpr BlinkUnit kFatalErrorPattern[8] = {
      BlinkUnit(BlinkUnit::Make(200, true)),  BlinkUnit(BlinkUnit::Make(200, false)),
      BlinkUnit(BlinkUnit::Make(200, true)),  BlinkUnit(BlinkUnit::Make(200, false)),
      BlinkUnit(BlinkUnit::Make(200, true)),  BlinkUnit(BlinkUnit::Make(200, false)),
      BlinkUnit(BlinkUnit::Make(1000, true)), BlinkUnit(BlinkUnit::Make(1000, false)),
  };
};

}  // namespace zeta::info_led

namespace zeta::info_led {

namespace {

constexpr uint8_t kBlinkOn    = BlinkUnit::Make(200, true);
constexpr uint8_t kBlinkOff   = BlinkUnit::Make(200, false);
constexpr uint8_t kCodeZeroOn = BlinkUnit::Make(1000, true);
constexpr uint8_t kCycleGap   = BlinkUnit::Make(1000, false);

}  // namespace

template <LedMode Mode>
void InfoLED<Mode>::Init(uint32_t pin) {
  led_.Init(pin);
}

template <LedMode Mode>
void InfoLED<Mode>::SetInfo(InfoType type) {
  const BlinkUnit* p = nullptr;
  size_t           n = 0;
  switch (type) {
    case InfoType::kOk:
      p = kOkPattern;
      n = std::size(kOkPattern);
      break;
    case InfoType::kWarning:
      p = kWarningPattern;
      n = std::size(kWarningPattern);
      break;
    case InfoType::kError:
      p = kErrorPattern;
      n = std::size(kErrorPattern);
      break;
    case InfoType::kFatalError:
      p = kFatalErrorPattern;
      n = std::size(kFatalErrorPattern);
      break;
    default:
      break;
  }
  if (pattern_ == p && pattern_size_ == n) {
    return;
  }
  if (p != nullptr) {
    pattern_      = p;
    pattern_size_ = n;
    step_         = 0;
    elapsed_ms_   = 0;
  }
}

template <LedMode Mode>
void InfoLED<Mode>::Stop() {
  pattern_      = nullptr;
  pattern_size_ = 0;
  step_         = 0;
  elapsed_ms_   = 0;
  led_.Turn(false);
}

template <LedMode Mode>
size_t InfoLED<Mode>::FillErrorCodePattern(uint8_t code) {
  size_t        i = 0;
  const uint8_t n = (code > kMaxCode) ? kMaxCode : code;
  for (uint8_t k = 0; k < n; ++k) {
    error_code_buffer_[i++].raw = kBlinkOn;
    error_code_buffer_[i++].raw = kBlinkOff;
  }
  error_code_buffer_[i++].raw = kCycleGap;
  return i;
}

template <LedMode Mode>
void InfoLED<Mode>::ShowErrorCode(uint8_t code) {
  if (code == 0) {
    SetInfo(InfoType::kOk);
    return;
  }
  const auto n = FillErrorCodePattern(code);
  if (n > 0) {
    pattern_      = error_code_buffer_;
    pattern_size_ = n;
    step_         = 0;
    elapsed_ms_   = 0;
  }
}

template <LedMode Mode>
void InfoLED<Mode>::Process(float dt) {
  if (pattern_ == nullptr || pattern_size_ == 0 || step_ >= pattern_size_) {
    return;
  }
  elapsed_ms_ += static_cast<uint32_t>(dt * 1000.0f);
  if (elapsed_ms_ >= pattern_[step_].duration_ms()) {
    elapsed_ms_ = 0;
    step_       = (step_ + 1) % pattern_size_;
  }
  led_.Turn(pattern_[step_].state());
}
}  // namespace zeta::info_led
