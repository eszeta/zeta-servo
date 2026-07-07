// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file timeout_limiter.h
 * @brief 超时限制器（电流等超过阈值持续一定时间后触发）
 */

#pragma once

#include <Arduino.h>

#include "noncopyable.h"

namespace zeta::utils {

/// @brief 超时限制器：当 current_value 超过阈值且持续超过 timeout_duration 时 Process 返回 true
class TimeoutLimiter : public zeta::Noncopyable {
 public:
  void set_threshold(float threshold);
  /** @brief 当前阈值 */
  float threshold() const;
  void  set_timeout_duration(float timeout_duration);
  /** @brief 当前超时时长 [s] */
  float timeout_duration() const;

  /**
   * @brief 更新状态，超过阈值则累加时间，否则清零；返回是否已超时
   * @param current_value 当前检测值
   * @param dt 时间步长 [s]
   * @return 是否已连续超时达到 timeout_duration
   */
  bool Process(float current_value, float dt);
  void Reset();

 private:
  float threshold_         = 0.0f;
  float timeout_duration_  = 0.0f;
  float exceeded_duration_ = 0.0f;
};

}  // namespace zeta::utils

namespace zeta::utils {

inline void TimeoutLimiter::set_threshold(float threshold) {
  threshold_ = threshold;
}

inline float TimeoutLimiter::threshold() const {
  return threshold_;
}

inline void TimeoutLimiter::set_timeout_duration(float timeout_duration) {
  timeout_duration_ = timeout_duration;
}

inline float TimeoutLimiter::timeout_duration() const {
  return timeout_duration_;
}

inline bool TimeoutLimiter::Process(float current_value, float dt) {
  if (current_value > threshold_) {
    exceeded_duration_ += dt;
  } else {
    exceeded_duration_ = 0.0f;
  }
  return exceeded_duration_ >= timeout_duration_;
}

inline void TimeoutLimiter::Reset() {
  exceeded_duration_ = 0.0f;
}

}  // namespace zeta::utils
