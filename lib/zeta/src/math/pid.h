// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file pid.h
 * @brief PID 控制器
 */

#pragma once

#include <Arduino.h>

#include "math/math.h"
#include "noncopyable.h"

namespace zeta::math {

/**
 * @brief PID 控制器类
 */
class Pid : public zeta::Noncopyable {
 public:
  /// @brief PID 配置参数
  struct Config {
    float kp;     ///< 比例增益 Kp
    float ki;     ///< 积分增益 Ki
    float kd;     ///< 微分增益 Kd
    float ka;     ///< 抗饱和增益 Ka，用于抑制积分饱和
    float limit;  ///< 输出限幅值
  };

  explicit Pid(const Config& config);

  /**
   * @brief 计算 PID 控制器输出
   * @param error 控制误差
   * @param dt 时间间隔 [s]
   * @param feedforward 前馈量，默认 0
   * @return 控制输出（已限幅）
   */
  float Compute(const float error, const float dt, const float feedforward = 0.0f);

  /** @brief 重置积分与微分状态 */
  void Reset();

  /**
   * @brief 设置比例增益
   * @param kp 比例增益
   */
  void set_kp(float kp);
  /** @brief 比例增益 */
  float kp() const;

  /**
   * @brief 设置积分增益
   * @param ki 积分增益
   */
  void set_ki(float ki);
  /** @brief 积分增益 */
  float ki() const;

  /**
   * @brief 设置微分增益
   * @param kd 微分增益
   */
  void set_kd(float kd);
  /** @brief 微分增益 */
  float kd() const;

  /**
   * @brief 设置抗饱和增益
   * @param ka 抗饱和增益
   */
  void set_ka(float ka);
  /** @brief 抗饱和增益 */
  float ka() const;

  /**
   * @brief 设置输出限幅值
   * @param limit 限幅值
   */
  void set_limit(float limit);
  /** @brief 输出限幅值 */
  float limit() const;

 private:
  Config config_;
  float  previous_error_       = 0.0f;
  float  integral_accumulator_ = 0.0f;
  float  antiwindup_feedback_  = 0.0f;
};

}  // namespace zeta::math

namespace zeta::math {

inline Pid::Pid(const Config& config) : config_(config) {}

inline float Pid::Compute(const float error, const float dt, const float feedforward) {
  const float proportional = config_.kp * error;

  const float integral = integral_accumulator_ +
                         config_.ki * dt * 0.5f * (error + previous_error_) -
                         config_.ka * antiwindup_feedback_;

  const float derivative = config_.kd * (error - previous_error_) / dt;

  const float output = proportional + integral + derivative + feedforward;

  const float output_clamped = constrain(output, -config_.limit, config_.limit);

  const float saturation_error = output - output_clamped;

  integral_accumulator_ = integral;
  previous_error_       = error;
  antiwindup_feedback_  = saturation_error;
  return output_clamped;
}

inline void Pid::Reset() {
  integral_accumulator_ = 0.0f;
  antiwindup_feedback_  = 0.0f;
  previous_error_       = 0.0f;
}

inline void Pid::set_kp(float kp) {
  config_.kp = kp;
}
inline float Pid::kp() const {
  return config_.kp;
}

inline void Pid::set_ki(float ki) {
  config_.ki = ki;
}
inline float Pid::ki() const {
  return config_.ki;
}

inline void Pid::set_kd(float kd) {
  config_.kd = kd;
}
inline float Pid::kd() const {
  return config_.kd;
}

inline void Pid::set_ka(float ka) {
  config_.ka = ka;
}
inline float Pid::ka() const {
  return config_.ka;
}

inline void Pid::set_limit(float limit) {
  config_.limit = limit;
}
inline float Pid::limit() const {
  return config_.limit;
}

}  // namespace zeta::math
