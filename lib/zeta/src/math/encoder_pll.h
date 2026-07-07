// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file encoder_pll.h
 * @brief 编码器 PLL 位置与速度估计
 */

#pragma once

#include <Arduino.h>

#include "math/math.h"
#include "math/resolution.h"
#include "servo/servo_types.h"

namespace zeta::math {

/**
 * @brief 编码器 PLL，从编码器读数估计平滑位置与速度
 * @tparam EncoderType 编码器类型（CRTP 派生）
 * @tparam Bits 分辨率位数
 */
template <typename EncoderType, uint8_t Bits>
class EncoderPll : public zeta::Noncopyable {
 public:
  /**
   * @brief 初始化 PLL，从编码器当前位建立初始状态
   * @param encoder 编码器指针，须已初始化
   * @return 错误码
   */
  Error Init(EncoderType* encoder);

  /**
   * @brief 返回绑定的编码器指针
   * @return 编码器指针
   */
  EncoderType* encoder() const;

  /**
   * @brief 估计位置
   * @return 估计位置 [pulse]
   */
  float pos() const;

  /**
   * @brief 估计速度（线速度）
   * @return 估计速度 [pulse/s]
   */
  float velocity() const;

  /**
   * @brief 估计转速
   * @return 估计转速 [RPM]
   */
  float rpm() const;

  /**
   * @brief 处理编码器数据
   * @param dt 时间间隔(秒)
   * @return 错误码
   */
  Error Process(float dt);

 private:
  static constexpr Resolution<Bits> kResolution{};

  /// @brief PLL带宽 [Hz]
  static constexpr float kPllBandwidth = 200.0f;
  /// @brief PLL比例增益
  static constexpr float kPllKp = 2.0f * kPllBandwidth;
  /// @brief PLL积分增益（临界阻尼）
  static constexpr float kPllKi = 0.25f * kPllKp * kPllKp;
  /// @brief 速度死区 [pulse/s]，低于此值视为静止
  static constexpr float kVelocityDeadband = 1.0f;

  // PLL 状态变量
  /// @brief 线性位置估计（平滑后）
  float pos_ = 0.0f;
  /// @brief 速度估计 [pulse/s]
  float velocity_ = 0.0f;
  /// @brief 速度估计 [RPM]
  float rpm_ = 0.0f;

  /// @brief 角度传感器
  EncoderType* encoder_ = nullptr;
};

}  // namespace zeta::math

namespace zeta::math {

template <typename EncoderType, uint8_t Bits>
Error EncoderPll<EncoderType, Bits>::Init(EncoderType* encoder) {
  encoder_                = encoder;
  const auto encoder_pos  = encoder->pos();
  const auto encoder_bits = encoder->kResolution.kBits;
  const auto init_pos     = math::mapResolutionCpr(encoder_pos, encoder_bits, kResolution.kBits);
  pos_                    = static_cast<float>(init_pos);
  velocity_               = 0.0f;
  return Error::kOk;
}

template <typename EncoderType, uint8_t Bits>
EncoderType* EncoderPll<EncoderType, Bits>::encoder() const {
  return encoder_;
}

template <typename EncoderType, uint8_t Bits>
float EncoderPll<EncoderType, Bits>::pos() const {
  return pos_;
}

template <typename EncoderType, uint8_t Bits>
float EncoderPll<EncoderType, Bits>::velocity() const {
  return velocity_;
}

template <typename EncoderType, uint8_t Bits>
float EncoderPll<EncoderType, Bits>::rpm() const {
  return rpm_;
}

template <typename EncoderType, uint8_t Bits>
Error EncoderPll<EncoderType, Bits>::Process(float dt) {
  CHECK(encoder_->Process(dt));
  const auto encoder_pos  = encoder_->pos();
  const auto encoder_bits = encoder_->kResolution.kBits;
  const auto mapped       = math::mapResolutionCpr(encoder_pos, encoder_bits, kResolution.kBits);
  pos_ += dt * velocity_;

  float error = static_cast<float>(mapped) - pos_;

  pos_ += dt * kPllKp * error;
  velocity_ += dt * kPllKi * error;

  if (fabs(velocity_) < kVelocityDeadband) {
    velocity_ = 0.0f;
  }
  rpm_ = (velocity_ / kResolution.kEncoderCpr) * 60.0f;
  return Error::kOk;
}

}  // namespace zeta::math