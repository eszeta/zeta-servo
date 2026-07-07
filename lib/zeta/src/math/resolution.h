// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file resolution.h
 * @brief 分辨率与 CPR 常量
 */

#pragma once

#include <Arduino.h>

#include "noncopyable.h"

namespace zeta::math {

/**
 * @brief 固定位数分辨率常量（位数、CPR、角度/弧度换算）
 * @tparam Bits 分辨率位数，CPR = 2^Bits
 */
template <uint8_t Bits>
struct Resolution : public zeta::Noncopyable {
 public:
  constexpr Resolution() = default;

  /// @brief 目标分辨率（位数），决定了传感器的精度和量程
  static constexpr uint8_t kBits = Bits;
  /// @brief 最大值
  static constexpr uint32_t kMax = (1 << Bits) - 1;
  /// @brief Counts Per Revolution
  static constexpr uint16_t kEncoderCpr = (1 << Bits);
  /// @brief 角度到计数值的转换系数，用于将角度转换为计数值
  static constexpr float kAngleToRaw = kEncoderCpr / 360.0f;
  /// @brief 弧度到计数值的转换系数，用于将弧度转换为计数值
  static constexpr float kRadianToRaw = kEncoderCpr / TWO_PI;
  /// @brief 计数值到角度的转换系数，用于将计数值转换为角度
  static constexpr float kRawToAngle = 360.0f / kEncoderCpr;
  /// @brief 计数值到弧度的转换系数，用于将计数值转换为弧度
  static constexpr float kRawToRadian = TWO_PI / kEncoderCpr;
};

}  // namespace zeta::math