// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file math.h
 * @brief 数学工具与分辨率映射
 */

#pragma once

#include <Arduino.h>

#include <type_traits>

namespace zeta::math {

/// @brief 微秒转换为秒
constexpr float kMicroToSec = 1e-6f;

/// @brief 毫秒转换为秒
constexpr float kMilliToSec = 1e-3f;

/// @brief 浮点数比较阈值，用于判断值是否接近 0
constexpr float kFloatThreshold = 0.001f;

/// @brief 按 max（2^N−1）缩放分辨率，与 STM32duino 行为一致。
/// @details 0 映射到 0，max(from) 映射到 max(to)，保持满量程语义。
///          适用于 PWM 占空比、DAC 输出等非回绕量。
/// @tparam T 数值类型（整型或浮点型）。
/// @param value 待映射的原始值。
/// @param from 输入分辨率位数。
/// @param to 输出分辨率位数。
/// @return 映射后的值。
template <typename T>
constexpr T mapResolution(T value, uint8_t from, uint8_t to);

/// @brief 按 CPR（2^N）缩放分辨率，保持物理角度不变。
/// @details 等价于 value × 2^to / 2^from。
///          适用于编码器位置等回绕量。
/// @tparam T 数值类型（整型或浮点型）。
/// @param value 待映射的原始值。
/// @param from 输入分辨率位数。
/// @param to 输出分辨率位数。
/// @return 映射后的值。
template <typename T>
constexpr T mapResolutionCpr(T value, uint8_t from, uint8_t to);

/// @brief 计算始终为非负的模运算结果。
/// @param dividend 被除数。
/// @param divisor 除数（应为正且非零）。
/// @return 区间 `[0, divisor)` 内的结果。
constexpr int mod(const int dividend, const int divisor);

/// @brief 计算始终为非负的浮点模运算结果。
/// @param x 被除数。
/// @param y 除数（应为正且非零）。
/// @return 区间 `[0, y)` 内的结果。
constexpr float fmodf_pos(float x, float y);

/// @brief 将浮点值折叠到以 0 为中心的最短距离表示。
/// @param x 待折叠值。
/// @param y 周期（应为正且非零）。
/// @return 结果位于 `[-y/2, y/2)`。
constexpr float wrap_pm(float x, float y);

/// @brief 将整数值折叠到以 0 为中心的最短距离表示。
/// @param x 待折叠值。
/// @param y 周期（应为正且非零）。
/// @return 结果位于 `[-y/2, y/2)`（离散整数区间）。
constexpr int32_t wrap_pm(int32_t x, int32_t y);

}  // namespace zeta::math

namespace zeta::math {

template <typename T>
constexpr T mapResolution(T value, uint8_t from, uint8_t to) {
  if (from == to)
    return value;
  if constexpr (std::is_integral_v<T>) {
    const int64_t max_from = static_cast<int64_t>((1ULL << from) - 1);
    const int64_t max_to   = static_cast<int64_t>((1ULL << to) - 1);
    return static_cast<T>((static_cast<int64_t>(value) * max_to) / max_from);
  } else {
    const float max_from = static_cast<float>((1ULL << from) - 1);
    const float max_to   = static_cast<float>((1ULL << to) - 1);
    return static_cast<T>(value * (max_to / max_from));
  }
}

template <typename T>
constexpr T mapResolutionCpr(T value, uint8_t from, uint8_t to) {
  if (from == to)
    return value;
  const float cpr_from = static_cast<float>(1ULL << from);
  const float cpr_to   = static_cast<float>(1ULL << to);
  return static_cast<T>(value * (cpr_to / cpr_from));
}

constexpr int mod(const int dividend, const int divisor) {
  int r = dividend % divisor;
  if (r < 0)
    r += divisor;
  return r;
}

constexpr float fmodf_pos(float x, float y) {
  float r = fmod(x, y);
  if (r < 0)
    r += y;
  return r;
}

constexpr float wrap_pm(float x, float y) {
  if (y <= 0.0f)
    return 0.0f;
  return fmodf_pos(x + 0.5f * y, y) - 0.5f * y;
}

constexpr int32_t wrap_pm(int32_t x, int32_t y) {
  if (y <= 0)
    return 0;
  int32_t wrapped = x % y;
  wrapped += (wrapped < 0) ? y : 0;
  const int32_t half = y / 2;
  return wrapped - ((wrapped >= (y - half)) ? y : 0);
}

}  // namespace zeta::math