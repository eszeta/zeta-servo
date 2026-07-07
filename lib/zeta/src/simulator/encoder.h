// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file encoder.h
 * @brief 仿真编码器（位置由 plant 写入，供算法验证）
 */

#pragma once

#include "error.h"
#include "math/math.h"
#include "math/resolution.h"
#include "servo/encoder.h"
#include "servo/servo_types.h"

namespace zeta::simulator {

/// @brief 仿真编码器分辨率位数，与 Servo kResolutionBits 一致
constexpr uint8_t kSimEncoderBits = 12;

/**
 * @brief 仿真用编码器
 *
 * 不读真实硬件，原始位置由仿真 plant 通过 SetRawPosition 写入，用于算法验证。
 * 仿真值存于 simulated_raw_，ReadRawImpl 读该值；基类 raw_pos_ 仅在 Init/Process 中更新。
 */
class SimulatorEncoder final : public servo::Encoder<SimulatorEncoder, kSimEncoderBits> {
 public:
  /**
   * @brief 设置仿真原始位置（按 CPR 取模到 [0, CPR-1]）
   *
   * 由仿真 plant 每步调用，供下一拍 Servo::Process 读取。
   */
  void SetRawPosition(int32_t pos_counts);

  Error ReadRawImpl(uint32_t& out_raw);

 private:
  uint32_t simulated_raw_ = 0;
};

}  // namespace zeta::simulator

namespace zeta::simulator {

inline void SimulatorEncoder::SetRawPosition(int32_t pos_counts) {
  constexpr int kCpr = static_cast<int>(kResolution.kEncoderCpr);
  simulated_raw_     = static_cast<uint32_t>(math::mod(pos_counts, kCpr));
}

inline Error SimulatorEncoder::ReadRawImpl(uint32_t& out_raw) {
  out_raw = simulated_raw_;
  return Error::kOk;
}

}  // namespace zeta::simulator
