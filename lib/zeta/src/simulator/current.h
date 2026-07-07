// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file current.h
 * @brief 仿真电流传感器（电流由 plant 写入，供算法验证）
 */

#pragma once

#include "servo/current.h"

namespace zeta::simulator {

/**
 * @brief 仿真用电流传感器
 *
 * 不读真实硬件，电流值由仿真 plant 通过 SetCurrent 写入，用于算法验证。
 */
class SimulatorCurrent final : public servo::Current<SimulatorCurrent> {
 public:
  /**
   * @brief 设置仿真电流（A）
   *
   * 由仿真 plant 每步调用，供下一拍 Servo::Process 读取。
   */
  void SetCurrent(float current) { current_ = current; }

  Error ReadCurrentImpl(float& current);

 private:
  float current_ = 0.0f;
};

}  // namespace zeta::simulator

namespace zeta::simulator {

inline Error SimulatorCurrent::ReadCurrentImpl(float& current) {
  current = current_;
  return Error::kOk;
}

}  // namespace zeta::simulator
