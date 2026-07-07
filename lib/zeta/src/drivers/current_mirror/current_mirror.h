// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file current_mirror.h
 * @brief 电流镜电流检测（RIPROPI + ADC）
 */

#pragma once

#include <Arduino.h>

#include "servo/current.h"

namespace zeta::drivers::CurrentMirror {

class Current;
using Base = servo::Current<Current>;

/// @brief 电流镜电流传感器（RIPROPI + ADC，Init 时校准零点）
class Current : public Base {
 public:
  /// @brief 配置：ADC、电阻、换算与校准次数
  struct Config {
    uint8_t  pin_adc;              ///< ADC 引脚
    float    ripropi_ohms;         ///< RIPROPI 电阻 [Ω]
    float    scaling_factor;       ///< 电流镜系数 [μA/A]
    uint16_t adc_resolution_bits;  ///< ADC 分辨率 [bit]
    float    adc_vref_volts;       ///< ADC 参考电压 [V]
    uint16_t calibration_samples;  ///< 校准采样次数
  };

  /**
   * @brief 初始化 ADC 与零点校准
   * @param config 配置（ADC、电阻、换算与校准次数）
   * @return 错误码
   */
  Error Init(const Config& config);
  Error ReadCurrentImpl(float& current);

 private:
  static constexpr float kAdcVoltageConv = 3.3f / 1024.0f;

  Error CalibrateOffsets();
  Error ReadADCVoltage(float& voltage);

  float    adc_to_voltage_      = 0.0f;
  float    voltage_to_current_  = 0.0f;
  float    zero_offset_voltage_ = 0.0f;
  uint8_t  pin_adc_             = 0;
  uint16_t calibration_samples_ = 50;
};

}  // namespace zeta::drivers::CurrentMirror

namespace zeta::drivers::CurrentMirror {

inline Error Current::Init(const Config& config) {
  CHECK(Base::Init());
  VERIFY(config.pin_adc != 0, Error::kInvalidArg);
  pin_adc_             = config.pin_adc;
  calibration_samples_ = config.calibration_samples;

  pinMode(pin_adc_, INPUT);

  const uint32_t adc_max_value = (1 << config.adc_resolution_bits) - 1;
  adc_to_voltage_              = config.adc_vref_volts / static_cast<float>(adc_max_value);

  const float current_sense_ratio = config.scaling_factor / 1e6f;
  voltage_to_current_             = 1.0f / (config.ripropi_ohms * current_sense_ratio);

  CHECK(CalibrateOffsets());

  return Error::kOk;
}

inline Error Current::ReadCurrentImpl(float& current) {
  float voltage;
  CHECK(ReadADCVoltage(voltage));

  const float voltage_diff = voltage - zero_offset_voltage_;
  current                  = voltage_diff * voltage_to_current_;

  return Error::kOk;
}

inline Error Current::CalibrateOffsets() {
  zero_offset_voltage_ = 0.0f;

  for (uint16_t i = 0; i < calibration_samples_; ++i) {
    float voltage;
    CHECK(ReadADCVoltage(voltage));
    zero_offset_voltage_ += voltage;
    delay(2);
  }

  zero_offset_voltage_ /= static_cast<float>(calibration_samples_);
  return Error::kOk;
}

inline Error Current::ReadADCVoltage(float& voltage) {
  const uint16_t adc_raw = analogRead(pin_adc_);
  voltage                = static_cast<float>(adc_raw) * adc_to_voltage_;
  return Error::kOk;
}

}  // namespace zeta::drivers::CurrentMirror