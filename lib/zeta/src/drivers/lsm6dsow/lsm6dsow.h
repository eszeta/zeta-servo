// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file lsm6dsow.h
 * @brief LSM6DSOW IMU（I2C，加速度/陀螺仪/温度）
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "drivers/lsm6dsow/lsm6dsow_regmap.h"
#include "drivers/lsm6dsow/lsm6dsow_types.h"
#include "servo/imu.h"

namespace zeta::drivers::LSM6DSOW {

class IMU;
using Base = servo::IMU<IMU>;

/// @brief LSM6DSOW IMU 实现（I2C）
class IMU final : public Base {
 public:
  static constexpr uint8_t kI2CAddress = 0x6A;

  /// @brief 配置：I2C 总线
  struct Config {
    TwoWire* wire;
  };

  /**
   * @brief 初始化 I2C 与 IMU
   * @param config 配置（I2C 总线）
   * @return 错误码
   */
  Error Init(const Config& config);
  Error ReadAccelerationImpl(float& x, float& y, float& z);
  bool  AccelerationAvailableImpl();
  Error ReadGyroscopeImpl(float& x, float& y, float& z);
  bool  GyroscopeAvailableImpl();
  Error ReadTemperatureImpl(float& temperature_deg);
  bool  TemperatureAvailableImpl();

 private:
  Regmap regmap_;
};

}  // namespace zeta::drivers::LSM6DSOW

namespace zeta::drivers::LSM6DSOW {

inline Error IMU::Init(const Config& config) {
  CHECK(regmap_.Init(config.wire, kI2CAddress));
  return Error::kOk;
}

inline Error IMU::ReadAccelerationImpl(float& x, float& y, float& z) {
  return regmap_.ReadAcceleration(x, y, z);
}

inline bool IMU::AccelerationAvailableImpl() {
  return regmap_.AccelerationAvailable();
}

inline Error IMU::ReadGyroscopeImpl(float& x, float& y, float& z) {
  return regmap_.ReadGyroscope(x, y, z);
}

inline bool IMU::GyroscopeAvailableImpl() {
  return regmap_.GyroscopeAvailable();
}

inline Error IMU::ReadTemperatureImpl(float& temperature_deg) {
  return regmap_.ReadTemperature(temperature_deg);
}

inline bool IMU::TemperatureAvailableImpl() {
  return regmap_.TemperatureAvailable();
}

}  // namespace zeta::drivers::LSM6DSOW