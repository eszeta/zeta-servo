// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file imu.h
 * @brief IMU 抽象基类（CRTP），加速度/陀螺仪/温度
 */

#pragma once

#include "error.h"
#include "noncopyable.h"
#include "servo/servo_types.h"

namespace zeta::servo {

/**
 * @brief IMU 基本接口
 */
template <typename DerivedType>
class IMU : public zeta::Noncopyable {
 public:
  /**
   * @brief 读取加速度
   * @param x 输出 X 轴 [g]
   * @param y 输出 Y 轴 [g]
   * @param z 输出 Z 轴 [g]
   * @return 错误码
   */
  Error ReadAcceleration(float& x, float& y, float& z);

  /**
   * @brief 加速度数据是否就绪
   * @return 就绪为 true
   */
  bool AccelerationAvailable();

  /**
   * @brief 读取陀螺仪
   * @param x 输出 X 轴 [°/s]
   * @param y 输出 Y 轴 [°/s]
   * @param z 输出 Z 轴 [°/s]
   * @return 错误码
   */
  Error ReadGyroscope(float& x, float& y, float& z);

  /**
   * @brief 陀螺仪数据是否就绪
   * @return 就绪为 true
   */
  bool GyroscopeAvailable();

  /**
   * @brief 读取温度
   * @param temperature_deg 输出温度 [°C]
   * @return 错误码
   */
  Error ReadTemperature(float& temperature_deg);

  /**
   * @brief 温度数据是否就绪
   * @return 就绪为 true
   */
  bool TemperatureAvailable();
};

}  // namespace zeta::servo

namespace zeta::servo {

template <typename DerivedType>
Error IMU<DerivedType>::ReadAcceleration(float& x, float& y, float& z) {
  return static_cast<DerivedType*>(this)->ReadAccelerationImpl(x, y, z);
}

template <typename DerivedType>
bool IMU<DerivedType>::AccelerationAvailable() {
  return static_cast<DerivedType*>(this)->AccelerationAvailableImpl();
}

template <typename DerivedType>
Error IMU<DerivedType>::ReadGyroscope(float& x, float& y, float& z) {
  return static_cast<DerivedType*>(this)->ReadGyroscopeImpl(x, y, z);
}

template <typename DerivedType>
bool IMU<DerivedType>::GyroscopeAvailable() {
  return static_cast<DerivedType*>(this)->GyroscopeAvailableImpl();
}

template <typename DerivedType>
Error IMU<DerivedType>::ReadTemperature(float& temperature_deg) {
  return static_cast<DerivedType*>(this)->ReadTemperatureImpl(temperature_deg);
}

template <typename DerivedType>
bool IMU<DerivedType>::TemperatureAvailable() {
  return static_cast<DerivedType*>(this)->TemperatureAvailableImpl();
}

}  // namespace zeta::servo
