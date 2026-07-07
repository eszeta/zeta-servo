// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file current.h
 * @brief 电流传感器抽象基类（CRTP）
 */

#pragma once

#include <Arduino.h>

#include "error.h"
#include "noncopyable.h"
#include "servo/servo_types.h"

namespace zeta::servo {

/**
 * @brief 电流传感器抽象基类
 * @details
 * 所有具体的电流传感器实现都必须继承自此类。该类提供了电流检测的基本接口定义。
 */
template <typename DerivedType>
class Current : public zeta::Noncopyable {
 public:
  /**
   * @brief 初始化电流传感器
   * @return 错误码
   */
  Error Init();

  /**
   * @brief 读取当前电流
   * @param current 输出电流 [A]
   * @return 错误码
   */
  Error ReadCurrent(float& current);

  /**
   * @brief 多次采样并平滑得到平均电流
   * @param n 采样次数
   * @param current 输入为初值，输出为平滑后的电流 [A]
   * @return 错误码
   */
  Error ReadAverageCurrents(int n, float& current);
};

}  // namespace zeta::servo

namespace zeta::servo {

template <typename DerivedType>
Error Current<DerivedType>::Init() {
  return Error::kOk;
}

template <typename DerivedType>
Error Current<DerivedType>::ReadCurrent(float& current) {
  return static_cast<DerivedType*>(this)->ReadCurrentImpl(current);
}

template <typename DerivedType>
Error Current<DerivedType>::ReadAverageCurrents(int n, float& current) {
  CHECK(ReadCurrent(current));
  for (int i = 0; i < n; ++i) {
    float new_current;
    CHECK(ReadCurrent(new_current));
    current = current * 0.6f + 0.4f * new_current;
    delay(3);
  }
  return Error::kOk;
}

}  // namespace zeta::servo
