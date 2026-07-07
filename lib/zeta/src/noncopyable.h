// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file noncopyable.h
 * @brief 禁止拷贝的基类
 */

#pragma once

namespace zeta {

/// @brief 禁止拷贝的基类，继承后子类不可拷贝构造与赋值
class Noncopyable {
 private:
  explicit Noncopyable(Noncopyable const&)   = delete;
  Noncopyable& operator=(Noncopyable const&) = delete;

 public:
  Noncopyable()  = default;
  ~Noncopyable() = default;
};
}  // namespace zeta