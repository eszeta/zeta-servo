// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file error.h
 * @brief 错误码与错误传播宏
 */

#pragma once

#include <stdint.h>

namespace zeta {

/**
 * @brief 错误码枚举
 *
 * 精简的通用错误码，覆盖嵌入式常见故障类型。
 * 用 uint8_t 底层类型，仅占 1 字节，零开销传播。
 */
enum class Error : uint8_t {
  kOk = 0,       ///< 成功
  kErr,          ///< 通用错误（无法归类时使用）
  kIO,           ///< 通信/IO 错误（I2C/SPI 失败）
  kTimeout,      ///< 超时
  kOutOfRange,   ///< 值或索引越界
  kInvalidArg,   ///< 无效参数（空指针、非法组合）
  kBadData,      ///< 数据校验失败（协议格式错误、校验和错误、未知指令等）
  kBadState,     ///< 当前状态下不允许此操作
  kUnsupported,  ///< 功能未实现或不支持
  kMax,          ///< 最大错误码（用于数组索引）
};

/// @brief 判断是否为成功码
/// @param e 错误码
/// @return 若 e == kOk 则为 true
bool IsOk(Error e);

/// @brief 判断是否为失败码
/// @param e 错误码
/// @return 若 e != kOk 则为 true
bool IsFail(Error e);

}  // namespace zeta

namespace zeta {

inline bool IsOk(Error e) {
  return e == Error::kOk;
}
inline bool IsFail(Error e) {
  return e != Error::kOk;
}

}  // namespace zeta

/** @name 错误传播宏 */
/// @{

/// @brief 错误传播：执行表达式，非 kOk 则立即返回该错误码
///
/// 使用 __VA_ARGS__ 兼容含逗号的模板调用，如 CHECK(Func<A, B>(v))。
#define CHECK(...)                         \
  do {                                     \
    ::zeta::Error _err_ = (__VA_ARGS__); \
    if (_err_ != ::zeta::Error::kOk)     \
      return _err_;                        \
  } while (0)

/// @brief 前置条件守卫：条件为 false 则返回指定错误码
///
/// 替代 `if (!cond) return err;`，使参数校验更简洁。
#define VERIFY(cond, err) \
  do {                    \
    if (!(cond))          \
      return (err);       \
  } while (0)

/// @}
