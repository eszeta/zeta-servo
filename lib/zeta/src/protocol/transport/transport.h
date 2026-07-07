// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file transport.h
 * @brief 传输层接口（CRTP），字节读写 + 中断回调
 */

#pragma once

#include <Arduino.h>

#include "error.h"
#include "noncopyable.h"

namespace zeta::protocol {

/**
 * @brief 传输层接口（CRTP 模式）
 *
 * 纯字节读写，无协议解析。派生类实现具体传输（I2C/Serial 等）。
 * OnReceive / OnRequest 供硬件中断回调注册用，不支持的传输层可空实现。
 *
 * @tparam DerivedType 派生类类型
 */
template <typename DerivedType>
class Transport : public zeta::Noncopyable {
 public:
  /**
   * @brief 读取一字节
   * @param byte 输出字节
   * @return 成功返回 true，无数据返回 false
   */
  bool ReadByte(uint8_t& byte) { return static_cast<DerivedType*>(this)->ReadByteImpl(byte); }

  /** @brief 当前可读字节数 */
  size_t Available() { return static_cast<DerivedType*>(this)->AvailableImpl(); }

  /**
   * @brief 写入字节
   * @param data 数据指针
   * @param size 字节数
   * @return 错误码
   */
  Error Write(const uint8_t* data, const size_t size) {
    return static_cast<DerivedType*>(this)->WriteImpl(data, size);
  }

  /**
   * @brief 接收中断回调（如 I2C onReceive）
   * @param n 本次收到的字节数
   */
  void OnReceive(int n) { static_cast<DerivedType*>(this)->OnReceiveImpl(n); }

  /**
   * @brief 请求发送回调（如 I2C onRequest）
   */
  void OnRequest() { static_cast<DerivedType*>(this)->OnRequestImpl(); }
};

}  // namespace zeta::protocol