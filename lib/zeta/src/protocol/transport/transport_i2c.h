// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file transport_i2c.h
 * @brief I2C 从机传输层（读 Wire 缓冲、OnRequest 发送）
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "error.h"
#include "protocol/transport/transport.h"

namespace zeta::protocol {

/**
 * @brief I2C 从机传输层
 *
 * RX 直读 Wire 内置缓冲（应用层须注册 onReceive/onRequest）。
 * Write 挂指针，OnRequest 时发送。
 */
class TransportI2C : public Transport<TransportI2C> {
 public:
  /**
   * @brief 初始化，绑定 I2C 总线
   * @param wire TwoWire 实例（如 &Wire）
   * @return 错误码
   */
  Error Init(TwoWire* wire);

  bool   ReadByteImpl(uint8_t& byte);
  size_t AvailableImpl();
  Error  WriteImpl(const uint8_t* data, const size_t size);

  /** @brief I2C 从机发送回调实现，写出 tx_data_ */
  void OnRequestImpl();

  /** @brief I2C 从机接收回调实现（收包靠 wire->available 判断） */
  void OnReceiveImpl(int n) { (void)n; }

 private:
  TwoWire* wire_ = nullptr;

  const uint8_t* tx_data_ = nullptr;
  size_t         tx_size_ = 0;
};

}  // namespace zeta::protocol

namespace zeta::protocol {

inline Error TransportI2C::Init(TwoWire* wire) {
  wire_ = wire;
  return Error::kOk;
}

inline bool TransportI2C::ReadByteImpl(uint8_t& byte) {
  if (wire_ == nullptr || wire_->available() == 0) {
    return false;
  }
  byte = static_cast<uint8_t>(wire_->read());
  return true;
}

inline size_t TransportI2C::AvailableImpl() {
  return wire_ != nullptr ? static_cast<size_t>(wire_->available()) : 0;
}

inline Error TransportI2C::WriteImpl(const uint8_t* data, const size_t size) {
  tx_data_ = data;
  tx_size_ = size;
  return Error::kOk;
}

inline void TransportI2C::OnRequestImpl() {
  if (wire_ != nullptr && tx_data_ != nullptr && tx_size_ > 0) {
    wire_->write(tx_data_, tx_size_);
  }
}

}  // namespace zeta::protocol