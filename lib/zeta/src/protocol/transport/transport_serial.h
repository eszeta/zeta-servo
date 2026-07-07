// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file transport_serial.h
 * @brief 串口传输层（直接读写 Serial，无缓冲）
 */

#pragma once

#include <Arduino.h>

#include "error.h"
#include "protocol/transport/transport.h"

namespace zeta::protocol {

/**
 * @brief 串口传输层，响应延迟由 ProtocolChannel 处理
 */
class TransportSerial : public Transport<TransportSerial> {
 public:
  /**
   * @brief 初始化，绑定串口
   * @param serial HardwareSerial 实例（如 &Serial1）
   * @return 错误码
   */
  Error Init(HardwareSerial* serial) {
    serial_ = serial;
    return Error::kOk;
  }

  bool   ReadByteImpl(uint8_t& byte);
  size_t AvailableImpl();
  Error  WriteImpl(const uint8_t* data, const size_t size);

  /** @brief 串口不需要请求发送回调 */
  void OnRequestImpl() {}
  /** @brief 串口不需要接收中断回调 */
  void OnReceiveImpl(int n) { (void)n; }

 private:
  HardwareSerial* serial_ = nullptr;
};

}  // namespace zeta::protocol

namespace zeta::protocol {

inline bool TransportSerial::ReadByteImpl(uint8_t& byte) {
  if (serial_ == nullptr || serial_->available() == 0) {
    return false;
  }
  byte = static_cast<uint8_t>(serial_->read());
  return true;
}

inline size_t TransportSerial::AvailableImpl() {
  return serial_ != nullptr ? static_cast<size_t>(serial_->available()) : 0;
}

inline Error TransportSerial::WriteImpl(const uint8_t* data, const size_t size) {
  if (serial_ == nullptr || data == nullptr) {
    return Error::kInvalidArg;
  }
  const size_t written = serial_->write(data, size);
  return written == size ? Error::kOk : Error::kIO;
}

}  // namespace zeta::protocol
