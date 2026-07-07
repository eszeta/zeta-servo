// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file reg_i2c.h
 * @brief I2C 寄存器传输层（Regmap 后端）
 */

#pragma once

#include <Wire.h>

#include "error.h"
#include "noncopyable.h"
#include "regmap/regmap_base.h"

namespace zeta::regmap {

/**
 * @brief I2C 寄存器读写，用于 Regmap 的远程表
 */
class RegI2C : public zeta::Noncopyable {
 public:
  /**
   * @brief 绑定 I2C 总线与从机地址
   * @param wire I2C 总线（如 &Wire）
   * @param address 从机 7 位地址
   * @return 错误码
   */
  Error Init(TwoWire* wire, const int address);
  /**
   * @brief 写连续寄存器
   * @param address 起始地址
   * @param data 数据指针
   * @param size 字节数
   * @return 错误码
   */
  Error Write(const uint8_t address, const uint8_t* data, const size_t size);
  /**
   * @brief 读连续寄存器
   * @param address 起始地址
   * @param size 字节数
   * @param data 输出缓冲区
   * @return 错误码
   */
  Error Read(const uint8_t address, const size_t size, uint8_t* data);

 protected:
  TwoWire* wire_{};     ///< I2C 总线
  int      address_{};  ///< 从机地址
};

}  // namespace zeta::regmap

namespace zeta::regmap {

inline Error RegI2C::Init(TwoWire* wire, const int address) {
  VERIFY(wire, Error::kInvalidArg);
  wire_    = wire;
  address_ = address;
  return Error::kOk;
}

inline Error RegI2C::Write(const uint8_t address, const uint8_t* data, const size_t size) {
  VERIFY(wire_ && data, Error::kInvalidArg);
  wire_->beginTransmission(address_);
  wire_->write(address);
  wire_->write(data, size);
  if (wire_->endTransmission() != 0) {
    return Error::kIO;
  }
  return Error::kOk;
}

inline Error RegI2C::Read(const uint8_t address, const size_t size, uint8_t* data) {
  VERIFY(wire_ && data, Error::kInvalidArg);
  wire_->beginTransmission(address_);
  wire_->write(address);
  wire_->endTransmission(false);
  wire_->requestFrom(address_, size);
  for (size_t i = 0; i < size; i++) {
    if (wire_->available()) {
      data[i] = wire_->read();
    } else {
      return Error::kIO;
    }
  }
  return Error::kOk;
}

}  // namespace zeta::regmap
