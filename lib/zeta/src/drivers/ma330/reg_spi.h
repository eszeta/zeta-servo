// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file reg_spi.h
 * @brief MA330 SPI 传输层（角度读取）
 */

#pragma once

#include <Arduino.h>

#include "regmap/reg_spi.h"

namespace zeta::drivers::MA330 {

/**
 * @brief SPI通信实现
 *
 * 实现通过SPI协议与MA330传感器通信的功能。
 * SPI模式提供更高的通信速度，但功能有限，主要用于角度读取。
 */
class RegSPI : public regmap::RegSPI {
 public:
  Error Write(const uint8_t address, const uint8_t* data, const size_t size);
  Error Read(const uint8_t address, const size_t size, uint8_t* data);
  Error ReadRaw(uint16_t& angle_raw);

 private:
  uint16_t transfer16(uint16_t outValue);
};

}  // namespace zeta::drivers::MA330

namespace zeta::drivers::MA330 {

inline Error RegSPI::Write(const uint8_t address, const uint8_t* data, const size_t size) {
  VERIFY(data, Error::kInvalidArg);
  for (size_t i = 0; i < size; ++i) {
    uint16_t cmd = 0x8000 | (((address + i) & 0x1F) << 8) | data[i];
    transfer16(cmd);
    delay(20);  // 20ms delay required per MA330 spec
    transfer16(0x0000);
  }
  return Error::kOk;
}

inline Error RegSPI::Read(const uint8_t address, const size_t size, uint8_t* data) {
  VERIFY(data, Error::kInvalidArg);
  for (size_t i = 0; i < size; ++i) {
    uint16_t cmd   = 0x4000 | (((address + i) & 0x001F) << 8);
    uint16_t value = transfer16(cmd);
    delayMicroseconds(1);
    value   = transfer16(0x0000);
    data[i] = value >> 8;
  }
  return Error::kOk;
}

inline Error RegSPI::ReadRaw(uint16_t& angle_raw) {
  angle_raw = transfer16(0x0000);
  return Error::kOk;
}

inline uint16_t RegSPI::transfer16(uint16_t outValue) {
  spi_->beginTransaction(spi_settings_);
  if (cs_pin_ >= 0)
    digitalWrite(cs_pin_, LOW);
  uint16_t value = spi_->transfer16(outValue);
  if (cs_pin_ >= 0)
    digitalWrite(cs_pin_, HIGH);
  spi_->endTransaction();
  return value;
}

}  // namespace zeta::drivers::MA330
