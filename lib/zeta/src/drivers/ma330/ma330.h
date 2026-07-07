// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ma330.h
 * @brief MA330 磁编码器（SPI，14 位）
 */

#pragma once

#include <SPI.h>

#include "drivers/ma330/ma330_regmap.h"
#include "error.h"
#include "servo/encoder.h"

namespace zeta::drivers::MA330 {
constexpr uint8_t kResolutionBits = 14;

class Encoder;
using Base = servo::Encoder<Encoder, kResolutionBits>;

/// @brief MA330 磁编码器实现（SPI，14 位）
class Encoder : public Base {
 public:
  static constexpr uint8_t kResolutionBits = zeta::drivers::MA330::kResolutionBits;
  /// @brief 配置：SPI 与片选
  struct Config : public Base::Config {
    SPIClass* spi;
    uint8_t   cs_pin;
  };

  /**
   * @brief 初始化 SPI 与编码器
   * @param config 配置（SPI 与片选）
   * @return 错误码
   */
  Error   Init(const Config& config);
  Error   ReadRawImpl(uint32_t& out_raw);
  Regmap* regmap();

 private:
  Regmap regmap_;
};

}  // namespace zeta::drivers::MA330

namespace zeta::drivers::MA330 {

inline Error Encoder::Init(const Config& config) {
  CHECK(regmap_.Init(config.spi, config.cs_pin, SPISettings(1000000, MSBFIRST, SPI_MODE3)));
  CHECK(Base::Init(config));
  return Error::kOk;
}

inline Error Encoder::ReadRawImpl(uint32_t& out_raw) {
  uint16_t raw;
  CHECK(regmap_.ReadRaw(raw));
  out_raw = static_cast<uint32_t>(raw);
  return Error::kOk;
}

inline Regmap* Encoder::regmap() {
  return &regmap_;
}

}  // namespace zeta::drivers::MA330
