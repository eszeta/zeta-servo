// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file reg_spi.h
 * @brief SPI 寄存器传输层（Regmap 后端）
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "error.h"
#include "noncopyable.h"
#include "regmap/regmap_base.h"

namespace zeta::regmap {

/**
 * @brief SPI 寄存器读写，用于 Regmap 的远程表（如 MT6701 SPI）
 */
class RegSPI : public zeta::Noncopyable {
 public:
  /** @brief SPI 实例 */
  SPIClass* spi();
  /** @brief 片选引脚 */
  int cs_pin() const;
  /** @brief SPI 时序配置 */
  SPISettings spi_settings() const;

  /**
   * @brief 绑定 SPI 与片选
   * @param spi SPI 实例
   * @param cs_pin 片选引脚（<0 表示无片选）
   * @param spi_settings 时序参数
   * @return 错误码
   */
  Error Init(SPIClass* spi, int cs_pin, const SPISettings& spi_settings);
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
  SPIClass*   spi_{};         ///< SPI 实例
  int         cs_pin_{};      ///< 片选引脚
  SPISettings spi_settings_;  ///< 时序配置
};

}  // namespace zeta::regmap

namespace zeta::regmap {

inline SPIClass* RegSPI::spi() {
  return spi_;
}

inline int RegSPI::cs_pin() const {
  return cs_pin_;
}

inline SPISettings RegSPI::spi_settings() const {
  return spi_settings_;
}

inline Error RegSPI::Init(SPIClass* spi, int cs_pin, const SPISettings& spi_settings) {
  VERIFY(spi, Error::kInvalidArg);
  spi_          = spi;
  cs_pin_       = cs_pin;
  spi_settings_ = spi_settings;
  if (cs_pin_ >= 0) {
    pinMode(cs_pin_, OUTPUT);
    digitalWrite(cs_pin_, HIGH);
  }
  return Error::kOk;
}

inline Error RegSPI::Write(const uint8_t address, const uint8_t* data, const size_t size) {
  VERIFY(spi_ && data, Error::kInvalidArg);
  spi_->beginTransaction(spi_settings_);
  if (cs_pin_ >= 0)
    digitalWrite(cs_pin_, LOW);
  spi_->transfer(address);
  spi_->transfer(const_cast<uint8_t*>(data), size);
  if (cs_pin_ >= 0)
    digitalWrite(cs_pin_, HIGH);
  spi_->endTransaction();
  return Error::kOk;
}

inline Error RegSPI::Read(const uint8_t address, const size_t size, uint8_t* data) {
  VERIFY(spi_ && data, Error::kInvalidArg);
  spi_->beginTransaction(spi_settings_);
  if (cs_pin_ >= 0)
    digitalWrite(cs_pin_, LOW);
  spi_->transfer(0x80 | address);
  spi_->transfer(data, size);
  if (cs_pin_ >= 0)
    digitalWrite(cs_pin_, HIGH);
  spi_->endTransaction();
  return Error::kOk;
}

}  // namespace zeta::regmap
