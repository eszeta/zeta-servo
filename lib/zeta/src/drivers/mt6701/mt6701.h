// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file mt6701.h
 * @brief MT6701 磁编码器（I2C/SPI，14 位）
 */

#pragma once

#include <SPI.h>
#include <Wire.h>

#include "drivers/mt6701/mt6701_regmap.h"
#include "drivers/mt6701/mt6701_types.h"
#include "servo/encoder.h"
#include "servo/servo_types.h"

namespace zeta::drivers::MT6701 {
constexpr uint8_t kResolutionBits = 14;

/**
 * @brief MT6701磁性角度传感器通用模板声明
 *
 * 使用BusType枚举作为模板参数，通过模板偏特化为不同总线类型提供实现。
 * 支持I2C和SPI两种通信方式，每种方式有独立的Init方法签名。
 */
template <BusType Bus>
class Encoder;

/**
 * @brief MT6701磁性角度传感器私有基类
 *
 * 包含所有与总线无关的公共代码，避免在特化中重复实现。
 * 使用CRTP模式继承servo::Encoder，提供统一的传感器接口。
 */
template <BusType Bus>
class EncoderBase;

template <BusType Bus>
class EncoderBase : public servo::Encoder<EncoderBase<Bus>, kResolutionBits> {
 public:
  static constexpr uint8_t kResolutionBits = zeta::drivers::MT6701::kResolutionBits;
  /**
   * @brief 获取控制器实例
   * @return 控制器实例引用
   *
   * 返回控制器实例的引用，用于直接操作传感器的配置和状态。
   */
  Regmap<Bus>& regmap() { return regmap_; }

  /**
   * @brief 获取原始角度值
   * @param out_raw 输出的原始角度值（14位分辨率，范围0-16383）
   * @return 错误码，成功返回OK
   *
   * 通过总线接口读取MT6701传感器的当前角度值。
   * 该方法实现了基类的纯虚函数。
   */
  Error ReadRawImpl(uint32_t& out_raw);

 protected:
  /// @brief 寄存器映射实例，负责与传感器的具体通信操作
  Regmap<Bus> regmap_;
};

/**
 * @brief MT6701磁性角度传感器I2C实现
 *
 * 该类封装了MT6701磁性角度传感器的I2C通信功能，继承自私有基类。
 * MT6701是一款高精度、低功耗的磁性角度传感器，提供14位分辨率的角度测量。
 * 本特化使用I2C接口与传感器通信，支持角度读取和状态查询。
 */

using I2CBase = EncoderBase<BusType::kI2C>;
template <>
class Encoder<BusType::kI2C> : public I2CBase {
 public:
  struct Config : public I2CBase::Config {
    TwoWire* wire;
  };

  Error Init(const Config& config) {
    CHECK(regmap_.Init(config.wire, kI2CAddress));
    CHECK(I2CBase::Init(config));
    return Error::kOk;
  }
};

/**
 * @brief MT6701磁性角度传感器SPI实现
 *
 * 该类封装了MT6701磁性角度传感器的SPI通信功能，继承自私有基类。
 * MT6701是一款高精度、低功耗的磁性角度传感器，提供14位分辨率的角度测量。
 * 本特化使用SPI接口与传感器通信，支持角度读取和状态查询。
 */
using SPIBase = EncoderBase<BusType::kSPI>;
template <>
class Encoder<BusType::kSPI> : public SPIBase {
 public:
  struct Config : public SPIBase::Config {
    SPIClass*   spi;
    int         cs_pin;
    SPISettings spi_settings;
  };

  Error Init(const Config& config) {
    CHECK(regmap_.Init(config.spi, config.cs_pin, config.spi_settings));
    CHECK(SPIBase::Init(config));
    return Error::kOk;
  }
};

}  // namespace zeta::drivers::MT6701

namespace zeta::drivers::MT6701 {

template <BusType Bus>
Error EncoderBase<Bus>::ReadRawImpl(uint32_t& out_raw) {
  uint16_t raw;
  Status   status        = Status::kNormal;
  bool     button_pushed = false;
  bool     track_loss    = false;
  CHECK(regmap_.ReadRaw(raw, status, button_pushed, track_loss));
  out_raw = static_cast<uint32_t>(raw);
  (void)status;
  (void)button_pushed;
  (void)track_loss;
  return Error::kOk;
}

}  // namespace zeta::drivers::MT6701