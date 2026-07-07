// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file lsm6dsow_regmap.h
 * @brief LSM6DSOW 寄存器映射（I2C）
 */

#pragma once

#include "drivers/lsm6dsow/lsm6dsow_types.h"
#include "error.h"
#include "regmap/reg_i2c.h"
#include "regmap/regmap_base.h"

namespace zeta::drivers::LSM6DSOW {

/**
 * @brief LSM6DSOW控制器类
 */
class Regmap : public regmap::Regmap<regmap::RegI2C> {
 public:
  Error Init(TwoWire* wire, const int address);
  Error ReadAcceleration(float& x, float& y, float& z);
  bool  AccelerationAvailable();
  Error ReadGyroscope(float& x, float& y, float& z);
  bool  GyroscopeAvailable();
  Error ReadTemperature(float& temperature_deg);
  bool  TemperatureAvailable();
};

}  // namespace zeta::drivers::LSM6DSOW

namespace zeta::drivers::LSM6DSOW {

inline Error Regmap::Init(TwoWire* wire, const int address) {
  CHECK(transport_.Init(wire, address));

  uint8_t value;
  using kWHO_AM_I = LSM6DSOWRegs::kWHO_AM_I;
  CHECK(ReadField<kWHO_AM_I>(value));
  VERIFY(value == 0x6C, Error::kErr);

  using kCTRL2_G = LSM6DSOWRegs::kCTRL2_G;
  CHECK(WriteField<kCTRL2_G>(0x4C));

  using kCTRL1_XL = LSM6DSOWRegs::kCTRL1_XL;
  CHECK(WriteField<kCTRL1_XL>(0x4A));

  using kCTRL7_G = LSM6DSOWRegs::kCTRL7_G;
  CHECK(WriteField<kCTRL7_G>(0x00));

  using kCTRL8_XL = LSM6DSOWRegs::kCTRL8_XL;
  CHECK(WriteField<kCTRL8_XL>(0x09));

  return Error::kOk;
}

inline Error Regmap::ReadAcceleration(float& x, float& y, float& z) {
  const uint8_t size = 6;
  uint8_t       data[size];
  using kOUTX_A = LSM6DSOWRegs::kOUTX_A;
  CHECK(Read(kOUTX_A::kAddress, size, data));
  const int16_t x_raw = static_cast<int16_t>(data[1] << 8 | data[0]);
  const int16_t y_raw = static_cast<int16_t>(data[3] << 8 | data[2]);
  const int16_t z_raw = static_cast<int16_t>(data[5] << 8 | data[4]);
  x                   = static_cast<float>(x_raw * 4.0 / 32768.0);
  y                   = static_cast<float>(y_raw * 4.0 / 32768.0);
  z                   = static_cast<float>(z_raw * 4.0 / 32768.0);
  return Error::kOk;
}

inline bool Regmap::AccelerationAvailable() {
  uint8_t value;
  using kXLDA     = LSM6DSOWRegs::kXLDA;
  const Error err = ReadField<kXLDA>(value);
  if (err != Error::kOk) {
    return false;
  }
  return value & 0x01;
}

inline Error Regmap::ReadGyroscope(float& x, float& y, float& z) {
  const uint8_t size = 6;
  uint8_t       data[size];
  using kOUTX_L_G = LSM6DSOWRegs::kOUTX_L_G;
  CHECK(Read(kOUTX_L_G::kAddress, size, data));
  const int16_t x_raw = static_cast<int16_t>(data[1] << 8 | data[0]);
  const int16_t y_raw = static_cast<int16_t>(data[3] << 8 | data[2]);
  const int16_t z_raw = static_cast<int16_t>(data[5] << 8 | data[4]);
  x                   = static_cast<float>(x_raw * 2000.0 / 32768.0);
  y                   = static_cast<float>(y_raw * 2000.0 / 32768.0);
  z                   = static_cast<float>(z_raw * 2000.0 / 32768.0);
  return Error::kOk;
}

inline bool Regmap::GyroscopeAvailable() {
  uint8_t value;
  using kGDA      = LSM6DSOWRegs::kGDA;
  const Error err = ReadField<kGDA>(value);
  if (err != Error::kOk) {
    return false;
  }
  return value & 0x02;
}

inline Error Regmap::ReadTemperature(float& temperature_deg) {
  const uint8_t size = 2;
  uint8_t       data[size];
  using kOUT_TEMP = LSM6DSOWRegs::kOUT_TEMP;
  CHECK(Read(kOUT_TEMP::kAddress, size, data));
  const int16_t value = static_cast<int16_t>(data[1] << 8 | data[0]);
  temperature_deg     = (static_cast<float>(value) / 256) + 25;
  return Error::kOk;
}

inline bool Regmap::TemperatureAvailable() {
  uint8_t value;
  using kTDA      = LSM6DSOWRegs::kTDA;
  const Error err = ReadField<kTDA>(value);
  if (err != Error::kOk) {
    return false;
  }
  return value & 0x04;
}

}  // namespace zeta::drivers::LSM6DSOW
