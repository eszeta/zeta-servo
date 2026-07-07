// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file mt6701_regmap.h
 * @brief MT6701 寄存器映射（I2C/SPI 特化）
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "drivers/mt6701/mt6701_types.h"
#include "error.h"
#include "regmap/reg_i2c.h"
#include "regmap/reg_spi.h"

namespace zeta::drivers::MT6701 {

template <BusType Bus>
class RegmapBase;

template <>
class RegmapBase<BusType::kI2C> : public zeta::regmap::Regmap<zeta::regmap::RegI2C> {
 public:
  Error Init(TwoWire* wire, const int address);
  Error ReadRaw(uint16_t& angle_raw, Status& field_status, bool& button_pushed, bool& track_loss);
};

template <>
class RegmapBase<BusType::kSPI> : public regmap::Regmap<regmap::RegSPI> {
 public:
  Error Init(SPIClass* spi, int cs_pin, const SPISettings& spi_settings);
  Error ReadRaw(uint16_t& angle_raw, Status& field_status, bool& button_pushed, bool& track_loss);
  Error Write(const uint8_t address, const uint8_t data);
  Error Write(const uint8_t address, const uint8_t* data, const size_t size);
  Error Read(const uint8_t address, uint8_t* data);
  Error Read(const uint8_t address, const size_t size, uint8_t* data);
};

template <BusType Bus>
class Regmap : public RegmapBase<Bus> {
 public:
  Error ReadFieldStatus(Status& status);
  Error WriteUvmMode(const uint8_t pairs);
  Error WriteAbzMode(const uint16_t   pulses_per_round,
                     const PulseWidth z_pulse_width,
                     const Hyst       hysteresis);
  Error WriteNaNbNzEnable(bool enable);
  Error WriteAnalogMode(const float start = 0.0f, const float stop = 360.0f);
  Error WritePwmMode(const PwmFreq frequency = PwmFreq::kPWMFreq497_2,
                     const PwmPol  polarity  = PwmPol::kHigh);
  Error WriteDirection(const Direction direction);
  Error GetDirection(Direction& direction);
  Error ProgramEEPROM();
  Error WriteABZPulsePerRound(uint16_t pulses);
  Error WriteUVWPolePair(uint8_t pairs);
  Error WriteMode(Mode mode);
  Error WriteZeroRaw(uint16_t zero);
  Error WriteZero(float zero);
  Error WriteHyst(Hyst hysteresis);
  Error WriteStartStopRaw(uint16_t start, uint16_t stop);
  Error WriteStartStop(float start, float stop);
  Error WritePulseWidth(PulseWidth width);
  Error ReadPulseWidth(PulseWidth& width);
  Error WritePwmFreq(PwmFreq freq);
  Error ReadPwmFreq(PwmFreq& freq);
  Error WritePwmPolarity(PwmPol polarity);
  Error ReadPwmPolarity(PwmPol& polarity);
  Error WriteOutMode(const OutMode mode);
  Error ReadOutMode(OutMode& mode);
};

}  // namespace zeta::drivers::MT6701

namespace zeta::drivers::MT6701 {

inline Error RegmapBase<BusType::kI2C>::Init(TwoWire* wire, const int address) {
  CHECK(transport_.Init(wire, address));
  return Error::kOk;
}

inline Error RegmapBase<BusType::kI2C>::ReadRaw(uint16_t& angle_raw,
                                                Status&   field_status,
                                                bool&     button_pushed,
                                                bool&     track_loss) {
  using kANGLE_6 = MT6701Regs::kANGLE_6;
  using kANGLE_0 = MT6701Regs::kANGLE_0;
  CHECK(ReadField<uint16_t, kANGLE_6, kANGLE_0>(angle_raw));
  (void)field_status;
  (void)button_pushed;
  (void)track_loss;
  return Error::kOk;
}

inline Error RegmapBase<BusType::kSPI>::Init(SPIClass*          spi,
                                             int                cs_pin,
                                             const SPISettings& spi_settings) {
  CHECK(transport_.Init(spi, cs_pin, spi_settings));
  return Error::kOk;
}

inline Error RegmapBase<BusType::kSPI>::ReadRaw(uint16_t& angle_raw,
                                                Status&   field_status,
                                                bool&     button_pushed,
                                                bool&     track_loss) {
  SPIClass*         spi          = transport_.spi();
  const int         cs_pin       = transport_.cs_pin();
  const SPISettings spi_settings = transport_.spi_settings();
  VERIFY(spi, Error::kInvalidArg);

  uint8_t data[3];
  digitalWrite(cs_pin, LOW);
  spi->beginTransaction(spi_settings);

  data[0] = spi->transfer(0xFF);
  data[1] = spi->transfer(0xFF);
  data[2] = spi->transfer(0xFF);

  spi->endTransaction();
  digitalWrite(cs_pin, HIGH);

  struct {
    uint16_t angle  : 14;
    uint8_t  status : 2;
    uint8_t  button : 1;
    uint8_t  track  : 1;
  } __attribute__((packed))* raw = reinterpret_cast<decltype(raw)>(data);

  angle_raw     = raw->angle;
  field_status  = static_cast<Status>(raw->status);
  button_pushed = raw->button;
  track_loss    = raw->track;

  return Error::kOk;
}

inline Error RegmapBase<BusType::kSPI>::Write(const uint8_t address, const uint8_t data) {
  return Error::kErr;
}

inline Error RegmapBase<BusType::kSPI>::Write(const uint8_t  address,
                                              const uint8_t* data,
                                              const size_t   size) {
  return Error::kErr;
}

inline Error RegmapBase<BusType::kSPI>::Read(const uint8_t address, uint8_t* data) {
  return Error::kErr;
}

inline Error RegmapBase<BusType::kSPI>::Read(const uint8_t address,
                                             const size_t  size,
                                             uint8_t*      data) {
  return Error::kErr;
}

template <BusType Bus>
Error Regmap<Bus>::ReadFieldStatus(Status& status) {
  uint16_t angle_raw     = 0;
  bool     button_pushed = false;
  bool     track_loss    = false;
  CHECK(this->ReadRaw(angle_raw, status, button_pushed, track_loss));
  (void)angle_raw;
  (void)button_pushed;
  (void)track_loss;
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WriteUvmMode(const uint8_t pairs) {
  CHECK(WriteUVWPolePair(pairs));
  CHECK(WriteMode(Mode::kUVW));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WriteAbzMode(const uint16_t   pulses_per_round,
                                const PulseWidth z_pulse_width,
                                const Hyst       hysteresis) {
  CHECK(WritePulseWidth(z_pulse_width));
  CHECK(WriteHyst(hysteresis));
  CHECK(WriteABZPulsePerRound(pulses_per_round));
  CHECK(WriteMode(Mode::kABZ));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WriteNaNbNzEnable(bool enable) {
  CHECK(this->template WriteField<MT6701Regs::kUVM_MUX>(enable));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WriteAnalogMode(const float start, const float stop) {
  CHECK(WriteStartStop(start, stop));
  CHECK(WriteOutMode(OutMode::kAnalog));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WritePwmMode(const PwmFreq frequency, const PwmPol polarity) {
  CHECK(WriteOutMode(OutMode::kPWM));
  CHECK(WritePwmFreq(frequency));
  CHECK(WritePwmPolarity(polarity));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WriteDirection(const Direction direction) {
  CHECK(this->template WriteField<MT6701Regs::kDIR>(static_cast<uint8_t>(direction)));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::GetDirection(Direction& direction) {
  uint8_t value;
  CHECK(this->template ReadField<MT6701Regs::kDIR>(value));
  direction = static_cast<Direction>(value);
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::ProgramEEPROM() {
  CHECK(this->Write(0x09, 0xB3));
  CHECK(this->Write(0x0A, 0x05));
  delay(600);
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WriteABZPulsePerRound(uint16_t pulses) {
  pulses--;
  VERIFY(pulses < 1024, Error::kOutOfRange);
  using kABZ_RES_8 = MT6701Regs::kABZ_RES_8;
  using kABZ_RES_0 = MT6701Regs::kABZ_RES_0;
  CHECK(this->template WriteField<uint16_t, kABZ_RES_8, kABZ_RES_0>(pulses));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WriteUVWPolePair(uint8_t pairs) {
  pairs--;
  VERIFY(pairs < 16, Error::kOutOfRange);
  using kUVM_RES_0 = MT6701Regs::kUVM_RES_0;
  CHECK(this->template WriteField<kUVM_RES_0>(pairs));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WriteMode(Mode mode) {
  using kABZ_MUX = MT6701Regs::kABZ_MUX;
  CHECK(this->template WriteField<kABZ_MUX>(static_cast<uint8_t>(mode)));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WriteZeroRaw(uint16_t zero) {
  using kZERO_8 = MT6701Regs::kZERO_8;
  using kZERO_0 = MT6701Regs::kZERO_0;
  CHECK(this->template WriteField<uint16_t, kZERO_8, kZERO_0>(zero));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WriteZero(float zero) {
  return WriteZeroRaw(static_cast<uint16_t>(zero * 4096 / 360.0f));
}

template <BusType Bus>
Error Regmap<Bus>::WriteHyst(Hyst hysteresis) {
  using kHYST_2 = MT6701Regs::kHYST_2;
  using kHYST_0 = MT6701Regs::kHYST_0;
  CHECK(this->template WriteField<uint8_t, kHYST_2, kHYST_0>(static_cast<uint8_t>(hysteresis)));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WriteStartStopRaw(uint16_t start, uint16_t stop) {
  VERIFY(start < 4096 && stop < 4096, Error::kOutOfRange);
  VERIFY(stop > start, Error::kInvalidArg);
  using kA_START_8 = MT6701Regs::kA_START_8;
  using kA_START_0 = MT6701Regs::kA_START_0;
  using kA_STOP_8  = MT6701Regs::kA_STOP_8;
  using kA_STOP_0  = MT6701Regs::kA_STOP_0;
  CHECK(this->template WriteField<uint16_t, kA_START_8, kA_START_0>(start));
  CHECK(this->template WriteField<uint16_t, kA_STOP_8, kA_STOP_0>(stop));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WriteStartStop(float start, float stop) {
  uint16_t start_u16 = static_cast<uint16_t>(start * 4096 / 360.0f);
  uint16_t stop_u16  = static_cast<uint16_t>(stop * 4096 / 360.0f);
  start_u16          = start_u16 >= 4096 ? 4095 : start_u16;
  stop_u16           = stop_u16 >= 4096 ? 4095 : stop_u16;
  return WriteStartStopRaw(start_u16, stop_u16);
}

template <BusType Bus>
Error Regmap<Bus>::WritePulseWidth(PulseWidth width) {
  using kPULSE_WIDTH = MT6701Regs::kPULSE_WIDTH;
  CHECK(this->template WriteField<kPULSE_WIDTH>(static_cast<uint8_t>(width)));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::ReadPulseWidth(PulseWidth& width) {
  uint8_t value;
  using kPULSE_WIDTH = MT6701Regs::kPULSE_WIDTH;
  CHECK(this->template ReadField<kPULSE_WIDTH>(value));
  width = static_cast<PulseWidth>(value);
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WritePwmFreq(PwmFreq freq) {
  using kPWM_FREQ = MT6701Regs::kPWM_FREQ;
  CHECK(this->template WriteField<kPWM_FREQ>(static_cast<uint8_t>(freq)));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::ReadPwmFreq(PwmFreq& freq) {
  uint8_t value;
  using kPWM_FREQ = MT6701Regs::kPWM_FREQ;
  CHECK(this->template ReadField<kPWM_FREQ>(value));
  freq = static_cast<PwmFreq>(value);
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WritePwmPolarity(PwmPol polarity) {
  using kPWM_POL = MT6701Regs::kPWM_POL;
  CHECK(this->template WriteField<kPWM_POL>(static_cast<uint8_t>(polarity)));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::ReadPwmPolarity(PwmPol& polarity) {
  uint8_t value;
  using kPWM_POL = MT6701Regs::kPWM_POL;
  CHECK(this->template ReadField<kPWM_POL>(value));
  polarity = static_cast<PwmPol>(value);
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::WriteOutMode(const OutMode mode) {
  using kOUT_MODE = MT6701Regs::kOUT_MODE;
  CHECK(this->template WriteField<kOUT_MODE>(static_cast<uint8_t>(mode)));
  return Error::kOk;
}

template <BusType Bus>
Error Regmap<Bus>::ReadOutMode(OutMode& mode) {
  uint8_t value;
  using kOUT_MODE = MT6701Regs::kOUT_MODE;
  CHECK(this->template ReadField<kOUT_MODE>(value));
  mode = static_cast<OutMode>(value);
  return Error::kOk;
}

}  // namespace zeta::drivers::MT6701
