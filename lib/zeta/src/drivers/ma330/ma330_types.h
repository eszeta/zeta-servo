// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ma330_types.h
 * @brief MA330 寄存器字段与枚举
 */

#pragma once

#include "regmap/reg_field.h"

namespace zeta::drivers::MA330 {

enum class FieldStrength : uint8_t {
  kNormal = 0x00,
  kLow    = 0x01,
  kHigh   = 0x02,
  kErr    = 0x03  // impossible state
};

namespace MA330Regs {
template <uint8_t Address, uint8_t Shift, uint8_t Bits>
using FieldU08 = regmap::Field<uint8_t, Address, Shift, Bits>;

template <uint8_t Address, uint8_t Shift, uint8_t Bits>
using FieldB08 = regmap::Field<bool, Address, Shift, Bits>;

struct kZ_L : FieldU08<0x00, 0, 8> {};
struct kZ_H : FieldU08<0x01, 0, 8> {};
struct kBCT : FieldU08<0x02, 0, 8> {};
struct kETX : FieldB08<0x03, 0, 1> {};
struct kETY : FieldB08<0x03, 1, 1> {};
struct kILIP : FieldU08<0x04, 2, 4> {};
struct kPPT_L : FieldU08<0x04, 6, 2> {};
struct kPPT_H : FieldU08<0x05, 0, 8> {};
struct kMGHT : FieldU08<0x06, 2, 3> {};
struct kMGLT : FieldU08<0x06, 5, 3> {};
struct kNPP : FieldU08<0x07, 5, 3> {};
struct kRD : FieldU08<0x09, 7, 1> {};
struct kFW : FieldU08<0x0E, 0, 8> {};
struct kHYS : FieldU08<0x10, 0, 8> {};
struct kMGL_MGH : FieldU08<0x1B, 6, 2> {};
};  // namespace MA330Regs

}  // namespace zeta::drivers::MA330
