// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* regmap::reg_field 单元测试。覆盖 Trait::SignExtend、Field GetValue/SetValue/Clear、
 * Converter/RatioConverter、Merged2。 */

#include <unity.h>

#include "regmap/reg_field.h"

namespace RegFieldTest {

namespace reg = zeta::regmap;

// 验证 Trait::SignExtend 按位宽做符号扩展。
void test_trait_sign_extend(void) {
  TEST_ASSERT_EQUAL_INT16(0, (reg::Trait::SignExtend<int16_t, 4>(0)));
  TEST_ASSERT_EQUAL_INT16(7, (reg::Trait::SignExtend<int16_t, 4>(7)));
  TEST_ASSERT_EQUAL_INT16(-1, (reg::Trait::SignExtend<int16_t, 4>(0xFu)));
  TEST_ASSERT_EQUAL_INT16(-8, (reg::Trait::SignExtend<int16_t, 4>(0x8u)));
  TEST_ASSERT_EQUAL_INT32(-1, (reg::Trait::SignExtend<int32_t, 12>(0xFFFu)));
}

using FieldU8  = reg::Field<uint8_t, 0, 2, 4>;
using FieldS12 = reg::Field<int16_t, 0, 0, 12>;

// 验证无符号 Field 的 GetValue/SetValue/Clear。
void test_field_unsigned_get_set_clear(void) {
  FieldU8::access_t data = 0xFFu;
  TEST_ASSERT_EQUAL_UINT8(0x0Fu, FieldU8::GetValue(data));
  FieldU8::SetValue(0x05u, data);
  TEST_ASSERT_EQUAL_UINT8(0xD7u, data);
  TEST_ASSERT_EQUAL_UINT8(0x05u, FieldU8::GetValue(data));
  FieldU8::Clear(data);
  TEST_ASSERT_EQUAL_UINT8(0xC3u, data);
}

// 验证有符号 Field 的 GetValue/SetValue 与符号扩展。
void test_field_signed_get_set(void) {
  FieldS12::access_t data = 0;
  FieldS12::SetValue(2047, data);
  TEST_ASSERT_EQUAL_INT16(2047, FieldS12::GetValue(data));
  FieldS12::SetValue(-1, data);
  TEST_ASSERT_EQUAL_INT16(-1, FieldS12::GetValue(data));
  FieldS12::SetValue(-2048, data);
  TEST_ASSERT_EQUAL_INT16(-2048, FieldS12::GetValue(data));
}

// 验证 Converter FromRaw/ToRaw 往返一致。
void test_converter_roundtrip(void) {
  TEST_ASSERT_EQUAL_INT16(100, reg::Converter<int16_t>::FromRaw<uint16_t>(100));
  TEST_ASSERT_EQUAL_UINT16(100, reg::Converter<int16_t>::ToRaw<uint16_t>(100));
}

// 验证 RatioConverter 比例换算与往返。
void test_ratio_converter(void) {
  using RC = reg::RatioConverter<int32_t, 360, 4096>;
  TEST_ASSERT_EQUAL_INT32(0, RC::FromRaw<uint16_t>(0));
  int32_t v = RC::FromRaw<uint16_t>(4096);
  TEST_ASSERT_EQUAL_INT32(360, v);
  TEST_ASSERT_EQUAL_UINT16(4096, RC::ToRaw<uint16_t>(360));
}

using High4   = reg::Field<uint8_t, 0, 4, 4>;
using Low4    = reg::Field<uint8_t, 0, 0, 4>;
using Merged8 = reg::Merged2<uint8_t, High4, Low4>;

// 验证 Merged2 高低段拼接后 GetValue/SetValue 一致。
void test_merged2_get_set(void) {
  High4::access_t high = 0;
  Low4::access_t  low  = 0;
  Merged8::SetValue(0xA5u, high, low);
  TEST_ASSERT_EQUAL_UINT8(0xA5u, Merged8::GetValue(high, low));
  TEST_ASSERT_EQUAL_UINT8(0xA0u, high);
  TEST_ASSERT_EQUAL_UINT8(0x05u, low);
}

void run_tests(void) {
  RUN_TEST(RegFieldTest::test_trait_sign_extend);
  RUN_TEST(RegFieldTest::test_field_unsigned_get_set_clear);
  RUN_TEST(RegFieldTest::test_field_signed_get_set);
  RUN_TEST(RegFieldTest::test_converter_roundtrip);
  RUN_TEST(RegFieldTest::test_ratio_converter);
  RUN_TEST(RegFieldTest::test_merged2_get_set);
}

}  // namespace RegFieldTest
