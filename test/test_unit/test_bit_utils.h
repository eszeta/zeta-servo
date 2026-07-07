// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* bit_utils 单元测试。覆盖 GetValue/IsMaskSet/IsBitSet、Set/Clear/Toggle、
 * CreateMask、CombineToUint16/Int16、TwosToSign/SignToTwos。 */

#include <unity.h>

#include "utils/bit_utils.h"

namespace BitUtilsTest {

namespace bu = zeta::utils::bit_utils;

// 验证 GetValue 按掩码提取值。
void test_get_value(void) {
  TEST_ASSERT_EQUAL_UINT32(0, bu::GetValue(0xFFu, 0u));
  TEST_ASSERT_EQUAL_UINT32(0x0F, bu::GetValue(0xFFu, 0x0Fu));
  TEST_ASSERT_EQUAL_UINT32(0x0F, bu::GetValue(0x3Fu, 0x0Fu));
  TEST_ASSERT_EQUAL_UINT8(0x04,
                          bu::GetValue(static_cast<uint8_t>(0x0F), static_cast<uint8_t>(0x04)));
}

// 验证 IsMaskSet 判断掩码位是否置位。
void test_is_mask_set(void) {
  TEST_ASSERT_FALSE(bu::IsMaskSet(0u, 1u));
  TEST_ASSERT_TRUE(bu::IsMaskSet(0xFFu, 0x80u));
  TEST_ASSERT_TRUE(bu::IsMaskSet(0x03u, 0x02u));
  TEST_ASSERT_FALSE(bu::IsMaskSet(0x01u, 0x02u));
}

// 验证 IsBitSet 判断特定位是否置位。
void test_is_bit_set(void) {
  TEST_ASSERT_FALSE(bu::IsBitSet(0u, 0));
  TEST_ASSERT_TRUE(bu::IsBitSet(1u, 0));
  TEST_ASSERT_TRUE(bu::IsBitSet(0x80u, 7));
  TEST_ASSERT_FALSE(bu::IsBitSet(0x7Fu, 7));
}

// 验证 SetMaskBit/ClearMaskBit/ToggleMaskBit 按掩码设置、清除、翻转。
void test_set_clear_toggle_mask(void) {
  uint8_t x = 0;
  x         = bu::SetMaskBit(x, static_cast<uint8_t>(0x0Cu));
  TEST_ASSERT_EQUAL_UINT8(0x0C, x);
  x = bu::ClearMaskBit(x, static_cast<uint8_t>(0x08u));
  TEST_ASSERT_EQUAL_UINT8(0x04, x);
  x = bu::ToggleMaskBit(x, static_cast<uint8_t>(0x0Fu));
  TEST_ASSERT_EQUAL_UINT8(0x0Bu, x);
}

// 验证 SetBit/ClearBit/ToggleBit 按位索引设置、清除、翻转。
void test_set_clear_toggle_bit(void) {
  uint16_t x = 0;
  x          = bu::SetBit(x, 5);
  TEST_ASSERT_EQUAL_UINT16(32, x);
  x = bu::ClearBit(x, 5);
  TEST_ASSERT_EQUAL_UINT16(0, x);
  x = bu::SetBit(x, 0);
  x = bu::ToggleBit(x, 0);
  TEST_ASSERT_EQUAL_UINT16(0, x);
}

// 验证 CreateMask 生成指定位宽与起始位的掩码。
void test_create_mask(void) {
  TEST_ASSERT_EQUAL_UINT32(0xFu, bu::CreateMask(0, 4));
  TEST_ASSERT_EQUAL_UINT32(0xF0u, bu::CreateMask(4, 4));
  TEST_ASSERT_EQUAL_UINT32(0xFFu, bu::CreateMask(0, 8));
  TEST_ASSERT_EQUAL_UINT32(0x3FF00u, bu::CreateMask(8, 10));
}

// 验证 CombineToUint16/CombineToInt16 高低字节拼接与符号。
void test_combine_uint16_int16(void) {
  TEST_ASSERT_EQUAL_UINT16(0x1234, bu::CombineToUint16(0x12, 0x34));
  TEST_ASSERT_EQUAL_UINT16(0, bu::CombineToUint16(0, 0));
  TEST_ASSERT_EQUAL_INT16(-1, bu::CombineToInt16(0xFF, 0xFF));
  TEST_ASSERT_EQUAL_INT16(0x7FFF, bu::CombineToInt16(0x7F, 0xFF));
}

// 验证 TwosToSign 补码转原码。
void test_twos_to_sign(void) {
  TEST_ASSERT_EQUAL_UINT16(0, bu::TwosToSign(0));
  TEST_ASSERT_EQUAL_UINT16(100, bu::TwosToSign(100));
  TEST_ASSERT_EQUAL_UINT16(0x8001, bu::TwosToSign(-1));
  TEST_ASSERT_TRUE(bu::IsBitSet(bu::TwosToSign(-1), 15));
  TEST_ASSERT_EQUAL_UINT16(1, bu::ClearBit(bu::TwosToSign(-1), 15));
}

// 验证 SignToTwos 原码转补码。
void test_sign_to_twos(void) {
  TEST_ASSERT_EQUAL_INT16(0, bu::SignToTwos(0));
  TEST_ASSERT_EQUAL_INT16(100, bu::SignToTwos(100));
  TEST_ASSERT_EQUAL_INT16(-1, bu::SignToTwos(0x8001));  // 原码 -1
  TEST_ASSERT_EQUAL_INT16(0, bu::SignToTwos(0x8000));   // 原码 -0 => 补码 0
}

void run_tests(void) {
  RUN_TEST(BitUtilsTest::test_get_value);
  RUN_TEST(BitUtilsTest::test_is_mask_set);
  RUN_TEST(BitUtilsTest::test_is_bit_set);
  RUN_TEST(BitUtilsTest::test_set_clear_toggle_mask);
  RUN_TEST(BitUtilsTest::test_set_clear_toggle_bit);
  RUN_TEST(BitUtilsTest::test_create_mask);
  RUN_TEST(BitUtilsTest::test_combine_uint16_int16);
  RUN_TEST(BitUtilsTest::test_twos_to_sign);
  RUN_TEST(BitUtilsTest::test_sign_to_twos);
}

}  // namespace BitUtilsTest
