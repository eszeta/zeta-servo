// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* math 工具函数单元测试。覆盖 mapResolution、mod、fmodf_pos、wrap_pm。 */

#include <unity.h>

#include "math/math.h"

namespace MathTest {

namespace m = zeta::math;

// 验证 mapResolution（max-based）：PWM/DAC 等非回绕量。
void test_map_resolution(void) {
  TEST_ASSERT_EQUAL_UINT32(0, m::mapResolution<uint32_t>(0, 8, 12));
  TEST_ASSERT_EQUAL_UINT32(255, m::mapResolution<uint32_t>(255, 8, 8));
  TEST_ASSERT_EQUAL_UINT32(4095, m::mapResolution<uint32_t>(255, 8, 12));
  TEST_ASSERT_EQUAL_UINT32(255, m::mapResolution<uint32_t>(4095, 12, 8));
}

// 验证 mapResolutionCpr（CPR-based）：编码器位置等回绕量。
void test_map_resolution_cpr(void) {
  TEST_ASSERT_EQUAL_UINT32(0, m::mapResolutionCpr<uint32_t>(0, 8, 12));
  TEST_ASSERT_EQUAL_UINT32(255, m::mapResolutionCpr<uint32_t>(255, 8, 8));
  TEST_ASSERT_EQUAL_UINT32(4080, m::mapResolutionCpr<uint32_t>(255, 8, 12));
  TEST_ASSERT_EQUAL_UINT32(2048, m::mapResolutionCpr<uint32_t>(128, 8, 12));
  TEST_ASSERT_EQUAL_UINT32(255, m::mapResolutionCpr<uint32_t>(4080, 12, 8));
  TEST_ASSERT_EQUAL_UINT32(128, m::mapResolutionCpr<uint32_t>(2048, 12, 8));
  TEST_ASSERT_EQUAL_INT32(-25, m::mapResolutionCpr<int32_t>(-100, 14, 12));
}

// 验证 mod 结果始终在 [0, divisor)。
void test_mod(void) {
  TEST_ASSERT_EQUAL_INT(0, m::mod(0, 5));
  TEST_ASSERT_EQUAL_INT(2, m::mod(2, 5));
  TEST_ASSERT_EQUAL_INT(2, m::mod(-3, 5));
  TEST_ASSERT_EQUAL_INT(0, m::mod(-5, 5));
  TEST_ASSERT_EQUAL_INT(1, m::mod(1, 3));
  TEST_ASSERT_EQUAL_INT(2, m::mod(-1, 3));
}

// 验证 fmodf_pos 结果非负。
void test_fmodf_pos(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m::fmodf_pos(5.0f, 2.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m::fmodf_pos(4.0f, 2.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m::fmodf_pos(-3.0f, 2.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f, m::fmodf_pos(-2.5f, 2.0f));
}

// 验证 wrap_pm 将周期折叠到 [-y/2, y/2) 区间。
void test_wrap_pm(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, m::wrap_pm(0.0f, 360.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 90.0f, m::wrap_pm(90.0f, 360.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -90.0f, m::wrap_pm(270.0f, 360.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, m::wrap_pm(360.0f, 360.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -90.0f, m::wrap_pm(-90.0f, 360.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -90.0f, m::wrap_pm(-450.0f, 360.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -180.0f, m::wrap_pm(180.0f, 360.0f));

  TEST_ASSERT_EQUAL_INT(0, m::wrap_pm(0, 360));
  TEST_ASSERT_EQUAL_INT(90, m::wrap_pm(90, 360));
  TEST_ASSERT_EQUAL_INT(-90, m::wrap_pm(270, 360));
  TEST_ASSERT_EQUAL_INT(0, m::wrap_pm(360, 360));
  TEST_ASSERT_EQUAL_INT(-90, m::wrap_pm(-90, 360));
  TEST_ASSERT_EQUAL_INT(-90, m::wrap_pm(-450, 360));
  TEST_ASSERT_EQUAL_INT(-180, m::wrap_pm(180, 360));
}

void run_tests(void) {
  RUN_TEST(MathTest::test_map_resolution);
  RUN_TEST(MathTest::test_map_resolution_cpr);
  RUN_TEST(MathTest::test_mod);
  RUN_TEST(MathTest::test_fmodf_pos);
  RUN_TEST(MathTest::test_wrap_pm);
}

}  // namespace MathTest
