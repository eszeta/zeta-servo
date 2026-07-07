// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* LowPassFilter 单元测试。覆盖构造/getter、Compute(dt<=0 返回 x)、收敛、Reset。 */

#include <unity.h>

#include "math/lowpass_filter.h"

namespace LowPassFilterTest {

using Filter = zeta::math::LowPassFilter;

// 验证构造与 time_constant getter/setter。
void test_constructor_and_time_constant(void) {
  Filter f(0.2f);
  TEST_ASSERT_EQUAL_FLOAT(0.2f, f.time_constant());

  Filter f2(0.5f);
  f2.set_time_constant(0.1f);
  TEST_ASSERT_EQUAL_FLOAT(0.1f, f2.time_constant());
}

// 验证 set_time_constant(0) 或负数不更新，time_constant() 保持原值。
void test_set_time_constant_zero_or_negative_ignored(void) {
  Filter f(0.5f);
  f.set_time_constant(0.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, f.time_constant());
  f.set_time_constant(-0.1f);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, f.time_constant());
}

// 验证 dt<=0 时 Compute 直接返回输入 x。
void test_compute_zero_dt_returns_input(void) {
  Filter f(0.1f);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, f.Compute(5.0f, 0.0f));
  TEST_ASSERT_EQUAL_FLOAT(-3.0f, f.Compute(-3.0f, -0.01f));
}

// 验证阶跃输入下一阶滤波输出渐近于输入。
void test_compute_step_converges(void) {
  Filter      f(0.1f);
  const float dt = 0.01f;
  const float x  = 10.0f;
  float       y  = f.Compute(x, dt);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, (1.0f - 0.1f / (0.1f + dt)) * x, y);
  for (int i = 0; i < 50; ++i) {
    y = f.Compute(x, dt);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.5f, x, y);
}

// 验证 Reset 后下一次 Compute 行为与初态一致。
void test_reset(void) {
  Filter f(0.1f);
  f.Compute(10.0f, 0.01f);
  f.Reset();
  float y     = f.Compute(20.0f, 0.01f);
  float alpha = 0.1f / (0.1f + 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, (1.0f - alpha) * 20.0f, y);
}

void run_tests(void) {
  RUN_TEST(LowPassFilterTest::test_constructor_and_time_constant);
  RUN_TEST(LowPassFilterTest::test_set_time_constant_zero_or_negative_ignored);
  RUN_TEST(LowPassFilterTest::test_compute_zero_dt_returns_input);
  RUN_TEST(LowPassFilterTest::test_compute_step_converges);
  RUN_TEST(LowPassFilterTest::test_reset);
}

}  // namespace LowPassFilterTest
