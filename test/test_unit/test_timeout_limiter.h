// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* TimeoutLimiter 单元测试。覆盖初始/设置、Process 累加与清零、Reset。 */

#include <unity.h>

#include "utils/timeout_limiter.h"

namespace TimeoutLimiterTest {

using Limiter = zeta::utils::TimeoutLimiter;

// 验证初始状态与 setter/getter。
void test_initial_and_setters(void) {
  Limiter l;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, l.threshold());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, l.timeout_duration());

  l.set_threshold(1.0f);
  l.set_timeout_duration(0.5f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, l.threshold());
  TEST_ASSERT_EQUAL_FLOAT(0.5f, l.timeout_duration());
}

// 验证当前值未超阈值时 Process 不触发超时。
void test_process_below_threshold(void) {
  Limiter l;
  l.set_threshold(10.0f);
  l.set_timeout_duration(1.0f);

  TEST_ASSERT_FALSE(l.Process(5.0f, 0.2f));
  TEST_ASSERT_FALSE(l.Process(5.0f, 0.2f));
  TEST_ASSERT_FALSE(l.Process(10.0f, 0.2f));  // 等于阈值不视为超过
}

// 验证超过阈值后累加 dt，达到 timeout_duration 时返回 true。
void test_process_above_threshold_accumulates(void) {
  Limiter l;
  l.set_threshold(0.0f);
  l.set_timeout_duration(0.5f);

  TEST_ASSERT_FALSE(l.Process(1.0f, 0.2f));
  TEST_ASSERT_FALSE(l.Process(1.0f, 0.2f));
  TEST_ASSERT_TRUE(l.Process(1.0f, 0.2f));  // 0.2+0.2+0.2 >= 0.5
}

// 验证超过阈值后再次低于阈值则累加时间清零。
void test_process_drop_below_resets_accumulator(void) {
  Limiter l;
  l.set_threshold(10.0f);
  l.set_timeout_duration(1.0f);

  l.Process(20.0f, 0.4f);                     // exceeded 0.4s
  l.Process(5.0f, 0.1f);                      // drop below, accumulator reset
  TEST_ASSERT_FALSE(l.Process(20.0f, 0.3f));  // only 0.3s again
  TEST_ASSERT_TRUE(l.Process(20.0f, 0.8f));   // 0.3+0.8 >= 1.0
}

// 验证 Reset 后累加时间归零，行为与初始一致。
void test_reset(void) {
  Limiter l;
  l.set_threshold(0.0f);
  l.set_timeout_duration(1.0f);
  l.Process(1.0f, 0.6f);
  l.Reset();
  TEST_ASSERT_FALSE(l.Process(1.0f, 0.4f));
  TEST_ASSERT_TRUE(l.Process(1.0f, 0.6f));
}

void run_tests(void) {
  RUN_TEST(TimeoutLimiterTest::test_initial_and_setters);
  RUN_TEST(TimeoutLimiterTest::test_process_below_threshold);
  RUN_TEST(TimeoutLimiterTest::test_process_above_threshold_accumulates);
  RUN_TEST(TimeoutLimiterTest::test_process_drop_below_resets_accumulator);
  RUN_TEST(TimeoutLimiterTest::test_reset);
}

}  // namespace TimeoutLimiterTest
