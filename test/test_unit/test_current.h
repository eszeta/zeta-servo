// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* SimulatorCurrent 单元测试。覆盖初始化、设定值与回读一致性及未设定时的默认零值。 */

#include <unity.h>

#include "error.h"
#include "simulator/current.h"

namespace CurrentTest {

using Current = zeta::simulator::SimulatorCurrent;
using Error   = zeta::Error;

void test_init_returns_ok(void) {
  Current    c;
  const auto err = c.Init();
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
}

// 验证 SetCurrent 设定后 ReadCurrent 回读与设定值一致（容差 1e-6）。
void test_set_current_read_back(void) {
  Current c;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(c.Init()));
  c.SetCurrent(1.5f);
  float      out = 0.0f;
  const auto err = c.ReadCurrent(out);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f, out);
}

void test_initial_read_zero(void) {
  Current c;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(c.Init()));
  float      out = -1.0f;
  const auto err = c.ReadCurrent(out);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out);
}

// 验证先设非零再设 0 时，回读为 0，即最后一次设定生效。
void test_set_current_zero_read_back(void) {
  Current c;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(c.Init()));
  c.SetCurrent(2.0f);
  c.SetCurrent(0.0f);
  float      out = 1.0f;
  const auto err = c.ReadCurrent(out);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out);
}

void run_tests(void) {
  RUN_TEST(CurrentTest::test_init_returns_ok);
  RUN_TEST(CurrentTest::test_set_current_read_back);
  RUN_TEST(CurrentTest::test_initial_read_zero);
  RUN_TEST(CurrentTest::test_set_current_zero_read_back);
}

}  // namespace CurrentTest
