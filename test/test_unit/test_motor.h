// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* SimulatorMotor 单元测试。包含初始 last_pwm、SetPWM 正负与满幅、Brake/Coast 清零、Brake 后 SetPWM 恢复驱动。 */

#include <unity.h>

#include "error.h"
#include "simulator/motor.h"

namespace MotorTest {

using Motor = zeta::simulator::SimulatorMotor;
using Error = zeta::Error;

void test_init_returns_ok(void) {
  Motor      m;
  const auto err = m.Init();
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
}

// 验证默认构造后 last_pwm 为 0。
void test_initial_last_pwm_zero(void) {
  Motor m;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(m.Init()));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.last_pwm());
}

// 验证 SetPWM 正数时 last_pwm 与设定一致。
void test_set_pwm_positive(void) {
  Motor m;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(m.Init()));
  m.SetPWM(0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.last_pwm());
}

// 验证 SetPWM 负数时 last_pwm 与设定一致。
void test_set_pwm_negative(void) {
  Motor m;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(m.Init()));
  m.SetPWM(-1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, m.last_pwm());
}

// 验证 SetPWM(1.0f) 时 last_pwm 为 1。
void test_set_pwm_full_positive(void) {
  Motor m;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(m.Init()));
  m.SetPWM(1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.last_pwm());
}

// 验证 Brake 后 last_pwm 置 0。
void test_brake_zeros_pwm(void) {
  Motor m;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(m.Init()));
  m.SetPWM(0.7f);
  m.Brake();
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.last_pwm());
}

// 验证 Coast 后 last_pwm 置 0。
void test_coast_zeros_pwm(void) {
  Motor m;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(m.Init()));
  m.SetPWM(-0.5f);
  m.Coast();
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.last_pwm());
}

// 验证 Brake 后再 SetPWM，last_pwm 为设定值，电机可恢复驱动。
void test_brake_then_set_pwm_restores_drive(void) {
  Motor m;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(m.Init()));
  m.SetPWM(0.7f);
  m.Brake();
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.last_pwm());
  m.SetPWM(0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.last_pwm());
}

void run_tests(void) {
  RUN_TEST(MotorTest::test_init_returns_ok);
  RUN_TEST(MotorTest::test_initial_last_pwm_zero);
  RUN_TEST(MotorTest::test_set_pwm_positive);
  RUN_TEST(MotorTest::test_set_pwm_negative);
  RUN_TEST(MotorTest::test_set_pwm_full_positive);
  RUN_TEST(MotorTest::test_brake_zeros_pwm);
  RUN_TEST(MotorTest::test_coast_zeros_pwm);
  RUN_TEST(MotorTest::test_brake_then_set_pwm_restores_drive);
}

}  // namespace MotorTest
