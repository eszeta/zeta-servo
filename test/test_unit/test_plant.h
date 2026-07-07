// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* SimulatorPlant 单元测试。包含空指针安全、PWM 符号与位置/电流方向、电流/位置与 PWM 比例、CPR 内回绕。 */

#include <unity.h>

#include "error.h"
#include "servo/servo_types.h"
#include "simulator/current.h"
#include "simulator/encoder.h"
#include "simulator/motor.h"
#include "simulator/plant.h"

namespace PlantTest {

using Motor   = zeta::simulator::SimulatorMotor;
using Encoder = zeta::simulator::SimulatorEncoder;
using Current = zeta::simulator::SimulatorCurrent;
using Plant   = zeta::simulator::SimulatorPlant;
using Error   = zeta::Error;
using Config  = Encoder::Config;
using Reverse = zeta::servo::Reverse;

constexpr uint32_t kCpr = (1U << zeta::simulator::kSimEncoderBits);

// 验证未挂接 motor/encoder/current 时 Process 返回 kInvalidArg。
void test_process_with_nullptrs_returns_error(void) {
  Plant      p;
  const auto err = p.Process(0.001f);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kInvalidArg), static_cast<int>(err));
}

// 验证 PWM 为正时位置增加且电流为正，为负时电流为负，为 0 时位置与电流不变。
void test_pwm_sign_matches_position_and_current_direction(void) {
  Motor   motor;
  Encoder encoder;
  Current current;
  Plant   plant;

  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(motor.Init()));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(encoder.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(current.Init()));
  plant.set_motor(&motor);
  plant.set_encoder(&encoder);
  plant.set_current_sensor(&current);

  motor.SetPWM(1.0f);
  (void)plant.Process(0.001f);
  (void)encoder.Process(0.001f);
  TEST_ASSERT_TRUE(encoder.raw_pos() > 0);
  float cur = 0.0f;
  current.ReadCurrent(cur);
  TEST_ASSERT_TRUE(cur > 0.0f);

  motor.SetPWM(-1.0f);
  for (int i = 0; i < 100; ++i) {
    (void)plant.Process(0.001f);
    (void)encoder.Process(0.001f);
  }
  current.ReadCurrent(cur);
  TEST_ASSERT_TRUE(cur < 0.0f);

  motor.SetPWM(0.0f);
  const uint32_t raw_at_zero = encoder.raw_pos();
  (void)plant.Process(0.001f);
  (void)encoder.Process(0.001f);
  TEST_ASSERT_EQUAL_UINT32(raw_at_zero, encoder.raw_pos());
  current.ReadCurrent(cur);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, cur);
}

// 验证同一 PWM 下电流与位置变化率成比例。
void test_current_and_position_proportional_to_pwm(void) {
  Motor   motor;
  Encoder encoder;
  Current current;
  Plant   plant;

  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(motor.Init()));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(encoder.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(current.Init()));
  plant.set_motor(&motor);
  plant.set_encoder(&encoder);
  plant.set_current_sensor(&current);

  const int steps = 100;
  motor.SetPWM(1.0f);
  for (int i = 0; i < steps; ++i) {
    (void)plant.Process(0.001f);
    (void)encoder.Process(0.001f);
  }
  const uint32_t raw_1 = encoder.raw_pos();
  float          cur_1 = 0.0f;
  current.ReadCurrent(cur_1);

  motor.SetPWM(0.5f);
  for (int i = 0; i < steps; ++i) {
    (void)plant.Process(0.001f);
    (void)encoder.Process(0.001f);
  }
  const uint32_t raw_2 = encoder.raw_pos();
  float          cur_2 = 0.0f;
  current.ReadCurrent(cur_2);

  TEST_ASSERT_TRUE(raw_1 > 0);
  const uint32_t delta_raw = (raw_2 >= raw_1) ? (raw_2 - raw_1) : (raw_2 + (kCpr - raw_1));
  const float    pos_ratio = static_cast<float>(delta_raw) / static_cast<float>(raw_1);
  const float    cur_ratio = (cur_1 != 0.0f) ? (cur_2 / cur_1) : 0.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.5f, pos_ratio);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.5f, cur_ratio);
}

// 验证多步后 raw_pos 始终在 [0, CPR-1]。
void test_process_multi_step_position_wraps_within_cpr(void) {
  Motor   motor;
  Encoder encoder;
  Current current;
  Plant   plant;

  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(motor.Init()));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(encoder.Init(Config{0, Reverse::kNormal})));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(current.Init()));
  plant.set_motor(&motor);
  plant.set_encoder(&encoder);
  plant.set_current_sensor(&current);

  motor.SetPWM(1.0f);
  const int total_steps = static_cast<int>(kCpr) * 2 + 500;
  for (int i = 0; i < total_steps; ++i) {
    const auto err = plant.Process(0.001f);
    TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
    (void)encoder.Process(0.001f);
    TEST_ASSERT_TRUE(encoder.raw_pos() < kCpr);
  }
  TEST_ASSERT_TRUE(encoder.raw_pos() < kCpr);
}

void run_tests(void) {
  RUN_TEST(PlantTest::test_process_with_nullptrs_returns_error);
  RUN_TEST(PlantTest::test_pwm_sign_matches_position_and_current_direction);
  RUN_TEST(PlantTest::test_current_and_position_proportional_to_pwm);
  RUN_TEST(PlantTest::test_process_multi_step_position_wraps_within_cpr);
}

}  // namespace PlantTest
