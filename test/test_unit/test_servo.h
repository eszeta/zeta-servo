// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <unity.h>

#include "error.h"
#include "servo/servo.h"
#include "servo/servo_types.h"
#include "simulator/current.h"
#include "simulator/encoder.h"
#include "simulator/motor.h"

namespace ServoTest {

using Motor   = zeta::simulator::SimulatorMotor;
using Encoder = zeta::simulator::SimulatorEncoder;
using Current = zeta::simulator::SimulatorCurrent;
using Servo   = zeta::servo::Servo<Motor, Encoder, Current, zeta::simulator::kSimEncoderBits>;
using EncoderConfig = Encoder::Config;
using Error         = zeta::Error;
using OperatingMode = zeta::servo::OperatingMode;
using Reverse       = zeta::servo::Reverse;

static constexpr float kDt = 0.01f;

#define ASSERT_OK(expr) TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(expr))

struct Fixture {
  Motor   motor;
  Encoder encoder;
  Current current;
  Servo   servo;

  void Init() {
    ASSERT_OK(encoder.Init(EncoderConfig{0, Reverse::kNormal}));
    ASSERT_OK(servo.Init(&motor, &encoder, &current));
    servo.set_current_limit(10.0f);
    servo.set_protection_time(0.3f);
    servo.set_pwm_limit(1.0f);
  }
};

void test_position_pwm_limit_uses_normalized_output(void) {
  Fixture f;
  f.Init();
  f.servo.set_position_pid(10.0f, 0.0f, 0.0f);
  f.servo.set_profile_velocity(0.0f);
  f.servo.set_goal_position(1000);
  f.servo.set_torque_enable(true);

  ASSERT_OK(f.servo.Process(kDt));

  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, f.motor.last_pwm());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, f.servo.present_pwm());
}

void test_pwm_mode_uses_normalized_goal_pwm(void) {
  Fixture f;
  f.Init();
  f.servo.set_operating_mode(OperatingMode::kPwm);
  f.servo.set_goal_pwm(0.5f);
  f.servo.set_torque_enable(true);

  ASSERT_OK(f.servo.Process(kDt));

  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, f.motor.last_pwm());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, f.servo.present_pwm());
}

void test_profile_velocity_change_rebuilds_current_profile(void) {
  Fixture f;
  f.Init();
  f.servo.set_position_pid(0.0f, 0.0f, 0.0f);
  f.servo.set_profile_acceleration(0.0f);
  f.servo.set_profile_velocity(60.0f);
  f.servo.set_goal_position(1000);
  f.servo.set_torque_enable(true);

  ASSERT_OK(f.servo.Process(kDt));
  const int32_t first_traj = f.servo.position_trajectory();

  f.servo.set_profile_velocity(30.0f);
  ASSERT_OK(f.servo.Process(kDt));
  const int32_t rebuilt_traj = f.servo.position_trajectory();

  TEST_ASSERT_TRUE(first_traj > 0);
  TEST_ASSERT_TRUE(rebuilt_traj > first_traj);
  TEST_ASSERT_TRUE((rebuilt_traj - first_traj) < first_traj);
}

void test_position_mode_limits_goal_position(void) {
  Fixture f;
  f.Init();
  f.servo.set_position_pid(0.0f, 0.0f, 0.0f);
  f.servo.set_profile_velocity(0.0f);
  f.servo.set_min_position_limit(100);
  f.servo.set_max_position_limit(200);
  f.servo.set_goal_position(1000);
  f.servo.set_torque_enable(true);

  ASSERT_OK(f.servo.Process(kDt));

  TEST_ASSERT_EQUAL_INT32(200, f.servo.position_trajectory());
}

void test_extended_position_ignores_single_turn_limits(void) {
  Fixture f;
  f.Init();
  f.servo.set_operating_mode(OperatingMode::kExtendedPosition);
  f.servo.set_position_pid(0.0f, 0.0f, 0.0f);
  f.servo.set_profile_velocity(0.0f);
  f.servo.set_min_position_limit(100);
  f.servo.set_max_position_limit(200);
  f.servo.set_goal_position(1000);
  f.servo.set_torque_enable(true);

  ASSERT_OK(f.servo.Process(kDt));

  TEST_ASSERT_EQUAL_INT32(1000, f.servo.position_trajectory());
}

void test_in_position_uses_pulse_tolerance_not_velocity_threshold(void) {
  Fixture f;
  f.Init();
  f.servo.set_moving_threshold(2.29f);
  f.servo.set_position_pid(0.0f, 0.0f, 0.0f);
  f.servo.set_profile_velocity(0.0f);
  f.servo.set_goal_position(2);
  f.servo.set_torque_enable(true);

  ASSERT_OK(f.servo.Process(kDt));

  TEST_ASSERT_FALSE(f.servo.moving_in_position());
  TEST_ASSERT_TRUE(f.servo.moving_following_error());
}

void test_following_error_tracks_position_trajectory_error_while_profile_runs(void) {
  Fixture f;
  f.Init();
  f.servo.set_position_pid(0.0f, 0.0f, 0.0f);
  f.servo.set_profile_acceleration(0.0f);
  f.servo.set_profile_velocity(60.0f);
  f.servo.set_goal_position(1000);
  f.servo.set_torque_enable(true);

  ASSERT_OK(f.servo.Process(kDt));

  TEST_ASSERT_TRUE(f.servo.moving_profile_ongoing());
  TEST_ASSERT_TRUE(f.servo.position_trajectory() > 1);
  TEST_ASSERT_TRUE(f.servo.moving_following_error());
}

void test_unsupported_mode_coasts_and_returns_error(void) {
  Fixture f;
  f.Init();
  f.motor.SetPWM(0.75f);
  f.servo.set_present_pwm(0.75f);
  f.servo.set_operating_mode(OperatingMode::kVelocity);
  f.servo.set_torque_enable(true);

  const auto err = f.servo.Process(kDt);

  TEST_ASSERT_EQUAL(static_cast<int>(Error::kUnsupported), static_cast<int>(err));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, f.motor.last_pwm());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, f.servo.present_pwm());
}

void test_unimplemented_current_position_mode_coasts_and_returns_error(void) {
  Fixture f;
  f.Init();
  f.motor.SetPWM(0.75f);
  f.servo.set_present_pwm(0.75f);
  f.servo.set_operating_mode(OperatingMode::kCurrentPosition);
  f.servo.set_torque_enable(true);

  const auto err = f.servo.Process(kDt);

  TEST_ASSERT_EQUAL(static_cast<int>(Error::kUnsupported), static_cast<int>(err));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, f.motor.last_pwm());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, f.servo.present_pwm());
}

void run_tests(void) {
  RUN_TEST(ServoTest::test_position_pwm_limit_uses_normalized_output);
  RUN_TEST(ServoTest::test_pwm_mode_uses_normalized_goal_pwm);
  RUN_TEST(ServoTest::test_profile_velocity_change_rebuilds_current_profile);
  RUN_TEST(ServoTest::test_position_mode_limits_goal_position);
  RUN_TEST(ServoTest::test_extended_position_ignores_single_turn_limits);
  RUN_TEST(ServoTest::test_in_position_uses_pulse_tolerance_not_velocity_threshold);
  RUN_TEST(ServoTest::test_following_error_tracks_position_trajectory_error_while_profile_runs);
  RUN_TEST(ServoTest::test_unsupported_mode_coasts_and_returns_error);
  RUN_TEST(ServoTest::test_unimplemented_current_position_mode_coasts_and_returns_error);
}

}  // namespace ServoTest

#undef ASSERT_OK
