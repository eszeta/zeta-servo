// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* Profile 单元测试。覆盖 SetGoal Step/Rect、Process 推进与完成、梯形阶段。 */

#include <unity.h>

#include "math/profile.h"

namespace ProfileTest {

using Profile = zeta::math::Profile;
using Type    = zeta::math::Profile::Type;

static constexpr uint16_t kCpr             = 4096;
static constexpr int      kMaxRectSteps    = 500;   // 步数足够 Rect 轮廓到达 goal
static constexpr int      kMaxProfileSteps = 2000;  // 步数足够梯形/三角形轮廓到达 goal

// 验证 profile_velocity_rpm==0 时 Step 立即完成且位置等于 goal。
void test_set_goal_step_zero_velocity(void) {
  Profile p(Profile::Config{kCpr});
  p.SetGoal(0.0f, 100, 0.0f, 0.0f);
  TEST_ASSERT_EQUAL(static_cast<uint8_t>(Type::kStep), static_cast<uint8_t>(p.type()));
  TEST_ASSERT_TRUE(p.is_complete());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 100.0f, p.position_trajectory());
  TEST_ASSERT_EQUAL_INT32(100, p.goal());
}

// 验证位移不足半脉冲时 Step 立即完成。
void test_set_goal_step_small_delta(void) {
  Profile p(Profile::Config{kCpr});
  p.SetGoal(0.0f, 0, 100.0f, 100.0f);
  TEST_ASSERT_TRUE(p.is_complete());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, p.position_trajectory());
}

// 验证 Rect 轮廓位置线性增加，结束时 is_complete 且到达 goal、速度为 0。
void test_rect_position_linear_then_complete(void) {
  Profile p(Profile::Config{kCpr});
  p.SetGoal(0.0f, 1000, 60.0f, 0.0f);
  TEST_ASSERT_EQUAL(static_cast<uint8_t>(Type::kRect), static_cast<uint8_t>(p.type()));
  TEST_ASSERT_FALSE(p.is_complete());

  float       prev_pos = p.position_trajectory();
  const float dt       = 0.01f;
  for (int i = 0; i < kMaxRectSteps; ++i) {
    p.Process(dt);
    if (p.is_complete()) {
      TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1000.0f, p.position_trajectory());
      TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, p.velocity_trajectory_cps());
      break;
    }
    float pos = p.position_trajectory();
    TEST_ASSERT_TRUE(pos >= prev_pos - 0.1f);
    prev_pos = pos;
  }
  TEST_ASSERT_TRUE(p.is_complete());
}

void test_position_trajectory_preserves_subpulse_motion(void) {
  Profile p(Profile::Config{kCpr});
  p.SetGoal(0.0f, 100, 1.0f, 0.0f);

  p.Process(0.001f);

  TEST_ASSERT_EQUAL_INT32(0, static_cast<int32_t>(p.position_trajectory()));
  TEST_ASSERT_TRUE(p.position_trajectory() > 0.0f);
}

// 验证带加速度轮廓 Process 多步后完成并到达 goal。
void test_profile_process_reaches_goal(void) {
  Profile p(Profile::Config{kCpr});
  p.SetGoal(0.0f, 5000, 120.0f, 1000.0f);

  const float dt = 0.005f;
  for (int i = 0; i < kMaxProfileSteps && !p.is_complete(); ++i) {
    p.Process(dt);
  }
  TEST_ASSERT_TRUE(p.is_complete());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5000.0f, p.position_trajectory());
  TEST_ASSERT_TRUE(p.type() == Type::kTrapezoidal || p.type() == Type::kTriangular);
}

void run_tests(void) {
  RUN_TEST(ProfileTest::test_set_goal_step_zero_velocity);
  RUN_TEST(ProfileTest::test_set_goal_step_small_delta);
  RUN_TEST(ProfileTest::test_rect_position_linear_then_complete);
  RUN_TEST(ProfileTest::test_position_trajectory_preserves_subpulse_motion);
  RUN_TEST(ProfileTest::test_profile_process_reaches_goal);
}

}  // namespace ProfileTest
