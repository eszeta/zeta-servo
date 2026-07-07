// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* SimulatorEncoder 单元测试，按功能模块分组：
 * 1. 初始化与配置
 * 2. 位置处理与多圈
 * 3. 对齐功能
 * 4. 方向控制
 * 5. 零点偏移
 * 6. 原始位置管理
 */

#include <unity.h>

#include "error.h"
#include "servo/servo_types.h"
#include "simulator/encoder.h"

#define ASSERT_OK(expr) TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(expr))

namespace EncoderTest {

using Encoder = zeta::simulator::SimulatorEncoder;
using Config  = Encoder::Config;
using Error   = zeta::Error;
using Reverse = zeta::servo::Reverse;

constexpr uint32_t kCpr = Encoder::kResolution.kEncoderCpr;
constexpr float    kDt  = 0.001f;

// 辅助：从 raw=1 步进到 raw=0 完成一圈 CW（调用前需已 Process 建立基准）
inline void StepOneRevolutionCw(Encoder& enc) {
  for (uint32_t raw = 1; raw < kCpr; ++raw) {
    enc.SetRawPosition(static_cast<int32_t>(raw));
    enc.Process(kDt);
  }
  enc.SetRawPosition(0);
  enc.Process(kDt);
}

// ============================================================================
// 1. 初始化与配置（Init & Configuration）
// ============================================================================

void test_init_basic(void) {
  Encoder enc;
  Config  config{};
  config.homing_offset = 0;
  config.reverse       = Reverse::kNormal;
  ASSERT_OK(enc.Init(config));
}

void test_init_with_homing_offset(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{50, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_INT32(150, enc.pos());
  TEST_ASSERT_EQUAL_INT32(50, enc.homing_offset());
}

void test_init_with_reverse(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{0, Reverse::kReverse}));
  const int32_t expected_pos = static_cast<int32_t>(kCpr) - 100;
  TEST_ASSERT_EQUAL_INT32(expected_pos, enc.pos());
  TEST_ASSERT_EQUAL(static_cast<int>(Reverse::kReverse), static_cast<int>(enc.reverse()));
}

void test_init_boundary_zero(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_INT32(0, enc.pos());
  TEST_ASSERT_EQUAL_INT32(0, enc.revolutions());
}

void test_init_boundary_cpr_minus_one(void) {
  Encoder enc;
  enc.SetRawPosition(static_cast<int32_t>(kCpr - 1));
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_INT32(-1, enc.pos());
}

// offset + reverse 组合：raw=0, reverse 后 local=0, +offset(CPR-1) -> SnapNearZero -> -1
void test_init_combined_offset_and_reverse(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Init(Config{static_cast<int32_t>(kCpr - 1), Reverse::kReverse}));
  TEST_ASSERT_EQUAL_INT32(-1, enc.pos());
}

// SnapNearZero 临界值精确验证
void test_snap_near_zero_at_threshold(void) {
  const int32_t threshold = Encoder::kEdgeThreshold;

  Encoder enc_at;
  enc_at.SetRawPosition(static_cast<int32_t>(kCpr - threshold));
  ASSERT_OK(enc_at.Init(Config{0, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_INT32(-threshold, enc_at.pos());

  Encoder enc_below;
  enc_below.SetRawPosition(static_cast<int32_t>(kCpr - threshold - 1));
  ASSERT_OK(enc_below.Init(Config{0, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(kCpr - threshold - 1), enc_below.pos());
}

// 二次 Init 重置多圈状态
void test_reinit_resets_state(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  ASSERT_OK(enc.Process(kDt));
  StepOneRevolutionCw(enc);
  TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(kCpr), enc.pos());
  TEST_ASSERT_EQUAL_INT32(1, enc.revolutions());

  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_INT32(100, enc.pos());
  TEST_ASSERT_EQUAL_INT32(0, enc.revolutions());
}

// ============================================================================
// 2. 位置处理与多圈（Position Processing & Multi-turn）
// ============================================================================

// 顺时针跨 CPR 边界（CPR-1 -> 0）
void test_process_cross_cpr_cw(void) {
  Encoder enc;
  enc.SetRawPosition(kCpr - 1);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  ASSERT_OK(enc.Process(kDt));
  const int32_t pos_before = enc.pos();

  enc.SetRawPosition(0);
  ASSERT_OK(enc.Process(kDt));
  TEST_ASSERT_EQUAL_INT32(pos_before + 1, enc.pos());
}

// 逆时针跨 CPR 边界（0 -> CPR-1）
void test_process_cross_cpr_ccw(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  ASSERT_OK(enc.Process(kDt));
  const int32_t pos_before = enc.pos();

  enc.SetRawPosition(static_cast<int32_t>(kCpr - 1));
  ASSERT_OK(enc.Process(kDt));
  TEST_ASSERT_EQUAL_INT32(pos_before - 1, enc.pos());
}

// 顺时针一圈验证
void test_process_one_revolution_cw(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  ASSERT_OK(enc.Process(kDt));

  for (uint32_t raw = 1; raw < kCpr; ++raw) {
    enc.SetRawPosition(static_cast<int32_t>(raw));
    ASSERT_OK(enc.Process(kDt));
  }
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Process(kDt));

  TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(kCpr), enc.pos());
  TEST_ASSERT_EQUAL_INT32(1, enc.revolutions());
}

// 逆时针一圈验证
void test_process_one_revolution_ccw(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  ASSERT_OK(enc.Process(kDt));

  for (uint32_t i = 0; i < kCpr - 1; ++i) {
    enc.SetRawPosition(static_cast<int32_t>(kCpr - 1 - i));
    ASSERT_OK(enc.Process(kDt));
  }
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Process(kDt));

  TEST_ASSERT_EQUAL_INT32(-static_cast<int32_t>(kCpr), enc.pos());
  TEST_ASSERT_EQUAL_INT32(-1, enc.revolutions());
}

// 两圈验证
void test_process_two_revolutions(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  ASSERT_OK(enc.Process(kDt));

  for (uint32_t i = 0; i < 2; ++i) {
    for (uint32_t raw = 1; raw < kCpr; ++raw) {
      enc.SetRawPosition(static_cast<int32_t>(raw));
      ASSERT_OK(enc.Process(kDt));
    }
    enc.SetRawPosition(0);
    ASSERT_OK(enc.Process(kDt));
  }

  TEST_ASSERT_EQUAL_INT32(2 * static_cast<int32_t>(kCpr), enc.pos());
  TEST_ASSERT_EQUAL_INT32(2, enc.revolutions());
}

// revolutions() 用具体值断言
void test_process_revolutions_calculation(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Init(Config{100, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_INT32(100, enc.pos());
  TEST_ASSERT_EQUAL_INT32(0, enc.revolutions());

  enc.SetRawPosition(500);
  ASSERT_OK(enc.Process(kDt));
  TEST_ASSERT_EQUAL_INT32(600, enc.pos());
  TEST_ASSERT_EQUAL_INT32(0, enc.revolutions());
}

// Reverse 下位置处理
void test_process_reverse_direction(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Init(Config{0, Reverse::kReverse}));
  ASSERT_OK(enc.Process(kDt));

  for (uint32_t raw = 1; raw < kCpr; ++raw) {
    enc.SetRawPosition(static_cast<int32_t>(raw));
    ASSERT_OK(enc.Process(kDt));
  }
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Process(kDt));

  TEST_ASSERT_EQUAL_INT32(-static_cast<int32_t>(kCpr), enc.pos());
  TEST_ASSERT_EQUAL_INT32(-1, enc.revolutions());
}

// 验证 Process 的 wrap_pm 最短路径：raw 10→CPR-10 应走 -20 而非 +CPR-20
void test_process_shortest_wrap_distance(void) {
  Encoder enc;
  enc.SetRawPosition(10);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  ASSERT_OK(enc.Process(kDt));
  const int32_t pos_before = enc.pos();

  enc.SetRawPosition(static_cast<int32_t>(kCpr - 10));
  ASSERT_OK(enc.Process(kDt));
  TEST_ASSERT_EQUAL_INT32(pos_before - 20, enc.pos());
}

// Reverse 下跨 CPR 边界（硬件 CW: CPR-1 -> 0，pos 应减 1）
void test_process_cross_cpr_with_reverse(void) {
  Encoder enc;
  enc.SetRawPosition(static_cast<int32_t>(kCpr - 1));
  ASSERT_OK(enc.Init(Config{0, Reverse::kReverse}));
  ASSERT_OK(enc.Process(kDt));
  const int32_t pos_before = enc.pos();

  enc.SetRawPosition(0);
  ASSERT_OK(enc.Process(kDt));
  TEST_ASSERT_EQUAL_INT32(pos_before - 1, enc.pos());
}

// ============================================================================
// 3. 对齐功能（Alignment）
// ============================================================================

void test_align_to_zero(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  ASSERT_OK(enc.AlignToPosition(0));
  TEST_ASSERT_EQUAL_INT32(0, enc.pos());
}

void test_align_to_position(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  ASSERT_OK(enc.AlignToPosition(500));
  TEST_ASSERT_EQUAL_INT32(500, enc.pos());
}

void test_align_then_move(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  ASSERT_OK(enc.AlignToPosition(500));
  TEST_ASSERT_EQUAL_INT32(500, enc.pos());

  enc.SetRawPosition(200);
  ASSERT_OK(enc.Process(kDt));
  TEST_ASSERT_EQUAL_INT32(600, enc.pos());
}

void test_align_across_boundary(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  ASSERT_OK(enc.AlignToPosition(0));
  TEST_ASSERT_EQUAL_INT32(0, enc.pos());
  ASSERT_OK(enc.Process(kDt));
  TEST_ASSERT_EQUAL_INT32(0, enc.pos());

  ASSERT_OK(enc.AlignToPosition(500));
  TEST_ASSERT_EQUAL_INT32(500, enc.pos());
  ASSERT_OK(enc.Process(kDt));
  TEST_ASSERT_EQUAL_INT32(500, enc.pos());
}

// Reverse 下对齐及后续移动：raw +100 对应 pos -100
void test_align_with_reverse(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{0, Reverse::kReverse}));
  ASSERT_OK(enc.AlignToPosition(500));
  TEST_ASSERT_EQUAL_INT32(500, enc.pos());

  enc.SetRawPosition(200);
  ASSERT_OK(enc.Process(kDt));
  TEST_ASSERT_EQUAL_INT32(400, enc.pos());
}

// 从负 pos（SnapNearZero 产生）对齐到正目标后继续追踪
void test_align_from_negative_pos(void) {
  Encoder enc;
  enc.SetRawPosition(static_cast<int32_t>(kCpr - 1));
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_INT32(-1, enc.pos());

  ASSERT_OK(enc.AlignToPosition(500));
  TEST_ASSERT_EQUAL_INT32(500, enc.pos());

  enc.SetRawPosition(0);
  ASSERT_OK(enc.Process(kDt));
  TEST_ASSERT_EQUAL_INT32(501, enc.pos());
}

// ============================================================================
// 4. 方向控制（Direction Control）
// ============================================================================

void test_set_reverse_basic(void) {
  Encoder enc;
  enc.SetRawPosition(200);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  TEST_ASSERT_EQUAL(static_cast<int>(Reverse::kNormal), static_cast<int>(enc.reverse()));
  TEST_ASSERT_EQUAL_INT32(200, enc.pos());

  enc.set_reverse(Reverse::kReverse);
  TEST_ASSERT_EQUAL(static_cast<int>(Reverse::kReverse), static_cast<int>(enc.reverse()));
  const int32_t expected = static_cast<int32_t>(kCpr) - 200;
  TEST_ASSERT_EQUAL_INT32(expected, enc.pos());
}

void test_set_reverse_recalculates_pos(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_INT32(100, enc.pos());

  enc.set_reverse(Reverse::kReverse);
  const int32_t expected = static_cast<int32_t>(kCpr) - 100;
  TEST_ASSERT_EQUAL_INT32(expected, enc.pos());
}

void test_set_reverse_at_boundary(void) {
  Encoder enc;
  enc.SetRawPosition(1);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_INT32(1, enc.pos());

  enc.set_reverse(Reverse::kReverse);
  TEST_ASSERT_EQUAL_INT32(-1, enc.pos());
}

void test_set_reverse_dynamic(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));

  enc.set_reverse(Reverse::kReverse);
  TEST_ASSERT_EQUAL(static_cast<int>(Reverse::kReverse), static_cast<int>(enc.reverse()));

  enc.set_reverse(Reverse::kNormal);
  TEST_ASSERT_EQUAL(static_cast<int>(Reverse::kNormal), static_cast<int>(enc.reverse()));
  TEST_ASSERT_EQUAL_INT32(100, enc.pos());

  const int32_t expected_reverse = static_cast<int32_t>(kCpr) - 100;
  enc.set_reverse(Reverse::kReverse);
  TEST_ASSERT_EQUAL_INT32(expected_reverse, enc.pos());
}

// 多圈累积后切换方向：ResetPosition 丢弃多圈状态
void test_set_reverse_after_multi_turn(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  ASSERT_OK(enc.Process(kDt));
  StepOneRevolutionCw(enc);
  TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(kCpr), enc.pos());
  TEST_ASSERT_EQUAL_INT32(1, enc.revolutions());

  enc.set_reverse(Reverse::kReverse);
  TEST_ASSERT_EQUAL_INT32(0, enc.pos());
  TEST_ASSERT_EQUAL_INT32(0, enc.revolutions());
}

// ============================================================================
// 5. 零点偏移（Homing Offset）
// ============================================================================

void test_set_homing_offset_basic(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{20, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_INT32(20, enc.homing_offset());
  TEST_ASSERT_EQUAL_INT32(120, enc.pos());
}

void test_set_homing_offset_adjusts_pos(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{20, Reverse::kNormal}));

  const int32_t old_pos = enc.pos();
  enc.set_homing_offset(50);
  TEST_ASSERT_EQUAL_INT32(50, enc.homing_offset());
  TEST_ASSERT_EQUAL_INT32(old_pos + (50 - 20), enc.pos());
}

void test_set_homing_offset_negative(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{-50, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_INT32(-50, enc.homing_offset());
  TEST_ASSERT_EQUAL_INT32(50, enc.pos());
}

void test_homing_offset_with_reverse(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Init(Config{50, Reverse::kReverse}));
  TEST_ASSERT_EQUAL_INT32(50, enc.homing_offset());

  const int32_t expected_pos = static_cast<int32_t>(kCpr) - 100 + 50;
  TEST_ASSERT_EQUAL_INT32(expected_pos, enc.pos());

  enc.set_homing_offset(100);
  TEST_ASSERT_EQUAL_INT32(100, enc.homing_offset());
  TEST_ASSERT_EQUAL_INT32(expected_pos + 50, enc.pos());
}

// 多圈后调整偏移应保留累积值
void test_set_homing_offset_after_multi_turn(void) {
  Encoder enc;
  enc.SetRawPosition(0);
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  ASSERT_OK(enc.Process(kDt));
  StepOneRevolutionCw(enc);
  TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(kCpr), enc.pos());

  enc.set_homing_offset(100);
  TEST_ASSERT_EQUAL_INT32(static_cast<int32_t>(kCpr) + 100, enc.pos());
  TEST_ASSERT_EQUAL_INT32(1, enc.revolutions());
}

// 偏移超过一圈范围：mod 归一化后仍正确
void test_homing_offset_larger_than_cpr(void) {
  Encoder enc;
  enc.SetRawPosition(100);
  const int32_t large_offset = static_cast<int32_t>(kCpr) + 50;
  ASSERT_OK(enc.Init(Config{large_offset, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_INT32(150, enc.pos());
  TEST_ASSERT_EQUAL_INT32(large_offset, enc.homing_offset());
}

// ============================================================================
// 6. 原始位置管理（Raw Position）
// ============================================================================

void test_raw_pos_in_range(void) {
  Encoder enc;
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  enc.SetRawPosition(100);
  ASSERT_OK(enc.Process(kDt));
  TEST_ASSERT_TRUE(enc.raw_pos() < kCpr);
  TEST_ASSERT_EQUAL_UINT32(100, enc.raw_pos());
}

void test_raw_pos_out_of_range_wraps(void) {
  Encoder enc;
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));

  enc.SetRawPosition(static_cast<int32_t>(kCpr + 100));
  ASSERT_OK(enc.Process(kDt));
  TEST_ASSERT_TRUE(enc.raw_pos() < kCpr);
  TEST_ASSERT_EQUAL_UINT32(100, enc.raw_pos());
}

void test_raw_pos_after_process(void) {
  const uint32_t raw_set = 123;
  Encoder        enc;
  enc.SetRawPosition(static_cast<int32_t>(raw_set));
  ASSERT_OK(enc.Init(Config{0, Reverse::kNormal}));
  TEST_ASSERT_EQUAL_UINT32(raw_set, enc.raw_pos());

  ASSERT_OK(enc.Process(kDt));
  TEST_ASSERT_EQUAL_UINT32(raw_set, enc.raw_pos());
}

void run_tests(void) {
  // 1. 初始化与配置
  RUN_TEST(EncoderTest::test_init_basic);
  RUN_TEST(EncoderTest::test_init_with_homing_offset);
  RUN_TEST(EncoderTest::test_init_with_reverse);
  RUN_TEST(EncoderTest::test_init_boundary_zero);
  RUN_TEST(EncoderTest::test_init_boundary_cpr_minus_one);
  RUN_TEST(EncoderTest::test_init_combined_offset_and_reverse);
  RUN_TEST(EncoderTest::test_snap_near_zero_at_threshold);
  RUN_TEST(EncoderTest::test_reinit_resets_state);

  // 2. 位置处理与多圈
  RUN_TEST(EncoderTest::test_process_cross_cpr_cw);
  RUN_TEST(EncoderTest::test_process_cross_cpr_ccw);
  RUN_TEST(EncoderTest::test_process_one_revolution_cw);
  RUN_TEST(EncoderTest::test_process_one_revolution_ccw);
  RUN_TEST(EncoderTest::test_process_two_revolutions);
  RUN_TEST(EncoderTest::test_process_revolutions_calculation);
  RUN_TEST(EncoderTest::test_process_reverse_direction);
  RUN_TEST(EncoderTest::test_process_shortest_wrap_distance);
  RUN_TEST(EncoderTest::test_process_cross_cpr_with_reverse);

  // 3. 对齐功能
  RUN_TEST(EncoderTest::test_align_to_zero);
  RUN_TEST(EncoderTest::test_align_to_position);
  RUN_TEST(EncoderTest::test_align_then_move);
  RUN_TEST(EncoderTest::test_align_across_boundary);
  RUN_TEST(EncoderTest::test_align_with_reverse);
  RUN_TEST(EncoderTest::test_align_from_negative_pos);

  // 4. 方向控制
  RUN_TEST(EncoderTest::test_set_reverse_basic);
  RUN_TEST(EncoderTest::test_set_reverse_recalculates_pos);
  RUN_TEST(EncoderTest::test_set_reverse_at_boundary);
  RUN_TEST(EncoderTest::test_set_reverse_dynamic);
  RUN_TEST(EncoderTest::test_set_reverse_after_multi_turn);

  // 5. 零点偏移
  RUN_TEST(EncoderTest::test_set_homing_offset_basic);
  RUN_TEST(EncoderTest::test_set_homing_offset_adjusts_pos);
  RUN_TEST(EncoderTest::test_set_homing_offset_negative);
  RUN_TEST(EncoderTest::test_homing_offset_with_reverse);
  RUN_TEST(EncoderTest::test_set_homing_offset_after_multi_turn);
  RUN_TEST(EncoderTest::test_homing_offset_larger_than_cpr);

  // 6. 原始位置管理
  RUN_TEST(EncoderTest::test_raw_pos_in_range);
  RUN_TEST(EncoderTest::test_raw_pos_out_of_range_wraps);
  RUN_TEST(EncoderTest::test_raw_pos_after_process);
}

}  // namespace EncoderTest

#undef ASSERT_OK
