// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* TaskScheduler 单元测试。覆盖初始状态、AddTask/RemoveTask/ClearTasks、
 * Tick() 触发回调与 dt 及返回值。 */

#include <unity.h>

#include "error.h"
#include "utils/task_scheduler.h"

namespace SchedulerTest {

using Scheduler = zeta::utils::TaskScheduler<>;
using Error     = zeta::Error;

// 占位回调，仅返回 kOk，用于 AddTask/RemoveTask 等不关心执行逻辑的用例。
static zeta::Error DummyCallback(float /*dt*/) {
  return zeta::Error::kOk;
}

// Tick 测试用：记录回调被调用次数。
static uint32_t g_tick_call_count = 0;
// Tick 测试用：记录最后一次回调收到的 dt（秒）。
static float g_tick_last_dt = 0.0f;

// 在 Tick 测试中用于统计调用次数并记录 dt。
static zeta::Error CountingCallback(float dt) {
  g_tick_call_count += 1;
  g_tick_last_dt = dt;
  return zeta::Error::kOk;
}

// 验证未添加任务时 size==0、capacity==16、min_period_us==0。
void test_initial_state(void) {
  Scheduler s;
  TEST_ASSERT_EQUAL_UINT32(0, s.size());
  TEST_ASSERT_EQUAL_UINT32(16, s.capacity());
  TEST_ASSERT_EQUAL_UINT32(0, s.min_period_us());
}

// 验证合法参数 AddTask(callback, 1000) 返回 kOk，size==1，min_period_us==1000。
void test_add_task_success(void) {
  Scheduler  s;
  const auto err = s.AddTask(DummyCallback, 1000);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(1, s.size());
  TEST_ASSERT_EQUAL_UINT32(1000, s.min_period_us());
}

// 验证 callback 为 nullptr、rate_hz 为 0 或导致 period_us==0 时 AddTask 返回 kInvalidArg。
void test_add_task_invalid_args(void) {
  Scheduler s;
  auto      err = s.AddTask(nullptr, 1000);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kInvalidArg), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(0, s.size());

  err = s.AddTask(DummyCallback, 0);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kInvalidArg), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(0, s.size());

  err = s.AddTask(DummyCallback, 2000000);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kInvalidArg), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(0, s.size());
}

// 验证任务列表已满时再添加返回 kInvalidArg，size 不变。
void test_add_task_full(void) {
  zeta::utils::TaskScheduler<2> s;
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(s.AddTask(DummyCallback, 100)));
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk),
                    static_cast<int>(s.AddTask(CountingCallback, 200)));
  const auto err = s.AddTask(DummyCallback, 300);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kInvalidArg), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(2, s.size());
}

// 验证 RemoveTask(nullptr) 返回 kInvalidArg。
void test_remove_task_null_callback(void) {
  Scheduler s;
  s.AddTask(DummyCallback, 1000);
  const auto err = s.RemoveTask(nullptr);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kInvalidArg), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(1, s.size());
}

// 验证按 callback 移除任务后 size 正确；重复移除同一 callback 仍返回 kOk。
void test_remove_task(void) {
  Scheduler s;
  s.AddTask(DummyCallback, 1000);
  s.AddTask(CountingCallback, 500);
  TEST_ASSERT_EQUAL_UINT32(2, s.size());

  const auto err = s.RemoveTask(CountingCallback);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(1, s.size());

  const auto err2 = s.RemoveTask(CountingCallback);
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err2));
  TEST_ASSERT_EQUAL_UINT32(1, s.size());

  s.RemoveTask(DummyCallback);
  TEST_ASSERT_EQUAL_UINT32(0, s.size());
}

// 验证 ClearTasks() 后 size==0、min_period_us==0。
void test_clear_tasks(void) {
  Scheduler s;
  s.AddTask(DummyCallback, 1000);
  s.AddTask(CountingCallback, 500);
  TEST_ASSERT_EQUAL_UINT32(2, s.size());

  const auto err = s.ClearTasks();
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(0, s.size());
  TEST_ASSERT_EQUAL_UINT32(0, s.min_period_us());
}

// 验证 Tick() 触发回调一次且传入 dt 约为 1/rate_hz（如 1000Hz => 0.001s）。
void test_tick_invokes_callback(void) {
  Scheduler s;
  g_tick_call_count = 0;
  g_tick_last_dt    = 0.0f;
  s.AddTask(CountingCallback, 1000);

  const auto err = s.Tick();
  TEST_ASSERT_EQUAL(static_cast<int>(Error::kOk), static_cast<int>(err));
  TEST_ASSERT_EQUAL_UINT32(1, g_tick_call_count);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.001f, g_tick_last_dt);
}

void run_tests(void) {
  RUN_TEST(SchedulerTest::test_initial_state);
  RUN_TEST(SchedulerTest::test_add_task_success);
  RUN_TEST(SchedulerTest::test_add_task_invalid_args);
  RUN_TEST(SchedulerTest::test_add_task_full);
  RUN_TEST(SchedulerTest::test_remove_task_null_callback);
  RUN_TEST(SchedulerTest::test_remove_task);
  RUN_TEST(SchedulerTest::test_clear_tasks);
  RUN_TEST(SchedulerTest::test_tick_invokes_callback);
}

}  // namespace SchedulerTest
