// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file task_scheduler.h
 * @brief 协作式任务调度器（按周期调用回调）
 */

#pragma once

#include <Arduino.h>

#include <array>

#include "error.h"
#include "noncopyable.h"

namespace zeta::utils {

/**
 * @brief 简单的基于时间片的协作式任务调度器
 *
 * 特点：
 * - 非阻塞，每个任务各自维护时间基准
 * - 固定容量（避免动态分配）
 * - 支持集中 Tick 与可选的按最快任务周期睡眠
 */
template <uint32_t MaxTasks = 16>
class TaskScheduler : public zeta::Noncopyable {
 public:
  using Callback = Error (*)(float dt);

  struct TaskDesc {
    Callback callback     = nullptr;  ///< 任务回调
    uint32_t period_us    = 0;        ///< 任务周期 [μs]
    uint32_t next_time_us = 0;        ///< 下次执行时刻 [μs]
  };

  static constexpr float kMicroToSec = 1.0f / 1000000.0f;

  uint32_t min_period_us() const;
  uint32_t size() const;
  uint32_t capacity() const;
  Error    AddTask(Callback callback, uint32_t rate_hz);
  Error    RemoveTask(const Callback callback);
  Error    ClearTasks();
  Error    Tick();

 private:
  void  UpdateMinPeriod();
  Error TickNonBlocking();

  uint32_t                       size_{0};
  std::array<TaskDesc, MaxTasks> tasks_{};
  uint32_t                       min_period_us_{0};
};

}  // namespace zeta::utils

namespace zeta::utils {

template <uint32_t MaxTasks>
uint32_t TaskScheduler<MaxTasks>::min_period_us() const {
  return min_period_us_;
}

template <uint32_t MaxTasks>
uint32_t TaskScheduler<MaxTasks>::size() const {
  return size_;
}

template <uint32_t MaxTasks>
uint32_t TaskScheduler<MaxTasks>::capacity() const {
  return tasks_.size();
}

template <uint32_t MaxTasks>
Error TaskScheduler<MaxTasks>::AddTask(Callback callback, uint32_t rate_hz) {
  VERIFY(callback != nullptr && rate_hz != 0, Error::kInvalidArg);
  VERIFY(size_ < tasks_.size(), Error::kInvalidArg);
  const auto period_us = 1000000u / rate_hz;
  VERIFY(period_us != 0u, Error::kInvalidArg);
  const uint32_t idx       = size_;
  tasks_[idx].callback     = callback;
  tasks_[idx].period_us    = period_us;
  tasks_[idx].next_time_us = micros();
  size_ += 1;
  UpdateMinPeriod();
  return Error::kOk;
}

template <uint32_t MaxTasks>
Error TaskScheduler<MaxTasks>::RemoveTask(const Callback callback) {
  VERIFY(callback != nullptr, Error::kInvalidArg);
  uint32_t found = size_;
  for (uint32_t i = 0; i < size_; ++i) {
    if (tasks_[i].callback == callback) {
      found = i;
      break;
    }
  }
  if (found == size_) {
    return Error::kOk;
  }
  for (uint32_t i = found; i < size_ - 1; ++i) {
    tasks_[i] = tasks_[i + 1];
  }
  size_ -= 1;
  UpdateMinPeriod();
  return Error::kOk;
}

template <uint32_t MaxTasks>
Error TaskScheduler<MaxTasks>::ClearTasks() {
  size_          = 0;
  min_period_us_ = 0;
  return Error::kOk;
}

template <uint32_t MaxTasks>
Error TaskScheduler<MaxTasks>::TickNonBlocking() {
  const uint32_t now = micros();
  for (uint32_t i = 0; i < size_; ++i) {
    TaskDesc& t = tasks_[i];
    if (now >= t.next_time_us) {
      const float dt = static_cast<float>(t.period_us) * kMicroToSec;
      t.next_time_us += t.period_us;
      CHECK(t.callback(dt));
    }
  }
  return Error::kOk;
}

template <uint32_t MaxTasks>
Error TaskScheduler<MaxTasks>::Tick() {
  const uint32_t start = micros();
  CHECK(TickNonBlocking());

  const uint32_t min_period = min_period_us();
  const uint32_t elapsed    = micros() - start;
  if (elapsed < min_period) {
    delayMicroseconds(min_period - elapsed);
  }
  return Error::kOk;
}

template <uint32_t MaxTasks>
void TaskScheduler<MaxTasks>::UpdateMinPeriod() {
  min_period_us_ = 0;
  for (uint32_t i = 0; i < size_; ++i) {
    if (min_period_us_ == 0 || tasks_[i].period_us < min_period_us_) {
      min_period_us_ = tasks_[i].period_us;
    }
  }
}

}  // namespace zeta::utils