// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file profile.h
 * @brief 速度轮廓生成器（Velocity-based Profile）
 */

#pragma once

#include <Arduino.h>

#include "math/math.h"
#include "noncopyable.h"

namespace zeta::math {

/**
 * @brief 速度轮廓生成器（Velocity-based Profile）
 *
 * 根据 Dynamixel XL330 协议规范实现四种轮廓类型：
 *   - Step（阶跃）：ProfileVelocity = 0，直接跳转到目标位置
 *   - Rectangular（矩形）：ProfileVelocity ≠ 0，ProfileAcceleration = 0
 *   - Triangular（三角形）：加速度非零，但位移不足以达到峰值速度
 *   - Trapezoidal（梯形）：完整的加速 + 匀速 + 减速三段轮廓
 *
 * 输出：
 *   - position_trajectory()       [pulses]     用于 PID 目标
 *   - velocity_trajectory_rpm()   [RPM]        用于写入 VelocityTrajectory 寄存器
 *   - velocity_trajectory_cps()   [counts/s]   用于 FF1st 前馈（匹配 encoder_pll.velocity() 单位）
 *   - acceleration_cps2()         [counts/s²]  用于 FF2nd 前馈
 */
class Profile : public zeta::Noncopyable {
 public:
  /// @brief 轮廓类型，与 MovingStatus 寄存器 Bit4-5 对应
  enum class Type : uint8_t {
    kStep        = 0,  ///< 阶跃（无轮廓）
    kRect        = 1,  ///< 矩形
    kTriangular  = 2,  ///< 三角形
    kTrapezoidal = 3,  ///< 梯形
  };

  struct Config {
    uint16_t cpr;  ///< Counts Per Revolution，编码器每圈计数（如4096）
  };

  explicit Profile(const Config& config) : config_(config) {}

  /**
   * @brief 设置新目标，重新生成轮廓参数
   *
   * @param start_pos_pulse       当前位置 [pulses]（float 保留小数精度）
   * @param goal_pos_pulse        目标位置 [pulses]
   * @param profile_velocity_rpm  轮廓速度 [RPM]，0 表示阶跃
   * @param profile_acc_rpm_min2  轮廓加速度 [rev/min²]，0 表示矩形
   */
  void SetGoal(float   start_pos_pulse,
               int32_t goal_pos_pulse,
               float   profile_velocity_rpm,
               float   profile_acc_rpm_min2);

  /**
   * @brief 推进轮廓，每个控制周期调用一次
   * @param dt 时间步长 [s]
   */
  void Process(float dt);

  /// @brief 当前轨迹位置 [pulses]
  float position_trajectory() const { return pos_traj_; }

  /// @brief 当前轨迹速度 [RPM]，用于写入寄存器
  float velocity_trajectory_rpm() const { return vel_traj_rpm_; }

  /// @brief 当前轨迹速度 [counts/s]，用于 FF1st 前馈
  float velocity_trajectory_cps() const { return vel_traj_cps_; }

  /// @brief 当前轨迹加速度 [counts/s²]，用于 FF2nd 前馈
  float acceleration_cps2() const { return accel_cps2_; }

  /// @brief 轮廓是否已完成
  bool is_complete() const { return completed_; }

  /// @brief 当前使用的轮廓类型
  Type type() const { return type_; }

  /// @brief 当前轮廓的目标位置 [pulses]
  int32_t goal() const { return static_cast<int32_t>(goal_pos_); }

 private:
  Config config_;

  /** @name 轮廓参数（SetGoal 时计算，Process 时使用） */
  Type type_      = Type::kStep;
  bool completed_ = true;

  float start_pos_ = 0.0f;  ///< 起始位置 [pulses]
  float goal_pos_  = 0.0f;  ///< 目标位置 [pulses]
  float dir_       = 0.0f;  ///< 方向（+1 或 -1）

  float peak_vel_cps_ = 0.0f;  ///< 峰值速度 [counts/s]
  float peak_vel_rpm_ = 0.0f;  ///< 峰值速度 [RPM]（缓存）
  float acc_cps2_     = 0.0f;  ///< 加速度 [counts/s²]

  float t1_ = 0.0f;  ///< 加速段结束时刻 [s]
  float t2_ = 0.0f;  ///< 匀速段结束时刻（减速开始）[s]
  float t3_ = 0.0f;  ///< 减速段结束时刻（轮廓总时长）[s]

  float d1_ = 0.0f;  ///< 加速段结束时的位移 [pulses]
  float d2_ = 0.0f;  ///< 匀速段结束时的位移 [pulses]（相对起点）

  /** @name 运行状态（Process 时更新） */
  float elapsed_      = 0.0f;  ///< 已运行时间 [s]
  float pos_traj_     = 0.0f;  ///< 当前轨迹位置 [pulses]
  float vel_traj_rpm_ = 0.0f;  ///< 当前轨迹速度 [RPM]
  float vel_traj_cps_ = 0.0f;  ///< 当前轨迹速度 [counts/s]
  float accel_cps2_   = 0.0f;  ///< 当前轨迹加速度 [counts/s²]

  /**
   * @brief 将 counts/s 转换为 RPM
   * @param cps 速度 [counts/s]
   * @return 转速 [RPM]
   */
  float CpsToRpm(float cps) const { return cps * 60.0f / static_cast<float>(config_.cpr); }
};

}  // namespace zeta::math

/** @name 实现 */
namespace zeta::math {

inline void Profile::SetGoal(float   start_pos_pulse,
                             int32_t goal_pos_pulse,
                             float   profile_velocity_rpm,
                             float   profile_acc_rpm_min2) {
  start_pos_ = start_pos_pulse;
  goal_pos_  = static_cast<float>(goal_pos_pulse);
  elapsed_   = 0.0f;
  completed_ = false;

  const float delta = goal_pos_ - start_pos_;
  if (fabsf(delta) < 0.5f) {
    // 误差不足半个脉冲，视为已到达
    pos_traj_     = goal_pos_;
    vel_traj_rpm_ = 0.0f;
    vel_traj_cps_ = 0.0f;
    accel_cps2_   = 0.0f;
    type_         = Type::kStep;
    completed_    = true;
    return;
  }

  dir_ = (delta > 0.0f) ? 1.0f : -1.0f;

  const float cpr_f = static_cast<float>(config_.cpr);

  // Step 模式：立即跳转
  if (profile_velocity_rpm <= 0.0f) {
    type_         = Type::kStep;
    pos_traj_     = goal_pos_;
    vel_traj_rpm_ = 0.0f;
    vel_traj_cps_ = 0.0f;
    accel_cps2_   = 0.0f;
    completed_    = true;
    return;
  }

  // 将峰值速度转换为 counts/s
  peak_vel_cps_ = profile_velocity_rpm * cpr_f / 60.0f;
  peak_vel_rpm_ = profile_velocity_rpm;

  // Rectangular 模式：无加速度坡
  if (profile_acc_rpm_min2 <= 0.0f) {
    type_     = Type::kRect;
    acc_cps2_ = 0.0f;
    t1_       = 0.0f;
    t2_       = 0.0f;
    d1_       = 0.0f;
    d2_       = 0.0f;
    t3_       = fabsf(delta) / peak_vel_cps_;
    return;
  }

  // 加速度 [counts/s²]：a[rev/min²] / 3600 [s²/min²] * cpr [counts/rev]
  acc_cps2_ = profile_acc_rpm_min2 * cpr_f / 3600.0f;

  // 计算加速段位移 d1 = 0.5 * peak_vel² / acc [counts]
  const float d1_full = 0.5f * peak_vel_cps_ * peak_vel_cps_ / acc_cps2_;

  if (2.0f * d1_full >= fabsf(delta)) {
    // 三角形：位移不足以达到峰速，实际峰速更低
    type_ = Type::kTriangular;
    // 解方程：2 * (0.5 * v² / acc) = |delta|  →  v = sqrt(acc * |delta|)
    peak_vel_cps_ = sqrtf(acc_cps2_ * fabsf(delta));
    peak_vel_rpm_ = CpsToRpm(peak_vel_cps_);
    d1_           = fabsf(delta) / 2.0f;
    d2_           = fabsf(delta);
    t1_           = peak_vel_cps_ / acc_cps2_;
    t2_           = t1_;  // 无匀速段，减速立即开始
    t3_           = 2.0f * t1_;
  } else {
    // 梯形：完整三段
    type_                = Type::kTrapezoidal;
    d1_                  = d1_full;
    t1_                  = peak_vel_cps_ / acc_cps2_;
    const float t_cruise = (fabsf(delta) - 2.0f * d1_) / peak_vel_cps_;
    t2_                  = t1_ + t_cruise;
    d2_                  = d1_ + peak_vel_cps_ * t_cruise;
    t3_                  = t2_ + t1_;
  }
}

inline void Profile::Process(float dt) {
  if (completed_)
    return;

  elapsed_ += dt;

  const float abs_delta = fabsf(goal_pos_ - start_pos_);

  if (elapsed_ >= t3_) {
    // 轮廓结束，固定在目标位置
    completed_    = true;
    pos_traj_     = goal_pos_;
    vel_traj_rpm_ = 0.0f;
    vel_traj_cps_ = 0.0f;
    accel_cps2_   = 0.0f;
    return;
  }

  float vel_cps   = 0.0f;
  float cur_accel = 0.0f;
  float offset    = 0.0f;  // 相对起点的位移（正值，方向由 dir_ 决定）

  if (type_ == Type::kRect) {
    // 矩形：全程匀速
    vel_cps   = peak_vel_cps_;
    cur_accel = 0.0f;
    offset    = peak_vel_cps_ * elapsed_;
    // 钳位到目标距离（避免浮点超出）
    if (offset > abs_delta)
      offset = abs_delta;
  } else {
    const float t = elapsed_;

    if (t < t1_) {
      // 阶段1：加速
      cur_accel = acc_cps2_;
      vel_cps   = acc_cps2_ * t;
      offset    = 0.5f * acc_cps2_ * t * t;
    } else if (t < t2_) {
      // 阶段2：匀速（梯形专有）
      cur_accel = 0.0f;
      vel_cps   = peak_vel_cps_;
      offset    = d1_ + peak_vel_cps_ * (t - t1_);
    } else {
      // 阶段3：减速
      const float td = t - t2_;
      cur_accel      = -acc_cps2_;
      vel_cps        = peak_vel_cps_ - acc_cps2_ * td;
      if (vel_cps < 0.0f)
        vel_cps = 0.0f;
      offset = d2_ + peak_vel_cps_ * td - 0.5f * acc_cps2_ * td * td;
    }

    // 防止浮点误差导致超出目标
    if (offset > abs_delta)
      offset = abs_delta;
  }

  pos_traj_     = start_pos_ + dir_ * offset;
  vel_traj_cps_ = dir_ * vel_cps;
  vel_traj_rpm_ = CpsToRpm(vel_traj_cps_);
  accel_cps2_   = dir_ * cur_accel;
}

}  // namespace zeta::math
