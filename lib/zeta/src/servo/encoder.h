// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file encoder.h
 * @brief 位置传感器基类（CRTP），单圈绝对→多圈累积
 */

#pragma once

#include <Arduino.h>

#include <algorithm>

#include "math/math.h"
#include "math/resolution.h"
#include "servo/servo_types.h"

namespace zeta::servo {

/**
 * @internal
 * @section encoder_design Encoder 设计说明
 *
 * ## 位置模型
 *
 * Encoder 将单圈绝对传感器转换为多圈累积位置：
 *
 *   raw_pos  [0, CPR-1]   硬件传感器读数，单圈绝对值，每圈回绕
 *   pos      [-inf, +inf] 用户可见位置，多圈累积，不回绕
 *
 * ## 坐标变换管线（仅 Init / ResetPosition 使用）
 *
 *   Hardware Sensor
 *        |
 *   raw_pos [0, CPR-1]         ReadRaw() 从硬件读取
 *        |
 *   [RawToLocalPosition]       reverse 时: local = (CPR - raw) % CPR
 *        |                     否则:        local = raw
 *   local_pos [0, CPR-1]
 *        |
 *   [+ homing_offset, mod CPR] 零点平移
 *        |
 *   pos_in_cpr [0, CPR-1]
 *        |
 *   [SnapNearZero]             接近 CPR 的值折叠为负数
 *        |                     例: CPR-1 → -1（偏好从零附近启动）
 *   initial pos_
 *
 * ## 两条更新路径
 *
 *   1. ResetPosition（绝对定位）
 *      走完整管线，从传感器当前值计算 pos_。
 *      用于 Init() 和 set_reverse()——此时需要从头建立位置。
 *
 *   2. Process（增量追踪）
 *      读新 raw，计算与上次 raw 的差值，wrap 到 [-CPR/2, CPR/2)
 *      取最短路径，按 reverse 取符号后累加到 pos_。
 *      自然处理多圈，无需边界判断。
 *
 * ## 关键不变量
 *
 *   - raw_pos_ 始终是最近一次硬件读数 [0, CPR-1]
 *   - pos_ 仅由 ResetPosition 重置或由 Process / set_homing_offset 增量调整
 *   - set_homing_offset 直接将偏移差值加到 pos_，不重读硬件，保留多圈状态
 *   - AlignToPosition 是 set_homing_offset 的语法糖：反推偏移使 pos()==target
 */

/**
 * @brief 位置传感器基类（CRTP），将单圈绝对传感器扩展为多圈累积位置
 *
 * 派生类只需实现 ReadRawImpl(uint32_t& out_raw) 提供硬件读数。
 *
 * @tparam DerivedType CRTP 派生类型，须提供 ReadRawImpl
 * @tparam Bits        传感器分辨率位数，CPR = 2^Bits
 */
template <typename DerivedType, uint8_t Bits>
class Encoder : public zeta::Noncopyable {
 public:
  struct Config {
    int32_t homing_offset;
    Reverse reverse;
  };

  /// @brief 传感器分辨率（编译期常量，存储在Flash）
  static constexpr math::Resolution<Bits> kResolution{};

  /**
   * @brief SnapNearZero 的判定宽度（约 1 度对应的 counts 数）
   *
   * 初始化时，若 pos_in_cpr >= CPR - kEdgeThreshold，则折叠为负值。
   * 确保启动位置尽量靠近零，避免 CPR-1 vs 0 的歧义。
   */
  static constexpr int32_t kEdgeThreshold =
      std::max(INT32_C(1), static_cast<int32_t>((kResolution.kEncoderCpr + 359) / 360));

  /**
   * @brief 硬件原始读数 [0, CPR-1]，由 ReadRaw 获取
   * @return 原始读数 [0, CPR-1]
   */
  uint32_t raw_pos() const;

  /**
   * @brief 多圈累积位置 [-inf, +inf]，Init 后由 Process 持续更新
   * @return 多圈位置 [pulse]
   */
  int32_t pos() const;

  /**
   * @brief 完整圈数 = pos() / CPR
   * @return 圈数
   */
  int32_t revolutions() const;

  /**
   * @brief 安装方向：kNormal(+1) 正转，kReverse(-1) 反转
   * @return 当前安装方向
   */
  Reverse reverse() const;

  /**
   * @brief 切换安装方向，触发 ResetPosition 重建 pos_
   * @param reverse 方向 kNormal / kReverse
   */
  void set_reverse(const Reverse reverse);

  /**
   * @brief 零点偏移 (counts)，加到 local_pos 上平移零点
   * @return 零点偏移 [counts]
   */
  int32_t homing_offset() const;

  /**
   * @brief 调整零点偏移，pos_ 同步偏移 (new - old) 而不重读硬件
   * @param homing_offset 新的零点偏移 [counts]
   */
  void set_homing_offset(const int32_t homing_offset);

  /**
   * @brief 初始化传感器
   *
   * 设置配置参数后调用 ResetPosition 从硬件建立初始 pos_。
   * 派生类若需额外初始化（如总线配置），应先完成后再调用此方法。
   * @param config 零点偏移与安装方向
   * @return 错误码
   */
  Error Init(const Config& config);

  /**
   * @brief 增量更新位置（主循环每拍调用）
   *
   * 读取新 raw，计算与上次 raw 的最短路径差值，按方向累加到 pos_。
   * @param dt 时间间隔 [s]
   * @return 错误码
   */
  Error Process(float dt);

  /**
   * @brief 对齐到目标位置
   *
   * 反推 homing_offset 使 pos() == target，不重读硬件。
   * @param target 目标位置 [pulse]
   * @return 错误码
   */
  Error AlignToPosition(uint32_t target);

 protected:
  /**
   * @brief CRTP 分派：调用派生类的 ReadRawImpl
   * @param out_raw 输出原始读数 [0, CPR-1]
   * @return 错误码
   */
  Error ReadRaw(uint32_t& out_raw);

  /**
   * @brief 绝对定位：从硬件读数走完整变换管线重建 pos_
   *
   * 调用场景：Init()、set_reverse()——需要从头计算位置。
   * 会丢弃已有的多圈累积值。
   */
  Error ResetPosition();

  /**
   * @brief 将硬件原始值转换为考虑安装方向后的本地位置
   * @param raw 硬件原始值 [0, CPR-1]
   * @param rev 安装方向
   * @param cpr 每圈计数
   * @return [0, CPR-1]，reverse 时翻转方向
   */
  static int32_t RawToLocalPosition(uint32_t raw, Reverse rev, uint32_t cpr);

  /**
   * @brief 将接近 CPR 的位置折叠为负值，使初始位置靠近零
   *
   * 例: 12-bit 编码器，CPR=4096，pos_in_cpr=4095 → 返回 -1
   * 判定边界: pos_in_cpr >= CPR - edge_threshold 时折叠
   * @param pos_in_cpr 当前 CPR 内位置 [0, CPR-1]
   * @param cpr 每圈计数
   * @param edge_threshold 判定宽度 [counts]
   * @return 折叠后的位置（可能为负）
   */
  static int32_t SnapNearZero(int32_t pos_in_cpr, int32_t cpr, int32_t edge_threshold);

  /** @name 状态变量 */
  /// @brief 最近一次硬件读数 [0, CPR-1]，Process 每拍更新
  uint32_t raw_pos_ = 0;
  /// @brief 多圈累积位置，ResetPosition 重置 / Process 增量更新
  int32_t pos_ = 0;
  /// @brief 安装方向 kNormal(+1) / kReverse(-1)，影响 delta 符号
  Reverse reverse_ = Reverse::kNormal;
  /// @brief 零点偏移 (counts)，加到 local_pos 平移用户零点
  int32_t homing_offset_ = 0;
};

}  // namespace zeta::servo

/** @name 模板实现 */
namespace zeta::servo {

// ---- 访问器 ----

template <typename DerivedType, uint8_t Bits>
uint32_t Encoder<DerivedType, Bits>::raw_pos() const {
  return raw_pos_;
}

template <typename DerivedType, uint8_t Bits>
int32_t Encoder<DerivedType, Bits>::pos() const {
  return pos_;
}

template <typename DerivedType, uint8_t Bits>
int32_t Encoder<DerivedType, Bits>::revolutions() const {
  return pos() / kResolution.kEncoderCpr;
}

template <typename DerivedType, uint8_t Bits>
Reverse Encoder<DerivedType, Bits>::reverse() const {
  return reverse_;
}

template <typename DerivedType, uint8_t Bits>
void Encoder<DerivedType, Bits>::set_reverse(const Reverse reverse) {
  reverse_ = reverse;
  ResetPosition();
}

template <typename DerivedType, uint8_t Bits>
int32_t Encoder<DerivedType, Bits>::homing_offset() const {
  return homing_offset_;
}

template <typename DerivedType, uint8_t Bits>
void Encoder<DerivedType, Bits>::set_homing_offset(const int32_t homing_offset) {
  const auto delta_offset = homing_offset - homing_offset_;
  pos_ += delta_offset;
  homing_offset_ = homing_offset;
}

// ---- 核心流程 ----

template <typename DerivedType, uint8_t Bits>
Error Encoder<DerivedType, Bits>::Init(const Config& config) {
  homing_offset_ = config.homing_offset;
  reverse_       = config.reverse;

  return ResetPosition();
}

template <typename DerivedType, uint8_t Bits>
Error Encoder<DerivedType, Bits>::Process(float dt) {
  (void)dt;
  uint32_t raw_new;
  CHECK(ReadRaw(raw_new));
  const int32_t raw_delta     = static_cast<int32_t>(raw_new) - static_cast<int32_t>(raw_pos_);
  const int32_t wrapped_delta = math::wrap_pm(raw_delta, kResolution.kEncoderCpr);
  const int32_t signed_delta  = (reverse_ == Reverse::kNormal) ? wrapped_delta : -wrapped_delta;
  pos_ += signed_delta;
  raw_pos_ = raw_new;
  return Error::kOk;
}

template <typename DerivedType, uint8_t Bits>
Error Encoder<DerivedType, Bits>::AlignToPosition(uint32_t target) {
  const auto delta_to_target = static_cast<int32_t>(target) - pos();
  set_homing_offset(homing_offset_ + delta_to_target);
  return Error::kOk;
}

// ---- CRTP 分派 ----

template <typename DerivedType, uint8_t Bits>
Error Encoder<DerivedType, Bits>::ReadRaw(uint32_t& out_raw) {
  return static_cast<DerivedType*>(this)->ReadRawImpl(out_raw);
}

// ---- 辅助函数 ----

template <typename DerivedType, uint8_t Bits>
int32_t Encoder<DerivedType, Bits>::RawToLocalPosition(uint32_t raw, Reverse rev, uint32_t cpr) {
  return (rev == Reverse::kNormal) ? static_cast<int32_t>(raw)
                                   : static_cast<int32_t>((cpr - raw) % cpr);
}

template <typename DerivedType, uint8_t Bits>
int32_t Encoder<DerivedType, Bits>::SnapNearZero(int32_t pos_in_cpr,
                                                 int32_t cpr,
                                                 int32_t edge_threshold) {
  const int32_t snap_boundary = cpr - edge_threshold;
  return (pos_in_cpr >= snap_boundary) ? (pos_in_cpr - cpr) : pos_in_cpr;
}

// ---- 绝对定位（完整变换管线） ----

template <typename DerivedType, uint8_t Bits>
Error Encoder<DerivedType, Bits>::ResetPosition() {
  CHECK(ReadRaw(raw_pos_));
  constexpr auto cpr             = kResolution.kEncoderCpr;
  const int32_t  local_pos       = RawToLocalPosition(raw_pos_, reverse_, cpr);
  const int32_t  pos_with_offset = math::mod(local_pos + homing_offset_, cpr);
  pos_                           = SnapNearZero(pos_with_offset, cpr, kEdgeThreshold);
  return Error::kOk;
}

}  // namespace zeta::servo
