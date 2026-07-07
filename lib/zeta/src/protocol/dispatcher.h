// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dispatcher.h
 * @brief 协议通道 + 指令分发器
 *
 * 集成了协议通道（Transport + 解析器，收包/发包）与指令分发（回调驱动）。
 *
 * 功能：
 * - 持有 Transport，完成字节级收发
 * - 持有 Protocol 解析器，完成 DYNAMIXEL 协议包解析/构造
 * - 管理指令处理器回调表
 * - 接收完整指令包后分发到对应回调
 * - 管理协议参数（ID、返回级别、状态）
 * - 生成并发送响应（支持延迟响应，Serial 多从机时使用）
 *
 * 特点：
 * - 不涉及寄存器操作，完全由回调处理业务逻辑
 * - Handler 直接填充响应数据
 */

#pragma once

#include <Arduino.h>

#include <functional>

#include "error.h"
#include "noncopyable.h"
#include "protocol/protocol.h"
#include "protocol/protocol_types.h"

namespace zeta::protocol {

/**
 * @brief 指令处理器回调签名
 *
 * @param packet 指令包（只读）
 * @param status 状态错误位（可修改）
 * @param response_data 响应数据缓冲（Handler 填充）
 * @param response_size 响应数据大小（Handler 设置）
 * @return 错误码
 */
using InstHandler = std::function<Error(const InstPacket&, StatusErrorBits&, uint8_t*, size_t&)>;

/**
 * @brief 处理器注册表项
 */
struct HandlerEntry {
  uint8_t     instruction = 0;
  InstHandler handler;
};

/**
 * @brief 协议通道 + 指令分发器
 *
 * 集协议收发与指令分发于一体。Transport 的 Init 参数通过本类的 Init 转发。
 *
 * @tparam TransportType 传输层类型（TransportI2C / TransportSerial）
 */
template <typename TransportType>
class Dispatcher : public zeta::Noncopyable {
 public:
  static constexpr uint8_t kMaxHandlers = 16;

  // ── 通道接口（原 Channel） ──

  /**
   * @brief 初始化传输层（I2C 须在 wire.begin 前调用）
   * @param args 转发给 Transport::Init 的参数（如 TwoWire* 或 HardwareSerial*）
   */
  template <typename... Args>
  Error Init(Args&&... args) {
    return transport_.Init(std::forward<Args>(args)...);
  }

  /**
   * @brief 设置响应延迟 [s]，Serial 多从机时使用
   * @param response_delay 延迟时间 [s]
   */
  void SetResponseDelay(const float response_delay) { response_delay_ = response_delay; }

  /** @brief 当前收到的指令包（只读） */
  const InstPacket& inst_packet() const { return inst_packet_; }

  /** @brief 协议解析器，用于构造响应包 */
  Protocol* parser() { return &parser_; }

  /** @brief 传输层访问 */
  TransportType*       transport() { return &transport_; }
  const TransportType* transport() const { return &transport_; }

  // ── 分发器接口 ──

  /**
   * @brief 主循环：收包 → 分发 → 回复
   * @param dt 时间间隔（秒）
   * @return 错误码
   */
  Error Process(float dt);

  /**
   * @brief 注册指令处理器回调
   * @param instruction 指令码
   * @param handler 处理器回调函数
   */
  void RegisterHandler(uint8_t instruction, const InstHandler& handler);

  /**
   * @brief 设置从机 ID
   * @param id 从机 ID (0–252)
   */
  void set_id(uint8_t id) { id_ = id; }

  /** @brief 获取从机 ID */
  uint8_t id() const { return id_; }

  /**
   * @brief 设置返回级别 (0/1/2)
   * @param level 返回级别
   */
  void set_return_level(uint8_t level) { return_level_ = level; }

  /** @brief 获取返回级别 */
  uint8_t return_level() const { return return_level_; }

  /**
   * @brief 获取状态错误位（可修改）
   * @return 状态引用
   */
  StatusErrorBits& status() { return status_; }

  /**
   * @brief 发送响应包
   * @param parameter 参数区指针（可为 nullptr）
   * @param parameter_size 参数区长度
   * @param reply_idx 回复索引（用于多回复延迟）
   * @return 错误码
   */
  Error SendResponse(const uint8_t* parameter, size_t parameter_size, uint8_t reply_idx = 0);

 private:
  // ── 通道层成员 ──
  TransportType transport_;
  Protocol      parser_;
  InstPacket    inst_packet_{};
  StatusPacket  pending_packet_{};  ///< 延迟待发送的响应包缓冲
  float         response_delay_   = 0.0f;
  float         delay_remaining_  = 0.0f;
  bool          response_pending_ = false;

  // ── 分发器层成员 ──
  uint8_t         id_           = 0;
  uint8_t         return_level_ = 0;
  StatusErrorBits status_{};
  HandlerEntry    handlers_[kMaxHandlers]{};
  uint8_t         handler_count_ = 0;

  // ── 分发器私有方法 ──

  /**
   * @brief 发送响应包（内部方法，直接写入 transport 或延迟）
   * @param packet 状态包
   * @param reply_idx 回复索引（用于多回复延迟）
   * @return 错误码
   */
  Error Response(const StatusPacket& packet, const uint8_t reply_idx);

  /**
   * @brief 根据 return_level_ 判断是否需要回复
   * @param instruction 指令码
   * @return 需要回复为 true
   */
  bool CheckResponse(uint8_t instruction) const;

  /**
   * @brief 在注册表中查找指令处理器
   * @param instruction 指令码
   * @return 处理器指针，未找到返回 nullptr
   */
  const InstHandler* FindHandler(uint8_t instruction) const;
};

}  // namespace zeta::protocol

namespace zeta::protocol {

// ── 通道层实现 ──

template <typename TransportType>
Error Dispatcher<TransportType>::Process(float dt) {
  // 延迟响应发送
  if (response_pending_) {
    delay_remaining_ -= dt;
    if (delay_remaining_ <= 0.0f) {
      response_pending_ = false;
      const size_t size = pending_packet_.GetBufferSize();
      CHECK(transport_.Write(pending_packet_.buffer, size));
    }
  }

  // 收包解析
  bool is_complete = false;
  while (transport_.Available() > 0) {
    uint8_t byte = 0;
    if (!transport_.ReadByte(byte)) {
      break;
    }
    CHECK(parser_.Process(inst_packet_, byte, is_complete));
    if (is_complete) {
      break;
    }
  }

  if (!is_complete) {
    return Error::kOk;
  }

  // ── 分发 ──
  const auto& packet       = inst_packet_;
  const auto  is_broadcast = packet.id == kBroadcastId;
  const auto  is_self      = packet.id == id_;

  if (!is_self && !is_broadcast) {
    return Error::kOk;
  }

  const auto instruction = packet.instructionOrError;

  // 在注册表中查找指令
  const auto* handler = FindHandler(instruction);

  // 重置状态
  status_.instruction_error = false;
  status_.range_error       = false;

  if (handler == nullptr) {
    // 未注册的指令
    status_.instruction_error = true;
    if (CheckResponse(instruction)) {
      CHECK(SendResponse(nullptr, 0));
    }
    return Error::kBadData;
  }

  // 调用回调处理
  uint8_t response_data[128]{};
  size_t  response_size = 0;
  CHECK((*handler)(packet, status_, response_data, response_size));

  // 如果需要回复
  if (CheckResponse(instruction)) {
    CHECK(SendResponse(response_data, response_size));
  }

  return Error::kOk;
}

template <typename TransportType>
void Dispatcher<TransportType>::RegisterHandler(uint8_t instruction, const InstHandler& handler) {
  if (handler_count_ < kMaxHandlers) {
    handlers_[handler_count_].instruction = instruction;
    handlers_[handler_count_].handler     = handler;
    handler_count_++;
  }
}

template <typename TransportType>
Error Dispatcher<TransportType>::SendResponse(const uint8_t* parameter,
                                              size_t         parameter_size,
                                              uint8_t        reply_idx) {
  StatusPacket packet;
  CHECK(parser_.CreateResponse(id_, status_, parameter, parameter_size, packet));
  CHECK(Response(packet, reply_idx));
  return Error::kOk;
}

template <typename TransportType>
Error Dispatcher<TransportType>::Response(const StatusPacket& packet, const uint8_t reply_idx) {
  memcpy(pending_packet_.buffer, packet.buffer, packet.GetBufferSize());

  const float delay = response_delay_ * static_cast<float>(reply_idx + 1);
  if (delay <= 0.0f) {
    const size_t size = pending_packet_.GetBufferSize();
    return transport_.Write(pending_packet_.buffer, size);
  }
  delay_remaining_  = delay;
  response_pending_ = true;
  return Error::kOk;
}

template <typename TransportType>
bool Dispatcher<TransportType>::CheckResponse(uint8_t instruction) const {
  switch (return_level_) {
    case 0:
      return instruction == Instruction::kPing;
    case 1:
      return instruction == Instruction::kPing || instruction == Instruction::kReadData ||
             instruction == Instruction::kBulkRead;
    case 2:
      return true;
  }
  return false;
}

template <typename TransportType>
const InstHandler* Dispatcher<TransportType>::FindHandler(uint8_t instruction) const {
  for (uint8_t i = 0; i < handler_count_; i++) {
    if (handlers_[i].instruction == instruction) {
      return &handlers_[i].handler;
    }
  }
  return nullptr;
}

}  // namespace zeta::protocol