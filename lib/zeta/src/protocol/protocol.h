// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file protocol.h
 * @brief DYNAMIXEL Protocol 1.0 指令包解析与状态包构造
 */

#pragma once

#include <Arduino.h>

#include "error.h"
#include "noncopyable.h"
#include "protocol/protocol_types.h"

namespace zeta::protocol {

/**
 * @brief 指令包解析状态机
 * @see https://emanual.robotis.com/docs/en/dxl/protocol1/
 */
enum class PacketState : uint8_t {
  kHeader1,
  kHeader2,
  kId,
  kLength,
  kInstructionOrError,
  kParameter,
  kChecksum,
};

/// @brief 指令包/状态包缓冲区（Dynamixel 协议格式）
struct __attribute__((packed)) InstPacket {
  static constexpr uint8_t kBufferCapacity  = 128;
  static constexpr uint8_t kParameterOffset = 5;
  union {
    uint8_t buffer[kBufferCapacity];
    struct {
      uint8_t header1;
      uint8_t header2;
      uint8_t id;
      uint8_t length;
      uint8_t instructionOrError;
      uint8_t parameter[kBufferCapacity - kParameterOffset];
    };
  };
  uint8_t GetId() const;
  uint8_t GetLength() const;
  uint8_t GetParameterSize() const;
  void    SetParameterSize(const uint8_t size);
  uint8_t GetChecksum() const;
  void    SetChecksum(uint8_t checksum);
  size_t  GetBufferSize() const;
  uint8_t CalculateChecksum() const;
};

using StatusPacket = InstPacket;

/// @brief 指令包解析器与状态包构造
class Protocol : public zeta::Noncopyable {
 public:
  /**
   * @brief 喂入一字节，推进解析状态
   * @param packet 当前解析中的包
   * @param recv_data 本拍接收字节
   * @param is_complete 输出：是否收完一整包
   * @return 错误码
   */
  Error Process(InstPacket& packet, const uint8_t recv_data, bool& is_complete);

  /**
   * @brief 构造状态包
   * @param id 从机 ID
   * @param status 状态错误位
   * @param parameter 参数区指针，可为 nullptr
   * @param parameter_size 参数区长度
   * @param packet 输出的状态包
   * @return 错误码
   */
  Error CreateResponse(const uint8_t          id,
                       const StatusErrorBits& status,
                       const uint8_t*         parameter,
                       const size_t           parameter_size,
                       StatusPacket&          packet);

 private:
  uint8_t     param_pos_    = 0;
  PacketState packet_state_ = PacketState::kHeader1;
};

}  // namespace zeta::protocol

namespace zeta::protocol {

inline uint8_t InstPacket::GetId() const {
  return buffer[PacketIndex::kId];
}

inline uint8_t InstPacket::GetLength() const {
  return length;
}

inline uint8_t InstPacket::GetParameterSize() const {
  return GetLength() - 2;
}

inline void InstPacket::SetParameterSize(const uint8_t size) {
  length = size + 2;
}

inline uint8_t InstPacket::GetChecksum() const {
  const uint8_t parameter_size = GetParameterSize();
  return buffer[PacketIndex::kParameter + parameter_size];
}

inline void InstPacket::SetChecksum(uint8_t checksum) {
  const uint8_t parameter_size                     = GetParameterSize();
  buffer[PacketIndex::kParameter + parameter_size] = checksum;
}

inline size_t InstPacket::GetBufferSize() const {
  return length + 4;
}

inline uint8_t InstPacket::CalculateChecksum() const {
  uint8_t       checksum = 0;
  const uint8_t end      = PacketIndex::kParameter + GetParameterSize();
  for (uint8_t i = PacketIndex::kId; i < end; i++) {
    checksum += buffer[i];
  }
  return ~checksum;
}

inline Error Protocol::Process(InstPacket& packet, const uint8_t recv_data, bool& is_complete) {
  is_complete = false;
  switch (packet_state_) {
    case PacketState::kHeader1: {
      if (recv_data == kHeaderByte) {
        packet_state_  = PacketState::kHeader2;
        packet.header1 = recv_data;
      }
      break;
    }
    case PacketState::kHeader2: {
      if (recv_data == kHeaderByte) {
        packet_state_  = PacketState::kId;
        packet.header2 = recv_data;
      } else {
        packet_state_ = PacketState::kHeader1;
        return Error::kBadData;
      }
      break;
    }
    case PacketState::kId: {
      packet.id     = recv_data;
      packet_state_ = PacketState::kLength;
      break;
    }
    case PacketState::kLength: {
      if (recv_data + 4 > InstPacket::kBufferCapacity || recv_data < 2) {
        packet_state_ = PacketState::kHeader1;
        return Error::kBadData;
      } else {
        packet.length = recv_data;
        packet_state_ = PacketState::kInstructionOrError;
      }
      break;
    }
    case PacketState::kInstructionOrError: {
      packet.instructionOrError = recv_data;
      const uint8_t param_size  = packet.GetParameterSize();
      if (param_size > 0) {
        param_pos_    = 0;
        packet_state_ = PacketState::kParameter;
      } else {
        packet_state_ = PacketState::kChecksum;
      }
      break;
    }
    case PacketState::kParameter: {
      const uint8_t param_size     = packet.GetParameterSize();
      packet.parameter[param_pos_] = recv_data;
      param_pos_ += 1;
      if (param_pos_ == param_size) {
        packet_state_ = PacketState::kChecksum;
      }
      break;
    }
    case PacketState::kChecksum: {
      const uint8_t checksum = packet.CalculateChecksum();
      packet.SetChecksum(recv_data);
      is_complete = true;
      if (checksum != recv_data) {
        packet_state_ = PacketState::kHeader1;
        return Error::kBadData;
      }
      packet_state_ = PacketState::kHeader1;
      break;
    }
    default: {
      packet_state_ = PacketState::kHeader1;
      return Error::kBadState;
    }
  }
  return Error::kOk;
}

inline Error Protocol::CreateResponse(const uint8_t          id,
                                      const StatusErrorBits& status,
                                      const uint8_t*         parameter,
                                      const size_t           parameter_size,
                                      StatusPacket&          packet) {
  packet.header1            = kHeaderByte;
  packet.header2            = kHeaderByte;
  packet.id                 = id;
  packet.instructionOrError = status.value;
  if (parameter == nullptr) {
    packet.SetParameterSize(0);
  } else {
    packet.SetParameterSize(parameter_size);
    memcpy(packet.parameter, parameter, parameter_size);
  }
  packet.SetChecksum(packet.CalculateChecksum());
  return Error::kOk;
}

}  // namespace zeta::protocol