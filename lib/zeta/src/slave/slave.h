// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file slave.h
 * @brief 伺服从机（协议指令分发 + 舵机 + 寄存器映射，I2C/Serial 通道）
 *
 * 使用 Dispatcher 回调模式，无 CRTP 依赖
 */

#pragma once

#include <Arduino.h>

#include "error.h"
#include "noncopyable.h"
#include "protocol/dispatcher.h"
#include "protocol/transport/transport_i2c.h"
#include "slave/slave_regmap.h"

namespace zeta::slave {

/**
 * @brief 伺服从机：绑定舵机与寄存器映射，实现协议指令与寄存器读写
 * @tparam ServoType 舵机类型（如 Servo<...>）
 * @tparam TransportType 传输层类型（TransportI2C / TransportSerial）
 */
template <typename ServoType, typename TransportType>
class Slave : public zeta::Noncopyable {
 public:
  /** @brief 获取绑定的舵机实例 */
  ServoType* servo() { return servo_; }

  /**
   * @brief 获取从机 ID
   */
  uint8_t id() const { return dispatcher_.id(); }

  /** @brief 获取传输层（用于 I2C OnRequest 等回调） */
  TransportType* transport() { return dispatcher_.transport(); }

  /**
   * @brief 初始化从机，绑定舵机、寄存器映射与传输层
   * @param servo 舵机指针
   * @param regmap 寄存器映射指针
   * @param args 转发给 Transport::Init 的参数（如 TwoWire*）
   * @return 错误码
   */
  template <typename... Args>
  Error Init(ServoType* servo, Regmap* regmap, Args&&... args);

  /**
   * @brief 主循环：接收指令、分发、更新状态
   * @param dt 时间间隔 [s]
   * @return 错误码
   */
  Error Process(float dt);

  /**
   * @brief 每拍处理完指令后的钩子（写实时状态、更新寄存器）
   * @param dt 时间间隔 [s]
   * @return 错误码
   */
  Error AfterProcessImpl(float dt);

  /** @brief 将舵机状态写回寄存器映射 */
  Error UpdateStatus();

 private:
  // 指令处理器
  Error OnPing(const protocol::InstPacket& packet,
               protocol::StatusErrorBits&  status,
               uint8_t*                    response_data,
               size_t&                     response_size);

  Error OnReadData(const protocol::InstPacket& packet,
                   protocol::StatusErrorBits&  status,
                   uint8_t*                    response_data,
                   size_t&                     response_size);

  Error OnWriteData(const protocol::InstPacket& packet,
                    protocol::StatusErrorBits&  status,
                    uint8_t*                    response_data,
                    size_t&                     response_size);

  Error OnRegWrite(const protocol::InstPacket& packet,
                   protocol::StatusErrorBits&  status,
                   uint8_t*                    response_data,
                   size_t&                     response_size);

  Error OnAction(const protocol::InstPacket& packet,
                 protocol::StatusErrorBits&  status,
                 uint8_t*                    response_data,
                 size_t&                     response_size);

  Error OnSyncWrite(const protocol::InstPacket& packet,
                    protocol::StatusErrorBits&  status,
                    uint8_t*                    response_data,
                    size_t&                     response_size);

  Error OnBulkRead(const protocol::InstPacket& packet,
                   protocol::StatusErrorBits&  status,
                   uint8_t*                    response_data,
                   size_t&                     response_size);

  Error OnReset(const protocol::InstPacket& packet,
                protocol::StatusErrorBits&  status,
                uint8_t*                    response_data,
                size_t&                     response_size);

  // 私有辅助方法
  Error ApplyProtocolConfig();   ///< 从寄存器应用协议配置
  Error ApplyAlignToPosition();  ///< 执行对齐到目标位置
  Error ApplyMotorConfig();      ///< 从寄存器应用电机/舵机配置
  Error UpdateMotorStatus();     ///< 将舵机状态写回寄存器

  // 成员变量
  protocol::Dispatcher<TransportType> dispatcher_;               ///< 协议通道 + 指令分发器
  Regmap*                             regmap_        = nullptr;  ///< 寄存器映射
  ServoType*                          servo_         = nullptr;  ///< 舵机实例
  uint16_t                            realtime_tick_ = 0;        ///< 实时时钟 [ms]
  uint8_t async_write_buffer_[128]                   = {};       ///< RegWrite 暂存，Action 时写入
  size_t  async_write_buffer_size_                   = 0;        ///< 暂存长度
};

}  // namespace zeta::slave

namespace zeta::slave {

template <typename ServoType, typename TransportType>
template <typename... Args>
Error Slave<ServoType, TransportType>::Init(ServoType* servo, Regmap* regmap, Args&&... args) {
  servo_  = servo;
  regmap_ = regmap;

  // 初始化分发器（转发到 Transport::Init）
  CHECK(dispatcher_.Init(std::forward<Args>(args)...));
  dispatcher_.set_id(regmap_->ReadId());
  dispatcher_.set_return_level(regmap_->ReadStatusReturnLevel());

  // 注册 8 个指令处理器
  dispatcher_.RegisterHandler(protocol::Instruction::kPing,
                              [this](const auto& pkt, auto& status, auto* resp, auto& resp_sz) {
                                return OnPing(pkt, status, resp, resp_sz);
                              });

  dispatcher_.RegisterHandler(protocol::Instruction::kReadData,
                              [this](const auto& pkt, auto& status, auto* resp, auto& resp_sz) {
                                return OnReadData(pkt, status, resp, resp_sz);
                              });

  dispatcher_.RegisterHandler(protocol::Instruction::kWriteData,
                              [this](const auto& pkt, auto& status, auto* resp, auto& resp_sz) {
                                return OnWriteData(pkt, status, resp, resp_sz);
                              });

  dispatcher_.RegisterHandler(protocol::Instruction::kRegWrite,
                              [this](const auto& pkt, auto& status, auto* resp, auto& resp_sz) {
                                return OnRegWrite(pkt, status, resp, resp_sz);
                              });

  dispatcher_.RegisterHandler(protocol::Instruction::kAction,
                              [this](const auto& pkt, auto& status, auto* resp, auto& resp_sz) {
                                return OnAction(pkt, status, resp, resp_sz);
                              });

  dispatcher_.RegisterHandler(protocol::Instruction::kSyncWrite,
                              [this](const auto& pkt, auto& status, auto* resp, auto& resp_sz) {
                                return OnSyncWrite(pkt, status, resp, resp_sz);
                              });

  dispatcher_.RegisterHandler(protocol::Instruction::kBulkRead,
                              [this](const auto& pkt, auto& status, auto* resp, auto& resp_sz) {
                                return OnBulkRead(pkt, status, resp, resp_sz);
                              });

  dispatcher_.RegisterHandler(protocol::Instruction::kReset,
                              [this](const auto& pkt, auto& status, auto* resp, auto& resp_sz) {
                                return OnReset(pkt, status, resp, resp_sz);
                              });

  CHECK(ApplyProtocolConfig());
  CHECK(ApplyMotorConfig());
  CHECK(UpdateMotorStatus());
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::Process(float dt) {
  CHECK(dispatcher_.Process(dt));
  CHECK(AfterProcessImpl(dt));
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::OnPing(const protocol::InstPacket& packet,
                                              protocol::StatusErrorBits&  status,
                                              uint8_t*                    response_data,
                                              size_t&                     response_size) {
  (void)packet;
  (void)status;
  (void)response_data;
  response_size = 0;
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::OnReadData(const protocol::InstPacket& packet,
                                                  protocol::StatusErrorBits&  status,
                                                  uint8_t*                    response_data,
                                                  size_t&                     response_size) {
  (void)status;
  const uint8_t address = packet.parameter[0];
  const uint8_t size    = packet.parameter[1];
  CHECK(regmap_->Read(address, size, response_data));
  response_size = size;
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::OnWriteData(const protocol::InstPacket& packet,
                                                   protocol::StatusErrorBits&  status,
                                                   uint8_t*                    response_data,
                                                   size_t&                     response_size) {
  (void)status;
  (void)response_data;
  response_size         = 0;
  const uint8_t address = packet.parameter[0];
  const uint8_t size    = packet.GetParameterSize() - 1;
  CHECK(regmap_->Write(address, packet.parameter + 1, size));
  CHECK(ApplyProtocolConfig());
  CHECK(ApplyMotorConfig());
  CHECK(UpdateMotorStatus());
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::OnRegWrite(const protocol::InstPacket& packet,
                                                  protocol::StatusErrorBits&  status,
                                                  uint8_t*                    response_data,
                                                  size_t&                     response_size) {
  (void)status;
  (void)response_data;
  response_size     = 0;
  const size_t size = packet.GetBufferSize();
  if (async_write_buffer_size_ + size > sizeof(async_write_buffer_)) {
    status.range_error = true;
    return Error::kOutOfRange;
  }
  memcpy(async_write_buffer_ + async_write_buffer_size_, packet.buffer, size);
  async_write_buffer_size_ += size;
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::OnAction(const protocol::InstPacket& packet,
                                                protocol::StatusErrorBits&  status,
                                                uint8_t*                    response_data,
                                                size_t&                     response_size) {
  (void)packet;
  (void)response_data;
  response_size         = 0;
  const uint8_t* buffer = async_write_buffer_;
  if (async_write_buffer_size_ == 0) {
    status.instruction_error = true;
    return Error::kOk;
  }
  while (async_write_buffer_size_ > 0) {
    const protocol::InstPacket* const reg_write_packet =
        reinterpret_cast<const protocol::InstPacket*>(buffer);
    const size_t     packet_size    = reg_write_packet->GetBufferSize();
    constexpr size_t kMinPacketSize = 6;
    if (async_write_buffer_size_ < packet_size || packet_size < kMinPacketSize ||
        packet_size > protocol::InstPacket::kBufferCapacity) {
      break;
    }
    const uint8_t address = reg_write_packet->parameter[0];
    const uint8_t size    = reg_write_packet->GetParameterSize() - 1;
    CHECK(regmap_->Write(address, reg_write_packet->parameter + 1, size));
    async_write_buffer_size_ -= packet_size;
    buffer += packet_size;
  }
  async_write_buffer_size_ = 0;
  CHECK(ApplyProtocolConfig());
  CHECK(ApplyMotorConfig());
  CHECK(UpdateMotorStatus());
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::OnSyncWrite(const protocol::InstPacket& packet,
                                                   protocol::StatusErrorBits&  status,
                                                   uint8_t*                    response_data,
                                                   size_t&                     response_size) {
  (void)status;
  (void)response_data;
  response_size                 = 0;
  const uint8_t  address        = packet.parameter[0];
  const uint8_t  data_size      = packet.parameter[1];
  const uint8_t  parameter_size = packet.GetParameterSize();
  const uint8_t  block_size     = data_size + 1;
  const uint8_t  block_count    = (parameter_size - 2) / block_size;
  const uint8_t* parameter      = packet.parameter + 2;
  for (uint8_t i = 0; i < block_count; i++) {
    const uint8_t target_id = parameter[0];
    if (dispatcher_.id() == target_id) {
      const uint8_t* data = parameter + 1;
      CHECK(regmap_->Write(address, data, data_size));
    }
    parameter += block_size;
  }
  CHECK(ApplyProtocolConfig());
  CHECK(ApplyMotorConfig());
  CHECK(UpdateMotorStatus());
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::OnBulkRead(const protocol::InstPacket& packet,
                                                  protocol::StatusErrorBits&  status,
                                                  uint8_t*                    response_data,
                                                  size_t&                     response_size) {
  (void)status;
  const uint8_t  parameter_size = packet.GetParameterSize();
  const uint8_t  block_count    = (parameter_size - 1) / 3;
  const uint8_t* parameter      = packet.parameter + 1;
  for (uint8_t i = 0; i < block_count; i++) {
    const uint8_t target_id = parameter[1];
    if (dispatcher_.id() == target_id) {
      const uint8_t data_size = parameter[0];
      const uint8_t address   = parameter[2];
      CHECK(regmap_->Read(address, data_size, response_data));
      response_size = data_size;
      return Error::kOk;
    }
    parameter += 3;
  }
  response_size = 0;
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::OnReset(const protocol::InstPacket& packet,
                                               protocol::StatusErrorBits&  status,
                                               uint8_t*                    response_data,
                                               size_t&                     response_size) {
  (void)packet;
  (void)status;
  (void)response_data;
  response_size = 0;
  CHECK(regmap_->RecoveryEeprom());
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::AfterProcessImpl(float dt) {
  realtime_tick_ += dt * 1000;
  regmap_->WriteRealtimeTick(realtime_tick_);
  CHECK(UpdateMotorStatus());
  return UpdateStatus();
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::UpdateStatus() {
  dispatcher_.status().input_voltage_error = servo_->hardware_input_voltage_error();
  dispatcher_.status().angle_limit_error   = servo_->hardware_angle_limit_error();
  dispatcher_.status().overheating_error   = servo_->hardware_overheating_error();
  dispatcher_.status().range_error         = servo_->hardware_range_error();
  dispatcher_.status().overload_error      = servo_->hardware_overload_error();
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::ApplyProtocolConfig() {
  dispatcher_.set_id(regmap_->ReadId());
  dispatcher_.set_return_level(regmap_->ReadStatusReturnLevel());
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::ApplyAlignToPosition() {
  const auto align_to_position = regmap_->ReadAlignToPosition();
  if (align_to_position) {
    servo_->AlignToPosition(align_to_position);
    regmap_->WriteAlignToPosition(0);
  }
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::ApplyMotorConfig() {
  ControlBits::DriveModeBits drive_mode{};
  drive_mode.value = regmap_->ReadDriveMode();
  servo_->set_motor_reverse_mode(drive_mode.motor_reverse_mode);
  servo_->set_encoder_reverse_mode(drive_mode.encoder_reverse_mode);

  const auto operating_mode = regmap_->ReadOperatingMode();
  servo_->set_operating_mode(operating_mode);

  ControlBits::ShutdownBits shutdown{};
  shutdown.value = regmap_->ReadShutdown();
  servo_->set_shutdown_input_voltage_error(shutdown.input_voltage_error);
  servo_->set_shutdown_overheating_error(shutdown.overheating_error);
  servo_->set_shutdown_motor_encoder_error(shutdown.motor_encoder_error);
  servo_->set_shutdown_electrical_shock_error(shutdown.electrical_shock_error);
  servo_->set_shutdown_overload_error(shutdown.overload_error);

  const auto homing_offset = regmap_->ReadHomingOffset();
  servo_->set_homing_offset(homing_offset);

  const auto moving_threshold = regmap_->ReadMovingThreshold();
  servo_->set_moving_threshold(moving_threshold);

  const auto temperature_limit = regmap_->ReadTemperatureLimit();
  servo_->set_temperature_limit(temperature_limit);

  const auto max_voltage_limit = regmap_->ReadMaxVoltageLimit();
  servo_->set_max_voltage_limit(max_voltage_limit);

  const auto min_voltage_limit = regmap_->ReadMinVoltageLimit();
  servo_->set_min_voltage_limit(min_voltage_limit);

  const auto pwm_limit = regmap_->ReadPwmLimit();
  servo_->set_pwm_limit(pwm_limit);

  const auto current_limit = regmap_->ReadCurrentLimit();
  servo_->set_current_limit(current_limit);

  const auto velocity_limit = regmap_->ReadVelocityLimit();
  servo_->set_velocity_limit(velocity_limit);

  const auto max_position_limit = regmap_->ReadMaxPositionLimit();
  servo_->set_max_position_limit(max_position_limit);

  const auto min_position_limit = regmap_->ReadMinPositionLimit();
  servo_->set_min_position_limit(min_position_limit);

  const auto protection_time = regmap_->ReadProtectionTime();
  servo_->set_protection_time(protection_time);

  const auto vi = regmap_->ReadVelocityIGain();
  const auto vp = regmap_->ReadVelocityPGain();
  servo_->set_velocity_pid(vp, vi, 0);

  const auto pi = regmap_->ReadPositionIGain();
  const auto pp = regmap_->ReadPositionPGain();
  const auto pd = regmap_->ReadPositionDGain();
  servo_->set_position_pid(pp, pi, pd);

  const auto feedforward_2nd_gain = regmap_->ReadFeedforward2ndGain();
  servo_->set_feedforward_2nd_gain(feedforward_2nd_gain);

  const auto feedforward_1st_gain = regmap_->ReadFeedforward1stGain();
  servo_->set_feedforward_1st_gain(feedforward_1st_gain);

  const auto profile_acceleration = regmap_->ReadProfileAcceleration();
  servo_->set_profile_acceleration(profile_acceleration);

  const auto profile_velocity = regmap_->ReadProfileVelocity();
  servo_->set_profile_velocity(profile_velocity);

  const auto torque_enable = regmap_->ReadTorqueEnable();
  servo_->set_torque_enable(torque_enable);

  const auto goal_pwm = regmap_->ReadGoalPwm();
  servo_->set_goal_pwm(goal_pwm);

  const auto goal_current = regmap_->ReadGoalCurrent();
  servo_->set_goal_current(goal_current);

  const auto goal_velocity = regmap_->ReadGoalVelocity();
  servo_->set_goal_velocity(goal_velocity);

  const auto goal_position = regmap_->ReadGoalPosition();
  servo_->set_goal_position(goal_position);
  return Error::kOk;
}

template <typename ServoType, typename TransportType>
Error Slave<ServoType, TransportType>::UpdateMotorStatus() {
  const auto torque_enable = servo_->torque_enable();
  regmap_->WriteTorqueEnable(torque_enable);

  ControlBits::HardwareErrorStatusBits hardware_error_status{};
  hardware_error_status.input_voltage_error    = servo_->hardware_input_voltage_error();
  hardware_error_status.overheating_error      = servo_->hardware_overheating_error();
  hardware_error_status.motor_encoder_error    = servo_->hardware_motor_encoder_error();
  hardware_error_status.electrical_shock_error = servo_->hardware_electrical_shock_error();
  hardware_error_status.overload_error         = servo_->hardware_overload_error();
  hardware_error_status.angle_limit_error      = servo_->hardware_angle_limit_error();
  hardware_error_status.range_error            = servo_->hardware_range_error();
  regmap_->WriteHardwareErrorStatus(hardware_error_status.value);

  const auto moving = servo_->moving();
  regmap_->WriteMoving(moving);

  ControlBits::MovingStatusBits moving_status{};
  moving_status.in_position     = servo_->moving_in_position();
  moving_status.profile_ongoing = servo_->moving_profile_ongoing();
  moving_status.following_error = servo_->moving_following_error();
  moving_status.profile_type    = servo_->moving_profile_type();
  regmap_->WriteMovingStatus(moving_status.value);

  const auto present_position = servo_->present_position();
  regmap_->WritePresentPosition(present_position);

  const auto present_velocity = servo_->present_velocity();
  regmap_->WritePresentVelocity(present_velocity);

  const auto present_current = servo_->present_current();
  regmap_->WritePresentCurrent(present_current);

  const auto present_input_voltage = servo_->present_input_voltage();
  regmap_->WritePresentInputVoltage(present_input_voltage);

  const auto present_temperature = servo_->present_temperature();
  regmap_->WritePresentTemperature(present_temperature);

  const auto present_pwm = servo_->present_pwm();
  regmap_->WritePresentPwm(present_pwm);

  const auto velocity_trajectory = servo_->velocity_trajectory();
  regmap_->WriteVelocityTrajectory(velocity_trajectory);

  const auto position_trajectory = servo_->position_trajectory();
  regmap_->WritePositionTrajectory(position_trajectory);

  return Error::kOk;
}

}  // namespace zeta::slave
