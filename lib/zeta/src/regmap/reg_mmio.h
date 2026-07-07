// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file reg_mmio.h
 * @brief 内存映射寄存器访问（直接读写内存区）
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "error.h"
#include "noncopyable.h"
#include "regmap/regmap_base.h"

namespace zeta::regmap {

/**
 * @brief 内存映射寄存器，用于 Regmap 的本地表（如 EEPROM 镜像）
 */
class RegMmio : public zeta::Noncopyable {
 public:
  /**
   * @brief 绑定寄存器基址与长度
   * @param regs 寄存器数组指针
   * @param size 数组长度 [byte]
   * @return 错误码
   */
  Error Init(uint8_t* regs, const size_t size);
  /**
   * @brief 写连续寄存器
   * @param address 起始地址
   * @param data 数据指针
   * @param size 字节数
   * @return 错误码
   */
  Error Write(const uint8_t address, const uint8_t* data, const size_t size);
  /**
   * @brief 读连续寄存器
   * @param address 起始地址
   * @param size 字节数
   * @param data 输出缓冲区
   * @return 错误码
   */
  Error Read(const uint8_t address, const size_t size, uint8_t* data);

 protected:
  uint8_t* regs_{};
  size_t   size_{};
};

}  // namespace zeta::regmap

namespace zeta::regmap {

inline Error RegMmio::Init(uint8_t* regs, const size_t size) {
  VERIFY(regs && size != 0, Error::kInvalidArg);
  regs_ = regs;
  size_ = size;
  return Error::kOk;
}

inline Error RegMmio::Write(const uint8_t address, const uint8_t* data, const size_t size) {
  VERIFY(regs_ && data, Error::kInvalidArg);
  VERIFY(size <= size_ && address <= size_ - size, Error::kInvalidArg);
  for (size_t i = 0; i < size; ++i) {
    regs_[address + i] = data[i];
  }
  return Error::kOk;
}

inline Error RegMmio::Read(const uint8_t address, const size_t size, uint8_t* data) {
  VERIFY(regs_ && data, Error::kInvalidArg);
  VERIFY(size <= size_ && address <= size_ - size, Error::kInvalidArg);
  for (size_t i = 0; i < size; ++i) {
    data[i] = regs_[address + i];
  }
  return Error::kOk;
}

}  // namespace zeta::regmap
