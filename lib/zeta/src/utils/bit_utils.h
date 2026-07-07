// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file bit_utils.h
 * @brief 位操作工具（掩码、置位、清位、创建掩码）
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace zeta::utils::bit_utils {

/**
 * @brief 获取特定位掩码的值
 * @param data 数据
 * @param mask 位掩码
 * @return 特定位掩码的值
 */
template <typename T>
constexpr T GetValue(const T data, const T mask) noexcept {
  return data & mask;
}

/**
 * @brief 检查位掩码是否设置
 * @param data 数据
 * @param mask 位掩码
 * @return 位掩码是否设置
 */
template <typename T>
constexpr bool IsMaskSet(const T data, const T mask) noexcept {
  return (data & mask) != 0;
}

/**
 * @brief 检查特定位是否设置
 * @param data 数据
 * @param idx 位索引
 * @return 特定位是否设置
 */
template <typename T>
constexpr bool IsBitSet(const T data, const uint8_t idx) noexcept {
  return (data & (1 << idx)) != 0;
}

/**
 * @brief 设置单个位
 * @param data 数据
 * @param mask 位掩码
 * @return 设置后的数据
 */
template <typename T>
constexpr T SetMaskBit(const T data, const T mask) noexcept {
  return data | mask;
}

/**
 * @brief 设置特定位
 * @param data 数据
 * @param idx 位索引
 * @return 设置后的数据
 */
template <typename T>
constexpr T SetBit(const T data, const uint8_t idx) noexcept {
  return data | (1 << idx);
}

/**
 * @brief 清除单个位
 * @param data 数据
 * @param mask 位掩码
 * @return 清除后的数据
 */
template <typename T>
constexpr T ClearMaskBit(const T data, const T mask) noexcept {
  return data & ~mask;
}

/**
 * @brief 清除特定位
 * @param data 数据
 * @param idx 位索引
 * @return 清除后的数据
 */
template <typename T>
constexpr T ClearBit(const T data, const uint8_t idx) noexcept {
  return data & ~(1 << idx);
}

/**
 * @brief 切换位的状态
 * @param data 数据
 * @param mask 位掩码
 * @return 切换后的数据
 */
template <typename T>
constexpr T ToggleMaskBit(const T data, const T mask) noexcept {
  return data ^ mask;
}

/**
 * @brief 切换特定位
 * @param data 数据
 * @param idx 位索引
 * @return 切换后的数据
 */
template <typename T>
constexpr T ToggleBit(const T data, const uint8_t idx) noexcept {
  return data ^ (1 << idx);
}

/**
 * @brief 创建位掩码
 * @param startBit 起始位
 * @param numBits 位宽
 * @return 位掩码
 */
constexpr size_t CreateMask(const uint8_t startBit, const uint8_t numBits) noexcept {
  return ((1ULL << numBits) - 1) << startBit;
}

/**
 * @brief 将两个字节拼接为uint16_t
 * @param highByte 高字节
 * @param lowByte 低字节
 * @return 拼接后的uint16_t值
 */
constexpr uint16_t CombineToUint16(const uint8_t highByte, const uint8_t lowByte) noexcept {
  return (static_cast<uint16_t>(highByte) << 8) | lowByte;
}

/**
 * @brief 将两个字节拼接为int16_t
 * @param highByte 高字节
 * @param lowByte 低字节
 * @return 拼接后的int16_t值
 */
constexpr int16_t CombineToInt16(const uint8_t highByte, const uint8_t lowByte) noexcept {
  return static_cast<int16_t>(CombineToUint16(highByte, lowByte));
}

/**
 * @brief 将补码转换为原码
 * @param value 补码值（int16_t）
 * @param sign 符号位索引（默认 15）
 * @return 原码值（uint16_t，最高位为符号位）
 */
constexpr uint16_t TwosToSign(const int16_t value, const uint8_t sign = 15) noexcept {
  if (value >= 0) {
    return static_cast<uint16_t>(value);
  }
  return static_cast<uint16_t>(SetBit(-value, sign));
}

/**
 * @brief 将原码转换为补码
 * @param value 原码值（uint16_t，最高位为符号位）
 * @param sign 符号位索引（默认 15）
 * @return 补码值（int16_t）
 */
constexpr int16_t SignToTwos(const uint16_t value, const uint8_t sign = 15) noexcept {
  if (!IsBitSet(value, sign)) {
    return static_cast<int16_t>(value);
  }
  return -static_cast<int16_t>(ClearBit(value, sign));
}

}  // namespace zeta::utils::bit_utils