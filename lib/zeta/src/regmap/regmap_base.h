// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file regmap_base.h
 * @brief 寄存器映射基类，按地址/字段读写
 */

#pragma once

#include "error.h"
#include "noncopyable.h"
#include "regmap/reg_field.h"

namespace zeta::regmap {

/**
 * @brief 寄存器映射基类，通过传输层读写寄存器
 * @tparam TransportType 传输层类型（如 RegMmio、RegI2c）
 */
template <typename TransportType>
class Regmap : public zeta::Noncopyable {
 public:
  /** @brief 传输层引用 */
  TransportType& transport();

  /**
   * @brief 写单个寄存器（按类型宽度）
   * @tparam T 数据类型（uint8_t/uint16_t/uint32_t 等）
   * @param address 寄存器地址
   * @param data 写入值
   * @return 错误码
   */
  template <typename T>
  Error Write(const uint8_t address, const T data);

  /**
   * @brief 写连续寄存器
   * @param address 起始地址
   * @param data 数据指针
   * @param size 字节数
   * @return 错误码
   */
  Error Write(const uint8_t address, const uint8_t* data, const size_t size);

  /**
   * @brief 读单个寄存器
   * @tparam T 数据类型
   * @param address 寄存器地址
   * @param data 输出值
   * @return 错误码
   */
  template <typename T>
  Error Read(const uint8_t address, T& data);

  /**
   * @brief 读连续寄存器
   * @param address 起始地址
   * @param size 字节数
   * @param data 输出缓冲区
   * @return 错误码
   */
  Error Read(const uint8_t address, const size_t size, uint8_t* data);

  /**
   * @brief 按字段描述写（单寄存器位域）
   * @tparam T 控制表项类型（含 Field、Converter）
   * @param value 写入值（用户类型）
   * @return 错误码
   */
  template <typename T>
  Error WriteField(typename Trait::WriteValueTypeOf<T>::Type value);

  /**
   * @brief 按字段描述写（跨两寄存器合并字段）
   * @tparam T 合并后的值类型
   * @tparam HighFieldType 高字节字段类型
   * @tparam LowFieldType 低字节字段类型
   * @param value 写入值
   * @return 错误码
   */
  template <typename T, typename HighFieldType, typename LowFieldType>
  Error WriteField(T value);

  /**
   * @brief 按字段描述读（单寄存器位域）
   * @tparam T 控制表项类型
   * @param value 输出值（用户类型）
   * @return 错误码
   */
  template <typename T>
  Error ReadField(typename Trait::ReadValueTypeOf<T>::Type& value);

  /**
   * @brief 按字段描述读（跨两寄存器合并字段）
   * @tparam T 合并后的值类型
   * @tparam HighFieldType 高字节字段类型
   * @tparam LowFieldType 低字节字段类型
   * @param value 输出值
   * @return 错误码
   */
  template <typename T, typename HighFieldType, typename LowFieldType>
  Error ReadField(T& value);

 protected:
  TransportType transport_;
};

}  // namespace zeta::regmap

namespace zeta::regmap {

template <typename TransportType>
TransportType& Regmap<TransportType>::transport() {
  return transport_;
}

template <typename TransportType>
template <typename T>
Error Regmap<TransportType>::Write(const uint8_t address, const T data) {
  return Write(address, reinterpret_cast<const uint8_t*>(&data), sizeof(T));
}

template <typename TransportType>
Error Regmap<TransportType>::Write(const uint8_t address, const uint8_t* data, const size_t size) {
  return transport_.Write(address, data, size);
}

template <typename TransportType>
template <typename T>
Error Regmap<TransportType>::Read(const uint8_t address, T& data) {
  return Read(address, sizeof(T), reinterpret_cast<uint8_t*>(&data));
}

template <typename TransportType>
Error Regmap<TransportType>::Read(const uint8_t address, const size_t size, uint8_t* data) {
  return transport_.Read(address, size, data);
}

template <typename TransportType>
template <typename T>
Error Regmap<TransportType>::WriteField(typename Trait::WriteValueTypeOf<T>::Type value) {
  using FieldBase = typename Trait::FieldOf<T>::Type;
  typename FieldBase::access_t access;
  CHECK(Read(FieldBase::kAddress, access));
  const auto raw = Trait::EncodePipeline<T>::Run(value);
  FieldBase::SetValue(raw, access);
  CHECK(Write(FieldBase::kAddress, access));
  return Error::kOk;
}

template <typename TransportType>
template <typename T, typename HighFieldType, typename LowFieldType>
Error Regmap<TransportType>::WriteField(T value) {
  typename HighFieldType::access_t high_access;
  typename LowFieldType::access_t  low_access;
  CHECK(Read(HighFieldType::kAddress, high_access));
  CHECK(Read(LowFieldType::kAddress, low_access));
  Merged2<T, HighFieldType, LowFieldType>::SetValue(value, high_access, low_access);
  CHECK(Write(HighFieldType::kAddress, high_access));
  CHECK(Write(LowFieldType::kAddress, low_access));
  return Error::kOk;
}

template <typename TransportType>
template <typename T>
Error Regmap<TransportType>::ReadField(typename Trait::ReadValueTypeOf<T>::Type& value) {
  using FieldBase = typename Trait::FieldOf<T>::Type;
  typename FieldBase::access_t access;
  CHECK(Read(FieldBase::kAddress, access));
  const auto raw = FieldBase::GetValue(access);
  value          = Trait::DecodePipeline<T>::Run(raw);
  return Error::kOk;
}

template <typename TransportType>
template <typename T, typename HighFieldType, typename LowFieldType>
Error Regmap<TransportType>::ReadField(T& value) {
  typename HighFieldType::access_t high_access;
  typename LowFieldType::access_t  low_access;
  CHECK(Read(HighFieldType::kAddress, high_access));
  CHECK(Read(LowFieldType::kAddress, low_access));
  value = Merged2<T, HighFieldType, LowFieldType>::GetValue(high_access, low_access);
  return Error::kOk;
}

}  // namespace zeta::regmap
