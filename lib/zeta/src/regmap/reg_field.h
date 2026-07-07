// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file reg_field.h
 * @brief 寄存器位域与类型转换（Field、Merged2、Converter、Trait）
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <tuple>
#include <type_traits>

#include "noncopyable.h"
#include "utils/bit_utils.h"

namespace zeta::regmap {

/// @brief 寄存器位域描述（见下方定义）
template <typename ValueType, uint8_t Address, uint8_t Shift, uint8_t Bits>
struct Field;

/// @brief 跨两寄存器的合并字段（见下方定义）
template <typename ValueType, typename HighFieldType, typename LowFieldType>
struct Merged2;

/// @brief 原始值与用户值转换（见下方定义）
template <typename ValueType>
struct Converter;

struct Trait;

}  // namespace zeta::regmap

namespace zeta::regmap {

/// @brief 寄存器字段元数据与编解码管道（FieldOf、ConverterOf、EncodePipeline 等）
struct Trait : public zeta::Noncopyable {
  template <typename T>
  struct AccessType {
    static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                  "Unsupported RegField element size");

    static constexpr size_t kIndex = sizeof(T) == 1 ? 0 : (sizeof(T) == 2 ? 1 : 2);

    /// 单寄存器存储类型（与 T 同宽，无符号，用于位域操作）
    using Type = std::tuple_element_t<kIndex, std::tuple<uint8_t, uint16_t, uint32_t>>;
  };

  /**
   * @brief 将 ValidBits 位有符号数符号扩展到 OutType 位宽
   * @tparam OutType 目标有符号整数类型
   * @tparam ValidBits 有效位数
   * @tparam RawType 原始无符号类型
   * @param raw 原始值
   * @return 符号扩展后的值
   */
  template <typename OutType, uint8_t ValidBits, typename RawType>
  static constexpr OutType SignExtend(const RawType raw) noexcept {
    static_assert(std::is_integral_v<OutType>, "SignExtend output type must be integral");
    static_assert(ValidBits > 0, "SignExtend valid bits must be > 0");

    constexpr uint8_t kTargetBits = sizeof(OutType) * 8U;
    static_assert(ValidBits <= kTargetBits, "SignExtend valid bits exceed output width");

    using unsigned_out_t     = std::make_unsigned_t<OutType>;
    constexpr uint8_t kShift = static_cast<uint8_t>(kTargetBits - ValidBits);

    const auto shifted   = static_cast<unsigned_out_t>(static_cast<unsigned_out_t>(raw) << kShift);
    const auto as_signed = static_cast<OutType>(shifted);
    return static_cast<OutType>(as_signed >> kShift);
  }

  template <typename T, typename = void>
  struct FieldOf {
    using Type = typename T::FieldBase;
  };

  template <typename T, typename = void>
  struct ConverterOf {
    using value_t = typename FieldOf<T>::Type::value_t;
    using Type    = ::zeta::regmap::Converter<value_t>;
  };

  template <typename T>
  struct ConverterOf<T, std::void_t<typename T::converter_t>> {
    using Type = typename T::converter_t;
  };

  template <typename T>
  struct ReadValueTypeOf {
    using Type = typename ConverterOf<T>::Type::value_t;
  };

  template <typename T>
  struct WriteValueTypeOf {
    using Type = typename ConverterOf<T>::Type::value_t;
  };

  template <typename T>
  struct EncodePipeline {
    using Field     = typename FieldOf<T>::Type;
    using Converter = typename ConverterOf<T>::Type;
    using in_t      = typename WriteValueTypeOf<T>::Type;
    using out_t     = typename Field::value_t;

    static constexpr out_t Run(const in_t value) noexcept {
      return Converter::template ToRaw<out_t>(value);
    }
  };

  template <typename T>
  struct DecodePipeline {
    using Field     = typename FieldOf<T>::Type;
    using Converter = typename ConverterOf<T>::Type;
    using in_t      = typename Field::value_t;
    using out_t     = typename ReadValueTypeOf<T>::Type;

    static constexpr out_t Run(const in_t raw) noexcept {
      return Converter::template FromRaw<in_t>(raw);
    }
  };
};

/**
 * @brief 寄存器位域描述结构
 *
 * 定义寄存器位域的地址、起始位置、位宽和掩码。
 * 用于描述寄存器中的特定位域。
 */
template <typename ValueType, uint8_t Address, uint8_t Shift, uint8_t Bits>
struct Field : public zeta::Noncopyable {
 public:
  using FieldBase = Field<ValueType, Address, Shift, Bits>;
  using value_t   = ValueType;
  using access_t  = typename Trait::AccessType<value_t>::Type;

  static constexpr uint8_t  kAddress  = Address;
  static constexpr uint8_t  kShift    = Shift;
  static constexpr uint8_t  kBits     = Bits;
  static constexpr size_t   kSize     = sizeof(access_t);
  static constexpr size_t   kSizeBits = kSize * 8U;
  static constexpr access_t kMask =
      static_cast<access_t>(zeta::utils::bit_utils::CreateMask(kShift, kBits));
  static constexpr access_t kClearMask = static_cast<access_t>(~kMask);

  static_assert(std::is_integral_v<value_t>, "Field value type must be integral");
  static_assert(sizeof(value_t) == 1 || sizeof(value_t) == 2 || sizeof(value_t) == 4,
                "Field value type must be 1/2/4 bytes");
  static_assert(Bits > 0, "Field Bits must be greater than 0");
  static_assert(Bits <= kSizeBits, "Field Bits exceeds underlying access width");
  static_assert(Shift < kSizeBits, "Field Shift exceeds underlying access width");
  static_assert(Shift + Bits <= kSizeBits, "Field range exceeds underlying access width");

  /**
   * @brief 从寄存器数据中提取位域值，有符号类型做符号扩展
   * @param data 寄存器原始值
   * @return 提取出的位域值
   */
  static constexpr value_t GetValue(const access_t data) noexcept {
    const access_t extracted = static_cast<access_t>((data & kMask) >> kShift);
    if constexpr (std::is_signed_v<value_t>) {
      return Trait::template SignExtend<value_t, kBits>(extracted);
    }
    return static_cast<value_t>(extracted);
  }

  /**
   * @brief 将 value 写入位域，其它位不变
   * @param value 要写入的值
   * @param data 寄存器值（原地修改）
   */
  static constexpr void SetValue(const value_t value, access_t& data) noexcept {
    const access_t shifted = static_cast<access_t>(static_cast<access_t>(value) << kShift);
    data                   = static_cast<access_t>((data & kClearMask) | (shifted & kMask));
  }

  static constexpr void Clear(access_t& data) noexcept { data &= kClearMask; }
};

/// @brief 跨两寄存器的合并字段（高/低字段组合为一个值）
template <typename ValueType, typename HighFieldType, typename LowFieldType>
struct Merged2 : public zeta::Noncopyable {
  using Merged2Base   = Merged2<ValueType, HighFieldType, LowFieldType>;
  using value_t       = ValueType;
  using HighField     = HighFieldType;
  using LowField      = LowFieldType;
  using access_t      = typename Trait::AccessType<value_t>::Type;
  using high_value_t  = typename HighField::value_t;
  using high_access_t = typename HighField::access_t;
  using low_value_t   = typename LowField::value_t;
  using low_access_t  = typename LowField::access_t;

  static constexpr size_t  kValueBits  = sizeof(value_t) * 8U;
  static constexpr uint8_t kMergedBits = HighField::kBits + LowField::kBits;

  static_assert(std::is_integral_v<value_t>, "Merged2 value type must be integral");
  static_assert(kMergedBits <= kValueBits, "Merged2 field width exceeds target value width");

  static constexpr void SetValue(const value_t  value,
                                 high_access_t& high_access,
                                 low_access_t&  low_access) noexcept {
    HighField::SetValue(static_cast<high_value_t>(value >> LowField::kBits), high_access);
    LowField::SetValue(static_cast<low_value_t>(value), low_access);
  }

  static constexpr value_t GetValue(const high_access_t high_access,
                                    const low_access_t  low_access) noexcept {
    constexpr uint8_t kHighBits = HighField::kBits;
    constexpr uint8_t kLowBits  = LowField::kBits;

    const access_t high_part = static_cast<access_t>(HighField::GetValue(high_access));
    const access_t low_part  = static_cast<access_t>(LowField::GetValue(low_access));

    constexpr access_t high_mask =
        static_cast<access_t>(zeta::utils::bit_utils::CreateMask(0, kHighBits));
    constexpr access_t low_mask =
        static_cast<access_t>(zeta::utils::bit_utils::CreateMask(0, kLowBits));

    const access_t extracted =
        static_cast<access_t>(((high_part & high_mask) << kLowBits) | (low_part & low_mask));

    if constexpr (std::is_signed_v<value_t>) {
      return Trait::template SignExtend<value_t, kMergedBits>(extracted);
    }
    return static_cast<value_t>(extracted);
  }
};

/// @brief 原始值与用户值的恒等转换（可特化或用 RatioConverter）
template <typename ValueType>
struct Converter : public zeta::Noncopyable {
  using value_t = ValueType;
  template <typename RawType>
  static constexpr value_t FromRaw(const RawType raw) noexcept {
    return static_cast<value_t>(raw);
  }

  template <typename RawType>
  static constexpr RawType ToRaw(const value_t value) noexcept {
    return static_cast<RawType>(value);
  }
};

/// @brief 原始值与用户值的比例转换（user = raw * Num / Den）
template <typename ValueType, int32_t Numerator, int32_t Denominator = 1>
struct RatioConverter : public Converter<ValueType> {
  using value_t = typename regmap::Converter<ValueType>::value_t;
  static_assert(Numerator != 0, "Numerator must not be zero");
  static_assert(Denominator != 0, "Denominator must not be zero");

  template <typename RawType>
  static constexpr value_t FromRaw(const RawType raw) noexcept {
    return static_cast<value_t>(raw) * static_cast<value_t>(Numerator) /
           static_cast<value_t>(Denominator);
  }

  template <typename RawType>
  static constexpr RawType ToRaw(const value_t value) noexcept {
    return static_cast<RawType>(value * static_cast<value_t>(Denominator) /
                                static_cast<value_t>(Numerator));
  }
};
}  // namespace zeta::regmap
