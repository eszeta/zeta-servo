// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file slave_regmap_impl.h
 * @brief 伺服从机寄存器映射的 EEPROM 持久化实现
 */

#pragma once

#include <EEPROM.h>

#include "error.h"
#include "slave/control_table.h"

namespace zeta::slave::detail {

template <typename RegmapType>
class EepromStore {
 public:
  /**
   * @brief 恢复 EEPROM 为默认值
   * @return 错误码
   * @note 恢复 EEPROM 为默认值，但不恢复 ID
   */
  Error RecoveryEeprom() {
    RestoreEeprom<ControlTable::kFirmwareVersion>();
    RestoreEeprom<ControlTable::kBaudRate>();
    RestoreEeprom<ControlTable::kReturnDelayTime>();
    RestoreEeprom<ControlTable::kStatusReturnLevel>();
    RestoreEeprom<ControlTable::kDriveMode>();
    RestoreEeprom<ControlTable::kOperatingMode>();
    RestoreEeprom<ControlTable::kShutdown>();
    RestoreEeprom<ControlTable::kHomingOffset>();
    RestoreEeprom<ControlTable::kMovingThreshold>();
    RestoreEeprom<ControlTable::kTemperatureLimit>();
    RestoreEeprom<ControlTable::kMaxVoltageLimit>();
    RestoreEeprom<ControlTable::kMinVoltageLimit>();
    RestoreEeprom<ControlTable::kPwmLimit>();
    RestoreEeprom<ControlTable::kCurrentLimit>();
    RestoreEeprom<ControlTable::kVelocityLimit>();
    RestoreEeprom<ControlTable::kMaxPositionLimit>();
    RestoreEeprom<ControlTable::kMinPositionLimit>();
    RestoreEeprom<ControlTable::kProtectionTime>();
    RestoreEeprom<ControlTable::kVelocityIGain>();
    RestoreEeprom<ControlTable::kVelocityPGain>();
    RestoreEeprom<ControlTable::kPositionDGain>();
    RestoreEeprom<ControlTable::kPositionIGain>();
    RestoreEeprom<ControlTable::kPositionPGain>();
    RestoreEeprom<ControlTable::kFeedforward2ndGain>();
    RestoreEeprom<ControlTable::kFeedforward1stGain>();
    RestoreEeprom<ControlTable::kProfileAcceleration>();
    RestoreEeprom<ControlTable::kProfileVelocity>();
    StoreEeprom();
    return Error::kOk;
  }

  /**
   * @brief 加载 EEPROM
   * @return 错误码
   */
  Error LoadEeprom() {
    auto* self = static_cast<RegmapType*>(this);
    int   pos  = 0;
    for (uint8_t address = TableBlocks::kEeprom::kBegin; address < TableBlocks::kEeprom::kEnd;
         ++address) {
      self->table_[address] = EEPROM.read(pos++);
    }
    return Error::kOk;
  }

  /**
   * @brief 存储 EEPROM
   * @return 错误码
   */
  Error StoreEeprom() {
    auto* self = static_cast<RegmapType*>(this);
    int   pos  = 0;
    for (uint8_t address = TableBlocks::kEeprom::kBegin; address < TableBlocks::kEeprom::kEnd;
         ++address) {
      EEPROM.update(pos++, self->table_[address]);
    }
    return Error::kOk;
  }

  /**
   * @brief 将单个 EEPROM 寄存器恢复为默认值
   * @tparam T 寄存器类型
   * @return 错误码
   */
  template <typename T>
  constexpr Error RestoreEeprom() {
    auto* self = static_cast<RegmapType*>(this);
    self->template WriteField<T>(T::kDefault);
    return Error::kOk;
  }

  /**
   * @brief 检测 EEPROM 存档是否为空（未初始化）
   * @return true 若 EEPROM 区全为 0xFF 或 0x00
   */
  bool IsEepromEmpty() const {
    const auto* self  = static_cast<const RegmapType*>(this);
    const auto  first = self->table_[TableBlocks::kEeprom::kBegin];
    const auto  begin = TableBlocks::kEeprom::kBegin;
    const auto  end   = TableBlocks::kEeprom::kEnd;
    for (uint8_t address = begin; address < end; ++address) {
      if (self->table_[address] != first) {
        return false;
      }
    }
    return first == 0x00 || first == 0xFF;
  }
};

}  // namespace zeta::slave::detail
