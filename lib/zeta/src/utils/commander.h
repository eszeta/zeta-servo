// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file commander.h
 * @brief 简单串口命令解析（单字符 ID + 空格分隔参数）
 */

#pragma once

#include <Arduino.h>

#include <cstddef>
#include <cstring>

#include "noncopyable.h"

namespace zeta::utils {

using CommandCallback = void (*)(int argc, char** argv);

/// @brief 串口命令解析器，首字符为命令 ID，后接空格分隔参数
class Commander : public zeta::Noncopyable {
 public:
  static constexpr size_t kLineBufferSize = 64;
  static constexpr size_t kMaxCommands    = 20;
  static constexpr size_t kMaxArgs        = 10;

  /**
   * @brief 注册命令：id 为首字符，on_command 为回调
   * @param id 命令首字符
   * @param on_command 回调 (argc, argv)
   * @return 注册成功（含覆盖原回调）返回 true；命令表已满返回 false
   */
  bool add(char id, CommandCallback on_command) {
    // 已存在则覆盖
    for (size_t i = 0; i < call_count_; i++) {
      if (entries_[i].id == id) {
        entries_[i].cb = on_command;
        return true;
      }
    }
    if (call_count_ >= kMaxCommands) {
      return false;
    }
    entries_[call_count_++] = {id, on_command};
    return true;
  }

  /**
   * @brief 解析并执行一行输入（首字符匹配则调用对应回调）
   * @param user_input 可写字符串（解析过程中会原地修改）；
   *                   首字符为命令 ID，后接空格分隔参数
   * @note 允许结尾带 \r/\n；空指针 / 空串安全忽略
   */
  void run(char* user_input) {
    if (user_input == nullptr || user_input[0] == '\0') {
      return;
    }

    const char id = user_input[0];

    // 容忍尾部 CR/LF
    size_t len = std::strlen(user_input);
    while (len > 0 && (user_input[len - 1] == '\n' || user_input[len - 1] == '\r')) {
      user_input[--len] = '\0';
    }

    for (size_t i = 0; i < call_count_; i++) {
      if (entries_[i].id != id) {
        continue;
      }

      char* argv[kMaxArgs];
      int   argc  = 0;
      char* save  = nullptr;
      char* token = (len > 1) ? ::strtok_r(&user_input[1], " ", &save) : nullptr;
      while (token != nullptr && argc < static_cast<int>(kMaxArgs)) {
        argv[argc++] = token;
        token        = ::strtok_r(nullptr, " ", &save);
      }
      entries_[i].cb(argc, argv);
      return;
    }
  }

  /**
   * @brief 输入一个串口字节，遇到换行符时执行当前行命令
   * @param byte 输入字节
   * @note 单行长度超过 kLineBufferSize-1 时整行丢弃（不会触发回调）
   */
  void readByte(char byte) {
    if (byte == '\n' || byte == '\r') {
      if (!overflow_ && line_len_ > 0) {
        line_buffer_[line_len_] = '\0';
        run(line_buffer_);
      }
      line_len_ = 0;
      overflow_ = false;
      return;
    }

    if (overflow_) {
      return;
    }
    if (line_len_ < kLineBufferSize - 1) {
      line_buffer_[line_len_++] = byte;
    } else {
      overflow_ = true;  // 超长，等待换行后整行丢弃
    }
  }

 private:
  struct Entry {
    char            id = '\0';
    CommandCallback cb = nullptr;
  };

  char   line_buffer_[kLineBufferSize]{};
  size_t line_len_ = 0;
  bool   overflow_ = false;
  Entry  entries_[kMaxCommands]{};
  size_t call_count_ = 0;
};

}  // namespace zeta::utils
