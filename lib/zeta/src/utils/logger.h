// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file logger.h
 * @brief 超轻量嵌入式日志系统接口
 */

#pragma once

#include <Arduino.h>

#include <cstdarg>
#include <cstddef>
#include <cstdio>

namespace zeta::utils {

/// @brief 日志级别，枚举值越大表示优先级越高
enum class LogLevel {
  kDebug,  ///< 调试信息，默认会被过滤
  kInfo,   ///< 普通运行信息
  kWarn,   ///< 警告信息
  kError,  ///< 错误信息
  kNone,   ///< 关闭所有日志输出
};

/// @brief 基于 Arduino Print 的静态日志输出器
class Logger {
 public:
  /// @brief 格式化日志使用的固定栈缓冲区大小
  static constexpr size_t kBufferSize = 128;

  /**
   * @brief 初始化日志输出目标和最低输出级别
   * @param output 输出目标；传入 nullptr 时禁用日志输出
   * @param level 最低输出级别，低于该级别的日志会被过滤
   */
  static void Init(::Print* output, LogLevel level = LogLevel::kInfo) {
    output_ = output;
    level_  = level;
  }

  /**
   * @brief 设置最低输出级别
   * @param level 新的最低输出级别；设置为 LogLevel::kNone 时关闭所有日志
   */
  static void SetLevel(LogLevel level) { level_ = level; }

  /**
   * @brief 获取当前最低输出级别
   * @return 当前最低输出级别
   */
  static LogLevel GetLevel() { return level_; }

  /**
   * @brief 判断指定级别的日志当前是否应该输出
   * @param level 待判断的日志级别
   * @return 输出目标有效且日志级别不低于当前阈值时返回 true
   */
  static bool ShouldOutput(LogLevel level) {
    return output_ != nullptr && level != LogLevel::kNone && level_ != LogLevel::kNone &&
           level >= level_;
  }

  /**
   * @brief 按 printf 风格格式化并输出一行日志
   * @param level 日志级别
   * @param format printf 风格格式字符串；为空指针时安全忽略
   * @param ... 格式化参数
   * @note 格式化结果超过 kBufferSize-1 字节时会被截断
   */
  static void Printf(LogLevel level, const char* format, ...) {
    if (!ShouldOutput(level) || format == nullptr) {
      return;
    }

    char    buffer[kBufferSize];
    va_list args;
    va_start(args, format);
    const int written = std::vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (written < 0) {
      return;
    }

    buffer[kBufferSize - 1] = '\0';
    PrintPrefix(level);
    output_->println(buffer);
  }

  /**
   * @brief 输出一行纯文本日志
   * @param level 日志级别
   * @param message 日志文本；为空指针时安全忽略
   */
  static void Print(LogLevel level, const char* message) {
    if (!ShouldOutput(level) || message == nullptr) {
      return;
    }

    PrintPrefix(level);
    output_->println(message);
  }

 private:
  /// @brief 获取日志级别对应的固定文本前缀
  static constexpr const char* Prefix(LogLevel level) {
    switch (level) {
      case LogLevel::kDebug:
        return "[D] ";
      case LogLevel::kInfo:
        return "[I] ";
      case LogLevel::kWarn:
        return "[W] ";
      case LogLevel::kError:
        return "[E] ";
      case LogLevel::kNone:
        return "";
    }
    return "";
  }

  /// @brief 输出日志级别前缀，调用前需确保输出目标有效
  static void PrintPrefix(LogLevel level) { output_->print(Prefix(level)); }

  inline static ::Print* output_ = nullptr;
  inline static LogLevel level_  = LogLevel::kInfo;
};

}  // namespace zeta::utils

/// @brief 输出 Debug 级别格式化日志；低于当前阈值时不求值格式化参数
#define LOG_DEBUG(format, ...)                                                                   \
  do {                                                                                           \
    if (::zeta::utils::Logger::ShouldOutput(::zeta::utils::LogLevel::kDebug)) {              \
      ::zeta::utils::Logger::Printf(::zeta::utils::LogLevel::kDebug, format, ##__VA_ARGS__); \
    }                                                                                            \
  } while (0)

/// @brief 输出 Info 级别格式化日志；低于当前阈值时不求值格式化参数
#define LOG_INFO(format, ...)                                                                   \
  do {                                                                                          \
    if (::zeta::utils::Logger::ShouldOutput(::zeta::utils::LogLevel::kInfo)) {              \
      ::zeta::utils::Logger::Printf(::zeta::utils::LogLevel::kInfo, format, ##__VA_ARGS__); \
    }                                                                                           \
  } while (0)

/// @brief 输出 Warn 级别格式化日志；低于当前阈值时不求值格式化参数
#define LOG_WARN(format, ...)                                                                   \
  do {                                                                                          \
    if (::zeta::utils::Logger::ShouldOutput(::zeta::utils::LogLevel::kWarn)) {              \
      ::zeta::utils::Logger::Printf(::zeta::utils::LogLevel::kWarn, format, ##__VA_ARGS__); \
    }                                                                                           \
  } while (0)

/// @brief 输出 Error 级别格式化日志；低于当前阈值时不求值格式化参数
#define LOG_ERROR(format, ...)                                                                   \
  do {                                                                                           \
    if (::zeta::utils::Logger::ShouldOutput(::zeta::utils::LogLevel::kError)) {              \
      ::zeta::utils::Logger::Printf(::zeta::utils::LogLevel::kError, format, ##__VA_ARGS__); \
    }                                                                                            \
  } while (0)

/// @brief 条件成立时输出 Debug 级别格式化日志
#define LOG_DEBUG_IF(condition, format, ...)                                                       \
  do {                                                                                             \
    if ((condition) && ::zeta::utils::Logger::ShouldOutput(::zeta::utils::LogLevel::kDebug)) { \
      ::zeta::utils::Logger::Printf(::zeta::utils::LogLevel::kDebug, format, ##__VA_ARGS__);   \
    }                                                                                              \
  } while (0)

/// @brief 条件成立时输出 Info 级别格式化日志
#define LOG_INFO_IF(condition, format, ...)                                                       \
  do {                                                                                            \
    if ((condition) && ::zeta::utils::Logger::ShouldOutput(::zeta::utils::LogLevel::kInfo)) { \
      ::zeta::utils::Logger::Printf(::zeta::utils::LogLevel::kInfo, format, ##__VA_ARGS__);   \
    }                                                                                             \
  } while (0)

/// @brief 条件成立时输出 Warn 级别格式化日志
#define LOG_WARN_IF(condition, format, ...)                                                       \
  do {                                                                                            \
    if ((condition) && ::zeta::utils::Logger::ShouldOutput(::zeta::utils::LogLevel::kWarn)) { \
      ::zeta::utils::Logger::Printf(::zeta::utils::LogLevel::kWarn, format, ##__VA_ARGS__);   \
    }                                                                                             \
  } while (0)

/// @brief 条件成立时输出 Error 级别格式化日志
#define LOG_ERROR_IF(condition, format, ...)                                                       \
  do {                                                                                             \
    if ((condition) && ::zeta::utils::Logger::ShouldOutput(::zeta::utils::LogLevel::kError)) { \
      ::zeta::utils::Logger::Printf(::zeta::utils::LogLevel::kError, format, ##__VA_ARGS__);   \
    }                                                                                              \
  } while (0)
