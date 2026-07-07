// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_logger.h
 * @brief 超轻量嵌入式日志系统单元测试
 */

#pragma once

#include <Arduino.h>
#include <PrintFake.h>
#include <unity.h>

#include <cstring>
#include <string>

#include "utils/logger.h"

// 解决与 ArduinoFake 中宏的冲突
#undef round
#undef abs

using zeta::utils::Logger;
using zeta::utils::LogLevel;

namespace LoggerTest {
namespace {

class RecordingPrint final : public PrintFake, public PrintFakeProxy {
 public:
  RecordingPrint() : PrintFakeProxy(this) {}

  size_t write(uint8_t ch) override {
    output_.push_back(static_cast<char>(ch));
    return 1;
  }

  size_t write(const uint8_t* buffer, size_t size) override {
    if (buffer == nullptr) {
      return 0;
    }

    output_.append(reinterpret_cast<const char*>(buffer), size);
    return size;
  }

  size_t print(const __FlashStringHelper*) override { return 0; }

  size_t print(const String& text) override { return print(text.c_str()); }

  size_t print(const char str[]) override {
    if (str == nullptr) {
      return 0;
    }
    return write(reinterpret_cast<const uint8_t*>(str), std::strlen(str));
  }

  size_t print(char ch) override { return write(static_cast<uint8_t>(ch)); }

  size_t print(unsigned char value, int /*base*/ = DEC) override {
    return print(static_cast<unsigned long>(value));
  }

  size_t print(int value, int /*base*/ = DEC) override { return print(static_cast<long>(value)); }

  size_t print(unsigned int value, int /*base*/ = DEC) override {
    return print(static_cast<unsigned long>(value));
  }

  size_t print(long value, int /*base*/ = DEC) override { return AppendNumber(value); }

  size_t print(unsigned long value, int /*base*/ = DEC) override { return AppendNumber(value); }

  size_t print(double value, int digits = 2) override { return AppendFloat(value, digits); }

  size_t print(const Printable& printable) override { return printable.printTo(*this); }

  size_t println(const __FlashStringHelper* text) override { return print(text) + Newline(); }

  size_t println(const String& text) override { return print(text) + Newline(); }

  size_t println(const char str[]) override { return print(str) + Newline(); }

  size_t println(char ch) override { return print(ch) + Newline(); }

  size_t println(unsigned char value, int base = DEC) override {
    return print(value, base) + Newline();
  }

  size_t println(int value, int base = DEC) override { return print(value, base) + Newline(); }

  size_t println(unsigned int value, int base = DEC) override {
    return print(value, base) + Newline();
  }

  size_t println(long value, int base = DEC) override { return print(value, base) + Newline(); }

  size_t println(unsigned long value, int base = DEC) override {
    return print(value, base) + Newline();
  }

  size_t println(double value, int digits = 2) override { return print(value, digits) + Newline(); }

  size_t println(const Printable& printable) override { return print(printable) + Newline(); }

  size_t println(void) override { return Newline(); }

  const char* c_str() const { return output_.c_str(); }

  size_t size() const { return output_.size(); }

  bool Contains(const char* text) const { return std::strstr(c_str(), text) != nullptr; }

 private:
  template <typename T>
  size_t AppendNumber(T value) {
    const std::string text = std::to_string(value);
    return write(reinterpret_cast<const uint8_t*>(text.c_str()), text.size());
  }

  size_t AppendFloat(double value, int digits) {
    char      buffer[32];
    const int written = std::snprintf(buffer, sizeof(buffer), "%.*f", digits, value);
    if (written <= 0) {
      return 0;
    }
    const size_t size = static_cast<size_t>(written);
    return write(reinterpret_cast<const uint8_t*>(buffer),
                 size < sizeof(buffer) ? size : sizeof(buffer) - 1);
  }

  size_t Newline() { return write(static_cast<uint8_t>('\n')); }

  std::string output_;
};

void InitLogger(RecordingPrint& output, LogLevel level = LogLevel::kInfo) {
  Logger::Init(static_cast<PrintFakeProxy*>(&output), level);
}

}  // namespace

void test_logger_initialization(void) {
  RecordingPrint output;
  InitLogger(output, LogLevel::kWarn);

  TEST_ASSERT_EQUAL(static_cast<int>(LogLevel::kWarn), static_cast<int>(Logger::GetLevel()));
  TEST_ASSERT_TRUE(Logger::ShouldOutput(LogLevel::kError));
  TEST_ASSERT_FALSE(Logger::ShouldOutput(LogLevel::kInfo));
}

void test_log_info_level_prints_by_default(void) {
  RecordingPrint output;
  InitLogger(output);

  LOG_INFO("ready");

  TEST_ASSERT_EQUAL_STRING("[I] ready\n", output.c_str());
}

void test_log_debug_level_is_suppressed_by_default(void) {
  RecordingPrint output;
  InitLogger(output);

  LOG_DEBUG("hidden");

  TEST_ASSERT_EQUAL_UINT32(0u, output.size());
}

void test_log_level_threshold_changes_output(void) {
  RecordingPrint output;
  InitLogger(output, LogLevel::kDebug);

  LOG_DEBUG("debug message");
  LOG_INFO("info message");
  LOG_ERROR("error message");

  TEST_ASSERT_TRUE(output.Contains("[D] debug message"));
  TEST_ASSERT_TRUE(output.Contains("[I] info message"));
  TEST_ASSERT_TRUE(output.Contains("[E] error message"));
}

void test_log_formatting(void) {
  RecordingPrint output;
  InitLogger(output);

  LOG_INFO("value: %d, str: %s", 42, "test");

  TEST_ASSERT_TRUE(output.Contains("value: 42"));
  TEST_ASSERT_TRUE(output.Contains("str: test"));
}

void test_log_conditional_print(void) {
  RecordingPrint output;
  InitLogger(output, LogLevel::kDebug);

  const int value = 0;
  LOG_DEBUG_IF(value == 0, "value is zero");
  LOG_DEBUG_IF(value != 0, "value is not zero");

  TEST_ASSERT_TRUE(output.Contains("value is zero"));
  TEST_ASSERT_FALSE(output.Contains("value is not zero"));
}

void run_tests(void) {
  RUN_TEST(LoggerTest::test_logger_initialization);
  RUN_TEST(LoggerTest::test_log_info_level_prints_by_default);
  RUN_TEST(LoggerTest::test_log_debug_level_is_suppressed_by_default);
  RUN_TEST(LoggerTest::test_log_level_threshold_changes_output);
  RUN_TEST(LoggerTest::test_log_formatting);
  RUN_TEST(LoggerTest::test_log_conditional_print);
}

}  // namespace LoggerTest