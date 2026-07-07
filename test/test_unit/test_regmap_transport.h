// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

#pragma once

/* regmap 传输层单元测试。当前覆盖 RegMmio 的初始化、空指针和范围边界。 */

#include <unity.h>

#include "regmap/reg_mmio.h"

namespace RegmapTransportTest {

using zeta::Error;
using zeta::regmap::RegMmio;

void assert_error_eq(const Error expected, const Error actual) {
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected), static_cast<uint8_t>(actual));
}

void test_mmio_rejects_invalid_init_and_uninitialized_access(void) {
  RegMmio mmio;
  uint8_t byte = 0;

  assert_error_eq(Error::kInvalidArg, mmio.Init(nullptr, sizeof(byte)));
  assert_error_eq(Error::kInvalidArg, mmio.Init(&byte, 0));
  assert_error_eq(Error::kInvalidArg, mmio.Read(0, sizeof(byte), &byte));
  assert_error_eq(Error::kInvalidArg, mmio.Write(0, &byte, sizeof(byte)));
}

void test_mmio_reads_writes_and_checks_bounds(void) {
  uint8_t regs[4]   = {};
  uint8_t input[2]  = {0xAA, 0xBB};
  uint8_t output[2] = {};
  RegMmio mmio;

  assert_error_eq(Error::kOk, mmio.Init(regs, sizeof(regs)));
  assert_error_eq(Error::kOk, mmio.Write(1, input, sizeof(input)));
  TEST_ASSERT_EQUAL_UINT8(0xAA, regs[1]);
  TEST_ASSERT_EQUAL_UINT8(0xBB, regs[2]);

  assert_error_eq(Error::kOk, mmio.Read(1, sizeof(output), output));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(input, output, sizeof(input));

  assert_error_eq(Error::kInvalidArg, mmio.Write(3, input, sizeof(input)));
  assert_error_eq(Error::kInvalidArg, mmio.Read(3, sizeof(output), output));
  assert_error_eq(Error::kInvalidArg, mmio.Write(0, nullptr, sizeof(input)));
  assert_error_eq(Error::kInvalidArg, mmio.Read(0, sizeof(output), nullptr));
}

void run_tests(void) {
  RUN_TEST(RegmapTransportTest::test_mmio_rejects_invalid_init_and_uninitialized_access);
  RUN_TEST(RegmapTransportTest::test_mmio_reads_writes_and_checks_bounds);
}

}  // namespace RegmapTransportTest
