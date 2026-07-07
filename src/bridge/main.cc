// Copyright 2025 ES_ZETA
// SPDX-License-Identifier: Apache-2.0

/**
 * @brief 串口 Commander 指令速查表
 *
 * 主指令：
 *
 * +------+----------------+-----------+-------------------+-------------------------------+
 * | 指令 | 参数           | 单位/取值 | 功能              | 写入 / 影响                   |
 * +------+----------------+-----------+-------------------+-------------------------------+
 * | p    | <angle>        | deg       | 设置目标位置      | angle * kAngleToRaw -> 0x98   |
 * | t    | <0/1>          | bool      | 力矩使能          | TorqueEnable(0x80)            |
 * | i    | <id>           | uint8_t   | 切换目标 Slave ID | 更新 target_id                |
 * | g    | <sub> <value>  | float     | 设置控制增益      | ControlTable 对应寄存器       |
 * +------+----------------+-----------+-------------------+-------------------------------+
 *
 * `g` 指令子标识：
 *
 * +------+-----------------------+----------------+
 * | sub  | ControlTable 寄存器   | 增益类型       |
 * +------+-----------------------+----------------+
 * | pp   | PositionPGain         | 位置环 P 增益  |
 * | pi   | PositionIGain         | 位置环 I 增益  |
 * | pd   | PositionDGain         | 位置环 D 增益  |
 * | vp   | VelocityPGain         | 速度环 P 增益  |
 * | vi   | VelocityIGain         | 速度环 I 增益  |
 * | f1   | Feedforward1stGain    | 一阶前馈增益   |
 * | f2   | Feedforward2ndGain    | 二阶前馈增益   |
 * +------+-----------------------+----------------+
 */

#include <Arduino.h>
#include <Wire.h>
#include <info_led/info_led.h>
#include <math/resolution.h>
#include <protocol/protocol.h>
#include <slave/control_table.h>
#include <utils/commander.h>
#include <utils/task_scheduler.h>

using InfoLED       = zeta::info_led::InfoLED<zeta::info_led::LedMode::kOpenDrain>;
using InfoLEDInfo   = InfoLED::InfoType;
using Commander     = zeta::utils::Commander;
using TaskScheduler = zeta::utils::TaskScheduler<>;
using Error         = zeta::Error;
using zeta::IsFail;

// Slave 编码器分辨率（与 STM32/main.cc 保持一致）
constexpr uint8_t kResolutionBits = 12;
using Resolution                  = zeta::math::Resolution<kResolutionBits>;

// ── 引脚常量 ──────────────────────────────────────────────────────────────────
constexpr auto kInfoLedPin     = PB1;
constexpr auto kMainLoopRateHz = 500u;
constexpr auto kLedOnlyRateHz  = 50u;

// ── 全局对象 ──────────────────────────────────────────────────────────────────
HardwareSerial serial(PIN_SERIAL_RX, PIN_SERIAL_TX);
TwoWire        wire(PIN_WIRE_SDA, PIN_WIRE_SCL);

InfoLED       led{};
Commander     commander{};
TaskScheduler scheduler{};

uint8_t target_id = 1;  // 当前控制的 Slave ID（`i` 命令可修改）

// ── 函数声明 ──────────────────────────────────────────────────────────────────
Error SystemSetup();
Error MainLoopCallback(float dt);
Error LedCallback(float dt);
void  EnterErrorMode(Error error);

/**
 * @brief 构建 DYNAMIXEL WRITE 包并经 I2C 发送给指定 Slave
 *
 * @tparam T    寄存器数据类型（uint8_t / uint16_t / int32_t …）
 * @param id       目标 Slave I2C 地址 / ID
 * @param reg_addr 寄存器起始地址
 * @param value    写入值（按类型大小字节序原样写入）
 */
template <typename T>
void WriteRegI2C(uint8_t id, uint8_t reg_addr, T value) {
  using namespace zeta::protocol;
  InstPacket pkt{};
  pkt.header1            = kHeaderByte;
  pkt.header2            = kHeaderByte;
  pkt.id                 = id;
  pkt.instructionOrError = Instruction::kWriteData;
  pkt.parameter[0]       = reg_addr;
  memcpy(&pkt.parameter[1], &value, sizeof(T));
  pkt.SetParameterSize(1u + static_cast<uint8_t>(sizeof(T)));
  pkt.SetChecksum(pkt.CalculateChecksum());
  wire.beginTransmission(id);
  wire.write(pkt.buffer, pkt.GetBufferSize());
  wire.endTransmission();
}

/**
 * @brief 将物理增益值转换为 raw uint16 并写入对应寄存器
 *
 * 地址和量纲转换均取自 slave::ControlTable 的寄存器定义。
 *
 * @tparam RegType  slave::ControlTable 中的寄存器结构（需有 kAddress 和 converter_t）
 */
template <typename RegType>
void WriteGain(uint8_t id, float val) {
  const uint16_t raw = RegType::converter_t::template ToRaw<uint16_t>(val);
  WriteRegI2C(id, RegType::kAddress, raw);
}

// cppcheck-suppress unusedFunction
void setup() {
  const auto error = SystemSetup();
  if (IsFail(error)) {
    EnterErrorMode(error);
  }
}

// cppcheck-suppress unusedFunction
void loop() {
  const auto error = scheduler.Tick();
  if (IsFail(error)) {
    EnterErrorMode(error);
  }
}

void EnterErrorMode(Error error) {
  led.ShowErrorCode(static_cast<uint8_t>(error));
  scheduler.RemoveTask(MainLoopCallback);
}

Error SystemSetup() {
  serial.begin(115200);
  wire.begin();
  led.Init(kInfoLedPin);
  led.SetInfo(InfoLEDInfo::kOk);

  // p <angle>  目标位置（单位：度，经 Resolution::kAngleToRaw 转换为 pulse）
  commander.add('p', [](int argc, char** argv) {
    if (argc < 1)
      return;
    const float   angle = atof(argv[0]);
    const int32_t pos   = static_cast<int32_t>(angle * Resolution::kAngleToRaw);
    WriteRegI2C(target_id, 0x98, pos);
  });

  // t <0|1>  力矩使能（TorqueEnable 0x80）
  commander.add('t', [](int argc, char** argv) {
    if (argc < 1)
      return;
    const uint8_t en = static_cast<uint8_t>(atoi(argv[0]));
    WriteRegI2C(target_id, 0x80, en);
  });

  // i <id>  切换目标 Slave ID
  commander.add('i', [](int argc, char** argv) {
    if (argc < 1)
      return;
    target_id = static_cast<uint8_t>(atoi(argv[0]));
  });

  // g <sub> <value>  PID / 前馈增益设置
  // 子标识: pp pi pd vp vi f1 f2
  // 地址与量纲转换来自 slave::ControlTable 寄存器定义
  commander.add('g', [](int argc, char** argv) {
    namespace CT = zeta::slave::ControlTable;
    if (argc < 2)
      return;
    const float val = atof(argv[1]);
    const char* sub = argv[0];
    if (strcmp(sub, "pp") == 0)
      WriteGain<CT::kPositionPGain>(target_id, val);
    else if (strcmp(sub, "pi") == 0)
      WriteGain<CT::kPositionIGain>(target_id, val);
    else if (strcmp(sub, "pd") == 0)
      WriteGain<CT::kPositionDGain>(target_id, val);
    else if (strcmp(sub, "vp") == 0)
      WriteGain<CT::kVelocityPGain>(target_id, val);
    else if (strcmp(sub, "vi") == 0)
      WriteGain<CT::kVelocityIGain>(target_id, val);
    else if (strcmp(sub, "f1") == 0)
      WriteGain<CT::kFeedforward1stGain>(target_id, val);
    else if (strcmp(sub, "f2") == 0)
      WriteGain<CT::kFeedforward2ndGain>(target_id, val);
  });

  CHECK(scheduler.AddTask(MainLoopCallback, kMainLoopRateHz));
  CHECK(scheduler.AddTask(LedCallback, kLedOnlyRateHz));
  return Error::kOk;
}

/**
 * @brief 主循环（500Hz）：串口 Commander 解析 + 编码器本地位置控制
 */
Error MainLoopCallback(float dt) {
  // --- 串口：逐字符输入 Commander，按行触发命令 ---
  while (serial.available() > 0) {
    commander.readByte(static_cast<char>(serial.read()));
  }
  return Error::kOk;
}

Error LedCallback(float dt) {
  led.Process(dt);
  return Error::kOk;
}
