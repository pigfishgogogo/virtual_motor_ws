// include/virtual_motor_device/canopen_definitions.hpp
#ifndef VIRTUAL_MOTOR_DEVICE__CANOPEN_DEFINITIONS_HPP_
#define VIRTUAL_MOTOR_DEVICE__CANOPEN_DEFINITIONS_HPP_

#include <cstdint>

namespace canopen {

// NMT 命令
constexpr uint8_t NMT_START = 0x01;
constexpr uint8_t NMT_STOP = 0x02;
constexpr uint8_t NMT_ENTER_PREOP = 0x80;
constexpr uint8_t NMT_RESET_NODE = 0x81;
constexpr uint8_t NMT_RESET_COMM = 0x82;

// NMT CAN-ID
constexpr uint32_t NMT_ID = 0x000;

// SDO 命令字
constexpr uint8_t SDO_UPLOAD_INITIATE = 0x40;  // 客户端 -> 服务器
constexpr uint8_t SDO_DOWNLOAD_INITIATE = 0x20; // 客户端 -> 服务器
constexpr uint8_t SDO_UPLOAD_SEGMENT = 0x60;    // 服务器 -> 客户端
constexpr uint8_t SDO_DOWNLOAD_SEGMENT = 0x00;  // 服务器 -> 客户端
constexpr uint8_t SDO_ABORT = 0x80;

// SDO CAN-ID 基址
constexpr uint32_t SDO_RX_BASE_ID = 0x600;  // 主站 -> 从站 (Client -> Server)
constexpr uint32_t SDO_TX_BASE_ID = 0x580;  // 从站 -> 主站 (Server -> Client)

// PDO CAN-ID 基址
constexpr uint32_t PDO1_RX_BASE_ID = 0x200;  // 接收PDO1 (主站 -> 从站)
constexpr uint32_t PDO1_TX_BASE_ID = 0x180;  // 发送PDO1 (从站 -> 主站)
constexpr uint32_t PDO2_RX_BASE_ID = 0x300;  // 接收PDO2
constexpr uint32_t PDO2_TX_BASE_ID = 0x280;  // 发送PDO2
constexpr uint32_t PDO3_RX_BASE_ID = 0x400;  // 接收PDO3
constexpr uint32_t PDO3_TX_BASE_ID = 0x380;  // 发送PDO3
constexpr uint32_t PDO4_RX_BASE_ID = 0x500;  // 接收PDO4
constexpr uint32_t PDO4_TX_BASE_ID = 0x480;  // 发送PDO4

// 紧急报文
constexpr uint32_t EMCY_BASE_ID = 0x080;

// 状态机状态
constexpr uint8_t STATE_INITIAL = 0x00;
constexpr uint8_t STATE_PRE_OP = 0x7F;
constexpr uint8_t STATE_OPERATIONAL = 0x05;
constexpr uint8_t STATE_STOPPED = 0x04;
constexpr uint8_t STATE_SWITCHED_ON = 0x06;
constexpr uint8_t STATE_READY_SWITCH_ON = 0x07;
constexpr uint8_t STATE_FAULT = 0x0F;

// 心跳报文
constexpr uint32_t HEARTBEAT_BASE_ID = 0x700;

// DS402 控制字命令
constexpr uint16_t CONTROL_WORD_SHUTDOWN = 0x0006;
constexpr uint16_t CONTROL_WORD_SWITCH_ON = 0x0007;
constexpr uint16_t CONTROL_WORD_ENABLE_OP = 0x000F;
constexpr uint16_t CONTROL_WORD_DISABLE_VOLTAGE = 0x0000;
constexpr uint16_t CONTROL_WORD_QUICK_STOP = 0x0002;
constexpr uint16_t CONTROL_WORD_FAULT_RESET = 0x0080;

// DS402 状态字状态
constexpr uint16_t STATUS_WORD_READY_SWITCH_ON = 0x0021;
constexpr uint16_t STATUS_WORD_SWITCHED_ON = 0x0023;
constexpr uint16_t STATUS_WORD_OPERATION_ENABLED = 0x0027;
constexpr uint16_t STATUS_WORD_FAULT = 0x0008;
constexpr uint16_t STATUS_WORD_VOLTAGE_ENABLED = 0x0010;
constexpr uint16_t STATUS_WORD_QUICK_STOP = 0x0002;
constexpr uint16_t STATUS_WORD_SWITCH_ON_DISABLED = 0x0040;
constexpr uint16_t STATUS_WORD_WARNING = 0x0080;
constexpr uint16_t STATUS_WORD_REMOTE = 0x0200;
constexpr uint16_t STATUS_WORD_TARGET_REACHED = 0x0400;
constexpr uint16_t STATUS_WORD_INTERNAL_LIMIT_ACTIVE = 0x0800;

// DS402 操作模式
constexpr int8_t MODE_POSITION = 0x01;
constexpr int8_t MODE_VELOCITY = 0x03;
constexpr int8_t MODE_TORQUE = 0x04;
constexpr int8_t MODE_HOMING = 0x06;
constexpr int8_t MODE_CSP = 0x08;  // 循环同步位置模式
constexpr int8_t MODE_CSV = 0x09;  // 循环同步速度模式
constexpr int8_t MODE_CST = 0x0A;  // 循环同步力矩模式

}  // namespace canopen

#endif  // VIRTUAL_MOTOR_DEVICE__CANOPEN_DEFINITIONS_HPP_