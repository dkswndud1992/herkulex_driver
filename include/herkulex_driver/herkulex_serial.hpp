// Copyright (c) 2026 HerkuleX ROS2 Driver
// Based on Herkulex Arduino Library by Alessandro Giacomel
// and HerkuleX C Library
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef HERKULEX_DRIVER__HERKULEX_SERIAL_HPP_
#define HERKULEX_DRIVER__HERKULEX_SERIAL_HPP_

#include <string>
#include <vector>
#include <cstdint>
#include <mutex>
#include <map>

namespace herkulex_driver
{

// ─── HerkuleX Model Definitions ────────────────────────────────

/// Supported HerkuleX servo models
enum class HerkulexModel : uint8_t
{
  DRS_0101 = 0,    ///< DRS-0101 (pot encoder, basic)
  DRS_0201 = 1,    ///< DRS-0201 (pot encoder, high-torque)
  DRS_0102 = 2,    ///< DRS-0102 (magnetic encoder, improved 01xx)
  DRS_0401 = 3,    ///< DRS-0401 (magnetic encoder)
  DRS_0402 = 4,    ///< DRS-0402 (magnetic absolute encoder)
  DRS_0601 = 5,    ///< DRS-0601 (magnetic encoder, high-torque)
  DRS_0602 = 6,    ///< DRS-0602 (magnetic absolute encoder, high-torque)
};

/// Model specification with physical characteristics and feature flags
struct ModelSpec
{
  std::string name;              ///< Model name string (e.g. "DRS-0101")
  bool has_velocity_gain;        ///< Whether velocity Kp/Ki registers exist
  int default_kp;                ///< Default position Kp
  int default_kd;                ///< Default position Kd
  int default_ki;                ///< Default position Ki
  int default_velocity_kp;       ///< Default velocity Kp (0x02 only)
  int default_velocity_ki;       ///< Default velocity Ki (0x02 only)
  float max_torque_kgcm;         ///< Max torque in kgf·cm
  float no_load_speed_rpm;       ///< No-load speed in RPM
  int min_position;              ///< Minimum valid position
  int max_position;              ///< Maximum valid position
  int center_position;           ///< Center (0 deg) position
  float deg_per_count;           ///< Degrees per encoder count
  uint8_t position_mask_msb;     ///< Mask for position MSB byte
};

/// Get the model specification for a given model
inline const ModelSpec & getModelSpec(HerkulexModel model)
{
  // Model specifications table
  //   DRS-0101/0201: Basic series (potentiometer encoder, 10-bit, center 512, 0.325 deg/count)
  //   DRS-0401/0601: Gen1 magnetic encoder series (11-bit, center 1024, 0.163 deg/count)
  //   DRS-0102/0402/0602: Gen2 magnetic absolute encoder series (15-bit, center 16384, 0.02778 deg/count, + velocity gains)
  static const std::map<HerkulexModel, ModelSpec> specs = {
    { HerkulexModel::DRS_0101, { "DRS-0101", false, 440, 8000, 0,   0,     0, 1.60f,  0.65f, 0,  1023,   512, 0.325f,   0x03 } },
    { HerkulexModel::DRS_0201, { "DRS-0201", false, 440, 8000, 0,   0,     0, 3.17f,  0.65f, 0,  1023,   512, 0.325f,   0x03 } },
    { HerkulexModel::DRS_0102, { "DRS-0102", true,  440, 8000, 0, 100, 12000, 1.60f,  0.65f, 0, 32767, 16384, 0.02778f, 0x7F } },
    { HerkulexModel::DRS_0401, { "DRS-0401", false, 440, 8000, 0,   0,     0, 3.96f, 66.00f, 0,  2047,  1024, 0.163f,   0x7F } },
    { HerkulexModel::DRS_0402, { "DRS-0402", true,  440, 8000, 0, 100, 12000, 3.96f, 66.00f, 0, 32767, 16384, 0.02778f, 0x7F } },
    { HerkulexModel::DRS_0601, { "DRS-0601", false, 440, 8000, 0,   0,     0, 7.40f, 61.80f, 0,  2047,  1024, 0.163f,   0x7F } },
    { HerkulexModel::DRS_0602, { "DRS-0602", true,  440, 8000, 0, 100, 12000, 7.40f, 61.80f, 0, 32767, 16384, 0.02778f, 0x7F } },
  };

  auto it = specs.find(model);
  if (it != specs.end()) {
    return it->second;
  }
  // Fallback to DRS-0101 if unknown
  return specs.at(HerkulexModel::DRS_0101);
}

/// Parse a model string (e.g. "0101", "DRS-0402", "0602", "602") to HerkulexModel enum
inline HerkulexModel parseModelString(const std::string & model_str)
{
  static const std::map<std::string, HerkulexModel> model_map = {
    { "0101", HerkulexModel::DRS_0101 }, { "DRS-0101", HerkulexModel::DRS_0101 }, { "101", HerkulexModel::DRS_0101 },
    { "0201", HerkulexModel::DRS_0201 }, { "DRS-0201", HerkulexModel::DRS_0201 }, { "201", HerkulexModel::DRS_0201 },
    { "0102", HerkulexModel::DRS_0102 }, { "DRS-0102", HerkulexModel::DRS_0102 }, { "102", HerkulexModel::DRS_0102 },
    { "0401", HerkulexModel::DRS_0401 }, { "DRS-0401", HerkulexModel::DRS_0401 }, { "401", HerkulexModel::DRS_0401 },
    { "0402", HerkulexModel::DRS_0402 }, { "DRS-0402", HerkulexModel::DRS_0402 }, { "402", HerkulexModel::DRS_0402 },
    { "0601", HerkulexModel::DRS_0601 }, { "DRS-0601", HerkulexModel::DRS_0601 }, { "601", HerkulexModel::DRS_0601 },
    { "0602", HerkulexModel::DRS_0602 }, { "DRS-0602", HerkulexModel::DRS_0602 }, { "602", HerkulexModel::DRS_0602 },
  };

  auto it = model_map.find(model_str);
  if (it != model_map.end()) {
    return it->second;
  }
  return HerkulexModel::DRS_0101;  // Default fallback
}

// ─── HerkuleX Protocol Constants ───────────────────────────────
// Packet structure
constexpr uint8_t HEADER_BYTE          = 0xFF;
constexpr uint8_t MIN_PACKET_SIZE      = 7;
constexpr uint8_t MAX_PACKET_SIZE      = 223;
constexpr uint8_t MAX_DATA_SIZE        = MAX_PACKET_SIZE - MIN_PACKET_SIZE;

// Servo ID
constexpr uint8_t MAX_SERVO_ID         = 0xFD;
constexpr uint8_t BROADCAST_ID         = 0xFE;

// Commands (Request)
constexpr uint8_t CMD_EEP_WRITE        = 0x01;
constexpr uint8_t CMD_EEP_READ         = 0x02;
constexpr uint8_t CMD_RAM_WRITE        = 0x03;
constexpr uint8_t CMD_RAM_READ         = 0x04;
constexpr uint8_t CMD_I_JOG            = 0x05;
constexpr uint8_t CMD_S_JOG            = 0x06;
constexpr uint8_t CMD_STAT             = 0x07;
constexpr uint8_t CMD_ROLLBACK         = 0x08;
constexpr uint8_t CMD_REBOOT           = 0x09;

// Commands (ACK)
constexpr uint8_t CMD_ACK_MASK         = 0x40;

// Status error flags
constexpr uint8_t ERR_INPUT_VOLTAGE    = 0x01;
constexpr uint8_t ERR_POS_LIMIT        = 0x02;
constexpr uint8_t ERR_TEMPERATURE      = 0x04;
constexpr uint8_t ERR_INVALID_PKT      = 0x08;
constexpr uint8_t ERR_OVERLOAD         = 0x10;
constexpr uint8_t ERR_DRIVER_FAULT     = 0x20;
constexpr uint8_t ERR_EEPREG_DISTORT   = 0x40;

// LED colors
constexpr uint8_t LED_OFF              = 0x00;
constexpr uint8_t LED_GREEN            = 0x01;
constexpr uint8_t LED_BLUE             = 0x02;
constexpr uint8_t LED_CYAN             = 0x03;
constexpr uint8_t LED_RED              = 0x04;
constexpr uint8_t LED_GREEN2           = 0x05;
constexpr uint8_t LED_PINK             = 0x06;
constexpr uint8_t LED_WHITE            = 0x07;

// RAM addresses (common to all models)
constexpr uint8_t ADDR_TORQUE_CONTROL  = 52;   // 0x34
constexpr uint8_t ADDR_LED_CONTROL     = 53;   // 0x35
constexpr uint8_t ADDR_STATUS_ERROR    = 48;   // 0x30
constexpr uint8_t ADDR_CAL_POSITION    = 58;   // 0x3A
constexpr uint8_t ADDR_SPEED           = 64;   // 0x40

// ─── PID Gain Register Addresses (EEP) ── common to all models
constexpr uint8_t EEP_ADDR_KP          = 30;   // Position Kp (2 bytes)
constexpr uint8_t EEP_ADDR_KD          = 32;   // Position Kd (2 bytes)
constexpr uint8_t EEP_ADDR_KI          = 34;   // Position Ki (2 bytes)
constexpr uint8_t EEP_ADDR_FEEDFORWARD1 = 36;  // Feedforward 1st gain (2 bytes)
constexpr uint8_t EEP_ADDR_FEEDFORWARD2 = 38;  // Feedforward 2nd gain (2 bytes)

// ─── PID Gain Register Addresses (RAM) ── common to all models
constexpr uint8_t RAM_ADDR_KP          = 24;   // Position Kp (2 bytes)
constexpr uint8_t RAM_ADDR_KD          = 26;   // Position Kd (2 bytes)
constexpr uint8_t RAM_ADDR_KI          = 28;   // Position Ki (2 bytes)
constexpr uint8_t RAM_ADDR_FEEDFORWARD1 = 30;  // Feedforward 1st gain (2 bytes)
constexpr uint8_t RAM_ADDR_FEEDFORWARD2 = 32;  // Feedforward 2nd gain (2 bytes)

// ─── Velocity Gain Register Addresses ── DRS-0x02 series only
constexpr uint8_t EEP_ADDR_VELOCITY_KP = 40;   // Velocity Kp (2 bytes, default 100)
constexpr uint8_t EEP_ADDR_VELOCITY_KI = 42;   // Velocity Ki (2 bytes, default 12000)
constexpr uint8_t RAM_ADDR_VELOCITY_KP = 34;   // Velocity Kp (2 bytes)
constexpr uint8_t RAM_ADDR_VELOCITY_KI = 36;   // Velocity Ki (2 bytes)

// Memory type for gain operations
constexpr uint8_t MEMORY_RAM           = 0;
constexpr uint8_t MEMORY_EEP           = 1;

// ─── Structs ───────────────────────────────────────────────────

struct ServoStatus
{
  uint8_t servo_id = 0;
  int position = 0;        // 0~1023
  float angle = 0.0f;      // -160.0~160.0
  int speed = 0;           // -1023~1023
  uint8_t status_error = 0;
  uint8_t status_detail = 0;
};

struct GainValues
{
  int kp = -1;              // Position P gain (0~32767, -1=invalid/skip)
  int kd = -1;              // Position D gain (0~32767, -1=invalid/skip)
  int ki = -1;              // Position I gain (0~32767, -1=invalid/skip)
  int feedforward1 = -1;    // Feedforward 1st gain (0~32767, -1=invalid/skip)
  int feedforward2 = -1;    // Feedforward 2nd gain (0~32767, -1=invalid/skip)
  int velocity_kp = -1;     // Velocity P gain (0x02 series, 0~32767, -1=not supported/skip)
  int velocity_ki = -1;     // Velocity I gain (0x02 series, 0~32767, -1=not supported/skip)
};

// ─── HerkuleX Serial Communication Class ───────────────────────

class HerkulexSerial
{
public:
  HerkulexSerial();
  ~HerkulexSerial();

  /// Open serial port
  bool open(const std::string & port, int baud_rate);

  /// Close serial port
  void close();

  /// Check if port is open
  bool isOpen() const;

  // ─── Model management ──────────────────────────────────────

  /// Set the default servo model (affects available features like velocity gains)
  void setModel(HerkulexModel model);

  /// Get the default servo model
  HerkulexModel getModel() const;

  /// Set model for a specific servo ID
  void setServoModel(uint8_t servo_id, HerkulexModel model);

  /// Get model for a specific servo ID (falls back to default model if not set)
  HerkulexModel getServoModel(uint8_t servo_id) const;

  /// Check if a servo model supports velocity gain registers
  bool supportsVelocityGain(uint8_t servo_id = 0) const;

  // ─── High-level servo control ───────────────────────────────

  /// Initialize all servos (clear error, torque ON)
  bool initialize();

  /// Get servo status error byte
  int stat(uint8_t servo_id);

  /// Set ACK policy (0=no reply, 1=reply to READ only, 2=always reply)
  bool setACK(int value);

  /// Clear error for a servo
  bool clearError(uint8_t servo_id);

  /// Set torque ON for a servo
  bool torqueOn(uint8_t servo_id);

  /// Set torque OFF (free) for a servo
  bool torqueOff(uint8_t servo_id);

  /// Move one servo to a position (0~1023) with playtime in ms
  bool moveOne(uint8_t servo_id, int goal, int playtime_ms, uint8_t led = 0);

  /// Move one servo to an angle (-160.0~160.0) with playtime in ms
  bool moveOneAngle(uint8_t servo_id, float angle, int playtime_ms, uint8_t led = 0);

  /// Move multiple servos synchronously to positions (raw counts) with playtime in ms using CMD_S_JOG
  bool moveMulti(
    const std::vector<uint8_t> & servo_ids,
    const std::vector<int> & goals,
    int playtime_ms,
    const std::vector<uint8_t> & leds = {});

  /// Move multiple servos synchronously to angles (-160.0~160.0) with playtime in ms using CMD_S_JOG
  bool moveMultiAngle(
    const std::vector<uint8_t> & servo_ids,
    const std::vector<float> & angles,
    int playtime_ms,
    const std::vector<uint8_t> & leds = {});

  /// Move one servo with continuous rotation speed (-1023~1023)
  bool moveSpeedOne(uint8_t servo_id, int speed, int playtime_ms, uint8_t led = 0);

  /// Get current position of a servo (returns -1 on error)
  int getPosition(uint8_t servo_id);

  /// Get current angle of a servo in degrees
  float getAngle(uint8_t servo_id);

  /// Get current speed of a servo
  int getSpeed(uint8_t servo_id);

  /// Reboot a servo
  bool reboot(uint8_t servo_id);

  /// Set LED color
  bool setLed(uint8_t servo_id, uint8_t led_color);

  /// Change servo ID (needs reboot after)
  bool setID(uint8_t old_id, uint8_t new_id);

  /// Write a single byte to RAM registry
  bool writeRegistryRAM(uint8_t servo_id, uint8_t address, uint8_t value);

  /// Write a single byte to EEP (ROM) registry
  bool writeRegistryEEP(uint8_t servo_id, uint8_t address, uint8_t value);

  /// Get full servo status
  ServoStatus getServoStatus(uint8_t servo_id);

  /// Read 2 bytes from RAM registry (returns value, -1 on error)
  int readRegistryRAM2(uint8_t servo_id, uint8_t address);

  /// Read 2 bytes from EEP registry (returns value, -1 on error)
  int readRegistryEEP2(uint8_t servo_id, uint8_t address);

  /// Write 2 bytes to RAM registry
  bool writeRegistryRAM2(uint8_t servo_id, uint8_t address, uint16_t value);

  /// Write 2 bytes to EEP registry
  bool writeRegistryEEP2(uint8_t servo_id, uint8_t address, uint16_t value);

  /// Set PID gain values (memory_type: 0=RAM, 1=EEP; use -1 to skip a field)
  bool setGain(uint8_t servo_id, uint8_t memory_type, const GainValues & gains);

  /// Get PID gain values (memory_type: 0=RAM, 1=EEP)
  GainValues getGain(uint8_t servo_id, uint8_t memory_type);

private:
  // ─── Low-level packet communication ─────────────────────────

  /// Build and send a packet
  bool sendPacket(uint8_t servo_id, uint8_t cmd,
    const std::vector<uint8_t> & data = {});

  /// Receive a response packet, returns data portion
  bool receivePacket(std::vector<uint8_t> & response, int expected_size, int timeout_ms = 80);

  /// Calculate checksum1
  static uint8_t checksum1(uint8_t packet_size, uint8_t servo_id,
    uint8_t cmd, const std::vector<uint8_t> & data);

  /// Calculate checksum2
  static uint8_t checksum2(uint8_t cs1);

  /// Build LED set byte from led_color
  static uint8_t buildSetByte(uint8_t led_color, bool speed_mode);

  /// Convert playtime in ms to protocol playtime value
  static uint8_t msToPlaytime(int ms);

  int fd_ = -1;
  HerkulexModel model_ = HerkulexModel::DRS_0101;
  std::map<uint8_t, HerkulexModel> servo_models_;
  std::mutex serial_mutex_;
};

}  // namespace herkulex_driver

#endif  // HERKULEX_DRIVER__HERKULEX_SERIAL_HPP_
