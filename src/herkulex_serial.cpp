// Copyright (c) 2026 HerkuleX ROS2 Driver
// Based on Herkulex Arduino Library by Alessandro Giacomel
// and HerkuleX C Library
// SPDX-License-Identifier: LGPL-2.1-or-later

#include "herkulex_driver/herkulex_serial.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include <cstring>
#include <chrono>
#include <thread>
#include <algorithm>

namespace herkulex_driver
{

// ─── Helper: convert baud rate int to termios constant ─────────
static speed_t baudToSpeed(int baud)
{
  switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 500000:  return B500000;
    case 576000:  return B576000;
    case 921600:  return B921600;
    case 1000000: return B1000000;
    default:      return B115200;
  }
}

// ─── Constructor / Destructor ──────────────────────────────────

HerkulexSerial::HerkulexSerial() = default;

HerkulexSerial::~HerkulexSerial()
{
  close();
}

// ─── Model Management ──────────────────────────────────────────

void HerkulexSerial::setModel(HerkulexModel model)
{
  model_ = model;
}

HerkulexModel HerkulexSerial::getModel() const
{
  return model_;
}

void HerkulexSerial::setServoModel(uint8_t servo_id, HerkulexModel model)
{
  servo_models_[servo_id] = model;
}

HerkulexModel HerkulexSerial::getServoModel(uint8_t servo_id) const
{
  auto it = servo_models_.find(servo_id);
  if (it != servo_models_.end()) {
    return it->second;
  }
  return model_;
}

bool HerkulexSerial::supportsVelocityGain(uint8_t servo_id) const
{
  return getModelSpec(getServoModel(servo_id)).has_velocity_gain;
}

// ─── Open / Close ──────────────────────────────────────────────

bool HerkulexSerial::open(const std::string & port, int baud_rate)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  // Configure serial port
  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));

  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  speed_t spd = baudToSpeed(baud_rate);
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  // 8N1, no flow control
  tty.c_cflag &= ~PARENB;        // No parity
  tty.c_cflag &= ~CSTOPB;        // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;            // 8 data bits
  tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
  tty.c_cflag |= CLOCAL | CREAD; // Enable receiver, ignore modem

  // Raw mode
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
  tty.c_oflag &= ~OPOST;

  // Read timeout settings
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;  // 100ms timeout

  tcflush(fd_, TCIFLUSH);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

void HerkulexSerial::close()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool HerkulexSerial::isOpen() const
{
  return fd_ >= 0;
}

// ─── Low-level Packet Communication ───────────────────────────

uint8_t HerkulexSerial::checksum1(
  uint8_t packet_size, uint8_t servo_id,
  uint8_t cmd, const std::vector<uint8_t> & data)
{
  uint8_t cs = packet_size ^ servo_id ^ cmd;
  for (auto b : data) {
    cs ^= b;
  }
  return cs & 0xFE;
}

uint8_t HerkulexSerial::checksum2(uint8_t cs1)
{
  return (~cs1) & 0xFE;
}

uint8_t HerkulexSerial::buildSetByte(uint8_t led_color, bool speed_mode)
{
  uint8_t set = 0;
  if (speed_mode) {
    set |= 0x02;  // mode bit = continuous rotation
  }
  // LED bits: green=bit2, blue=bit3, red=bit4
  switch (led_color) {
    case LED_GREEN:  set |= 0x04; break;
    case LED_BLUE:   set |= 0x08; break;
    case LED_CYAN:   set |= 0x0C; break;
    case LED_RED:    set |= 0x10; break;
    case LED_GREEN2: set |= 0x14; break;
    case LED_PINK:   set |= 0x18; break;
    case LED_WHITE:  set |= 0x1C; break;
    default: break;
  }
  return set;
}

uint8_t HerkulexSerial::msToPlaytime(int ms)
{
  if (ms <= 0) {return 0;}
  if (ms > 2856) {ms = 2856;}
  return static_cast<uint8_t>(static_cast<float>(ms) / 11.2f);
}

bool HerkulexSerial::sendPacket(
  uint8_t servo_id, uint8_t cmd,
  const std::vector<uint8_t> & data)
{
  if (fd_ < 0) {return false;}

  uint8_t packet_size = static_cast<uint8_t>(MIN_PACKET_SIZE + data.size());

  uint8_t cs1 = checksum1(packet_size, servo_id, cmd, data);
  uint8_t cs2 = checksum2(cs1);

  std::vector<uint8_t> packet;
  packet.reserve(packet_size);
  packet.push_back(HEADER_BYTE);       // Header 1
  packet.push_back(HEADER_BYTE);       // Header 2
  packet.push_back(packet_size);       // Packet size
  packet.push_back(servo_id);          // Servo ID
  packet.push_back(cmd);               // Command
  packet.push_back(cs1);               // Checksum 1
  packet.push_back(cs2);               // Checksum 2
  for (auto b : data) {
    packet.push_back(b);               // Data
  }

  // Flush input buffer before sending
  tcflush(fd_, TCIFLUSH);

  ssize_t written = ::write(fd_, packet.data(), packet.size());
  tcdrain(fd_);  // Wait until all bytes are transmitted

  return written == static_cast<ssize_t>(packet.size());
}

bool HerkulexSerial::receivePacket(
  std::vector<uint8_t> & response,
  int expected_size, int timeout_ms)
{
  if (fd_ < 0) {return false;}

  response.clear();
  response.resize(expected_size, 0);

  auto start = std::chrono::steady_clock::now();
  int total_read = 0;
  bool header_found = false;
  int header_idx = 0;

  std::vector<uint8_t> raw_buf;
  raw_buf.reserve(expected_size * 2);

  while (total_read < expected_size) {
    auto elapsed = std::chrono::steady_clock::now() - start;
    int remaining_ms = timeout_ms -
      static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
    if (remaining_ms <= 0) {
      return false;
    }

    // Use select for timeout
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    struct timeval tv;
    tv.tv_sec = remaining_ms / 1000;
    tv.tv_usec = (remaining_ms % 1000) * 1000;

    int sel = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (sel <= 0) {
      return false;
    }

    uint8_t buf[256];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) {
      continue;
    }

    for (ssize_t i = 0; i < n; i++) {
      raw_buf.push_back(buf[i]);
    }

    // Search for header 0xFF 0xFF and align
    if (!header_found) {
      for (size_t i = 0; i + 1 < raw_buf.size(); i++) {
        if (raw_buf[i] == HEADER_BYTE && raw_buf[i + 1] == HEADER_BYTE) {
          // Wait until we have at least MIN_PACKET_SIZE (7 bytes) to safely inspect header fields
          if (raw_buf.size() < i + MIN_PACKET_SIZE) {
            // Not enough bytes arrived yet, break and let next select/read accumulate more data
            break;
          }
          uint8_t pkt_sz = raw_buf[i + 2];
          uint8_t pkt_cmd = raw_buf[i + 4];
          // In HerkuleX protocol, a valid response packet MUST have ACK bit set: (cmd & 0x40) != 0.
          // Outgoing request packets have cmd < 0x40.
          // If half-duplex echo is received or packet size mismatches, skip this header!
          if ((pkt_cmd & 0x40) == 0 || (expected_size > 0 && pkt_sz != expected_size)) {
            continue;
          }
          header_found = true;
          header_idx = static_cast<int>(i);
          // Copy from header start
          int available = static_cast<int>(raw_buf.size()) - header_idx;
          int to_copy = std::min(available, expected_size);
          std::memcpy(response.data(), raw_buf.data() + header_idx, to_copy);
          total_read = to_copy;
          break;
        }
      }
    } else {
      // Continue reading remaining bytes
      int already = total_read;
      int available = static_cast<int>(raw_buf.size()) - header_idx;
      int to_copy = std::min(available, expected_size);
      if (to_copy > already) {
        std::memcpy(
          response.data() + already,
          raw_buf.data() + header_idx + already,
          to_copy - already);
        total_read = to_copy;
      }
    }
  }

  // Verify checksums
  if (total_read >= MIN_PACKET_SIZE) {
    uint8_t pkt_size = response[2];
    uint8_t pkt_id = response[3];
    uint8_t pkt_cmd = response[4];
    uint8_t pkt_cs1 = response[5];
    uint8_t pkt_cs2 = response[6];

    std::vector<uint8_t> data_part(response.begin() + 7, response.end());
    uint8_t calc_cs1 = checksum1(pkt_size, pkt_id, pkt_cmd, data_part);
    uint8_t calc_cs2 = checksum2(calc_cs1);

    if (calc_cs1 != pkt_cs1 || calc_cs2 != pkt_cs2) {
      return false;
    }
  }

  return true;
}

// ─── High-level servo control ──────────────────────────────────

bool HerkulexSerial::initialize()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  if (!clearError(BROADCAST_ID)) {return false;}
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  if (!setACK(1)) {return false;}
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  if (!torqueOn(BROADCAST_ID)) {return false;}
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  return true;
}

int HerkulexSerial::stat(uint8_t servo_id)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  if (!sendPacket(servo_id, CMD_STAT)) {
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  std::vector<uint8_t> response;
  if (!receivePacket(response, 9)) {
    return -1;
  }

  return response[7];
}

bool HerkulexSerial::setACK(int value)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  // HerkuleX RAM Address 0x07 (and EEP 0x07) is ACK Policy
  // 0: Reply only to READ commands (STAT, RAM_READ, EEP_READ)
  // 1: Reply to all commands
  // 2: Reply to all commands including broadcast
  std::vector<uint8_t> data = {
    0x07,                          // Address 7: ACK Policy
    0x01,                          // Length
    static_cast<uint8_t>(value)    // ACK value (1 = reply to all)
  };

  return sendPacket(BROADCAST_ID, CMD_RAM_WRITE, data);
}

bool HerkulexSerial::clearError(uint8_t servo_id)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  std::vector<uint8_t> data = {
    ADDR_STATUS_ERROR,  // Address 48 (0x30)
    0x02,               // Length = 2
    0x00,               // Clear status error
    0x00                // Clear status detail
  };

  return sendPacket(servo_id, CMD_RAM_WRITE, data);
}

bool HerkulexSerial::torqueOn(uint8_t servo_id)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  std::vector<uint8_t> data = {
    ADDR_TORQUE_CONTROL,  // Address 52
    0x01,                 // Length
    0x60                  // Torque ON
  };

  return sendPacket(servo_id, CMD_RAM_WRITE, data);
}

bool HerkulexSerial::torqueOff(uint8_t servo_id)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  std::vector<uint8_t> data = {
    ADDR_TORQUE_CONTROL,  // Address 52
    0x01,                 // Length
    0x00                  // Torque Free
  };

  return sendPacket(servo_id, CMD_RAM_WRITE, data);
}

bool HerkulexSerial::moveOne(uint8_t servo_id, int goal, int playtime_ms, uint8_t led)
{
  auto spec = getModelSpec(getServoModel(servo_id));
  if (goal < spec.min_position || goal > spec.max_position) {return false;}
  if (playtime_ms < 0 || playtime_ms > 2856) {return false;}

  std::lock_guard<std::mutex> lock(serial_mutex_);

  uint8_t pt = msToPlaytime(playtime_ms);
  uint8_t goalLSB = goal & 0xFF;
  uint8_t goalMSB = (goal >> 8) & 0xFF;
  uint8_t set_byte = buildSetByte(led, false);

  std::vector<uint8_t> data = {
    goalLSB,            // JOG LSB
    goalMSB,            // JOG MSB
    set_byte,           // SET (position mode + LED)
    servo_id            // Servo ID
  };

  // For S_JOG, checksum must include playtime XOR'd into ID field
  uint8_t packet_size = static_cast<uint8_t>(MIN_PACKET_SIZE + 1 + data.size());  // +1 for playtime
  // We need to build the packet manually for S_JOG
  std::vector<uint8_t> full_data;
  full_data.push_back(pt);  // Execution time
  for (auto b : data) {
    full_data.push_back(b);
  }

  // Custom checksum: ID field in checksum uses servo_id ^ playtime
  uint8_t cs_id = servo_id ^ pt;
  uint8_t cs1 = checksum1(packet_size, cs_id, CMD_S_JOG, data);
  uint8_t cs2 = checksum2(cs1);

  std::vector<uint8_t> packet;
  packet.push_back(HEADER_BYTE);
  packet.push_back(HEADER_BYTE);
  packet.push_back(packet_size);
  packet.push_back(servo_id);
  packet.push_back(CMD_S_JOG);
  packet.push_back(cs1);
  packet.push_back(cs2);
  packet.push_back(pt);    // Execution time
  for (auto b : data) {
    packet.push_back(b);
  }

  ssize_t written = ::write(fd_, packet.data(), packet.size());
  tcdrain(fd_);

  return written == static_cast<ssize_t>(packet.size());
}

bool HerkulexSerial::moveOneAngle(uint8_t servo_id, float angle, int playtime_ms, uint8_t led)
{
  if (angle > 160.0f || angle < -160.0f) {return false;}
  auto spec = getModelSpec(getServoModel(servo_id));
  int position = static_cast<int>(angle / spec.deg_per_count) + spec.center_position;
  return moveOne(servo_id, position, playtime_ms, led);
}

bool HerkulexSerial::moveMulti(
  const std::vector<uint8_t> & servo_ids,
  const std::vector<int> & goals,
  int playtime_ms,
  const std::vector<uint8_t> & leds)
{
  if (servo_ids.empty() || servo_ids.size() != goals.size()) {
    return false;
  }
  if (playtime_ms < 0 || playtime_ms > 2856) {
    return false;
  }

  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (fd_ < 0) {
    return false;
  }

  uint8_t pt = msToPlaytime(playtime_ms);

  // S_JOG data: first byte is playtime, followed by 4 bytes per servo [goalLSB, goalMSB, set_byte, id]
  std::vector<uint8_t> full_data;
  full_data.reserve(1 + servo_ids.size() * 4);
  full_data.push_back(pt);

  for (size_t i = 0; i < servo_ids.size(); ++i) {
    uint8_t id = servo_ids[i];
    auto spec = getModelSpec(getServoModel(id));
    int goal = goals[i];
    if (goal < spec.min_position) {goal = spec.min_position;}
    if (goal > spec.max_position) {goal = spec.max_position;}
    uint8_t led = (i < leds.size()) ? leds[i] : 0;

    uint8_t goalLSB = goal & 0xFF;
    uint8_t goalMSB = (goal >> 8) & 0xFF;
    uint8_t set_byte = buildSetByte(led, false);

    full_data.push_back(goalLSB);
    full_data.push_back(goalMSB);
    full_data.push_back(set_byte);
    full_data.push_back(id);
  }

  uint8_t packet_size = static_cast<uint8_t>(MIN_PACKET_SIZE + full_data.size());
  uint8_t target_pid = (servo_ids.size() == 1) ? servo_ids[0] : BROADCAST_ID;
  uint8_t cs1 = checksum1(packet_size, target_pid, CMD_S_JOG, full_data);
  uint8_t cs2 = checksum2(cs1);

  std::vector<uint8_t> packet;
  packet.reserve(packet_size);
  packet.push_back(HEADER_BYTE);
  packet.push_back(HEADER_BYTE);
  packet.push_back(packet_size);
  packet.push_back(target_pid);
  packet.push_back(CMD_S_JOG);
  packet.push_back(cs1);
  packet.push_back(cs2);
  for (auto b : full_data) {
    packet.push_back(b);
  }

  ssize_t written = ::write(fd_, packet.data(), packet.size());
  tcdrain(fd_);

  return written == static_cast<ssize_t>(packet.size());
}

bool HerkulexSerial::moveMultiAngle(
  const std::vector<uint8_t> & servo_ids,
  const std::vector<float> & angles,
  int playtime_ms,
  const std::vector<uint8_t> & leds)
{
  if (servo_ids.empty() || servo_ids.size() != angles.size()) {
    return false;
  }

  std::vector<int> goals;
  goals.reserve(servo_ids.size());

  for (size_t i = 0; i < servo_ids.size(); ++i) {
    float angle = angles[i];
    if (angle > 160.0f) {angle = 160.0f;}
    if (angle < -160.0f) {angle = -160.0f;}

    auto spec = getModelSpec(getServoModel(servo_ids[i]));
    int position = static_cast<int>(angle / spec.deg_per_count) + spec.center_position;
    if (position < spec.min_position) {position = spec.min_position;}
    if (position > spec.max_position) {position = spec.max_position;}
    goals.push_back(position);
  }

  return moveMulti(servo_ids, goals, playtime_ms, leds);
}

bool HerkulexSerial::moveSpeedOne(uint8_t servo_id, int speed, int playtime_ms, uint8_t led)
{
  if (speed > 1023 || speed < -1023) {return false;}
  if (playtime_ms < 0 || playtime_ms > 2856) {return false;}

  std::lock_guard<std::mutex> lock(serial_mutex_);

  uint8_t pt = msToPlaytime(playtime_ms);

  int goal_speed_sign = speed;
  if (speed < 0) {
    goal_speed_sign = (-speed) | 0x4000;  // bit 14 = negative direction
  }

  uint8_t speedLSB = goal_speed_sign & 0xFF;
  uint8_t speedMSB = (goal_speed_sign >> 8) & 0xFF;
  uint8_t set_byte = buildSetByte(led, true);

  std::vector<uint8_t> data = {
    speedLSB,
    speedMSB,
    set_byte,
    servo_id
  };

  uint8_t packet_size = static_cast<uint8_t>(MIN_PACKET_SIZE + 1 + data.size());

  uint8_t cs_id = servo_id ^ pt;
  uint8_t cs1_val = checksum1(packet_size, cs_id, CMD_S_JOG, data);
  uint8_t cs2_val = checksum2(cs1_val);

  std::vector<uint8_t> packet;
  packet.push_back(HEADER_BYTE);
  packet.push_back(HEADER_BYTE);
  packet.push_back(packet_size);
  packet.push_back(servo_id);
  packet.push_back(CMD_S_JOG);
  packet.push_back(cs1_val);
  packet.push_back(cs2_val);
  packet.push_back(pt);
  for (auto b : data) {
    packet.push_back(b);
  }

  ssize_t written = ::write(fd_, packet.data(), packet.size());
  tcdrain(fd_);

  return written == static_cast<ssize_t>(packet.size());
}

int HerkulexSerial::getPosition(uint8_t servo_id)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  std::vector<uint8_t> data = {
    ADDR_CAL_POSITION,    // Address 58 (0x3A)
    0x02                  // Length = 2 bytes
  };

  if (!sendPacket(servo_id, CMD_RAM_READ, data)) {
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  std::vector<uint8_t> response;
  if (!receivePacket(response, 13)) {
    return -1;
  }

  auto spec = getModelSpec(getServoModel(servo_id));
  int position = ((response[10] & spec.position_mask_msb) << 8) | response[9];
  return position;
}

float HerkulexSerial::getAngle(uint8_t servo_id)
{
  int pos = getPosition(servo_id);
  if (pos < 0) {return -999.0f;}
  auto spec = getModelSpec(getServoModel(servo_id));
  return (pos - spec.center_position) * spec.deg_per_count;
}

int HerkulexSerial::getSpeed(uint8_t servo_id)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  std::vector<uint8_t> data = {
    ADDR_SPEED,           // Address 64 (0x40)
    0x02                  // Length = 2 bytes
  };

  if (!sendPacket(servo_id, CMD_RAM_READ, data)) {
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  std::vector<uint8_t> response;
  if (!receivePacket(response, 13)) {
    return -1;
  }

  int speed = (static_cast<int>(response[10]) << 8) | response[9];
  return speed;
}

bool HerkulexSerial::reboot(uint8_t servo_id)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  return sendPacket(servo_id, CMD_REBOOT);
}

bool HerkulexSerial::setLed(uint8_t servo_id, uint8_t led_color)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  std::vector<uint8_t> data = {
    ADDR_LED_CONTROL,     // Address 53
    0x01,                 // Length
    led_color             // LED value
  };

  return sendPacket(servo_id, CMD_RAM_WRITE, data);
}

bool HerkulexSerial::setID(uint8_t old_id, uint8_t new_id)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  std::vector<uint8_t> data = {
    0x06,       // Address (Servo ID in EEP)
    0x01,       // Length
    new_id      // New ID
  };

  return sendPacket(old_id, CMD_EEP_WRITE, data);
}

bool HerkulexSerial::writeRegistryRAM(uint8_t servo_id, uint8_t address, uint8_t value)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  std::vector<uint8_t> data = {
    address,
    0x01,
    value
  };

  return sendPacket(servo_id, CMD_RAM_WRITE, data);
}

bool HerkulexSerial::writeRegistryEEP(uint8_t servo_id, uint8_t address, uint8_t value)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  std::vector<uint8_t> data = {
    address,
    0x01,
    value
  };

  return sendPacket(servo_id, CMD_EEP_WRITE, data);
}

ServoStatus HerkulexSerial::getServoStatus(uint8_t servo_id)
{
  ServoStatus status;
  status.servo_id = servo_id;
  status.position = -1;
  status.angle = 0.0f;
  status.speed = 0;
  status.status_error = 0;
  status.status_detail = 0;

  std::lock_guard<std::mutex> lock(serial_mutex_);

  // HerkuleX RAM Address 58 (0x3A): Calibrated Position (2 bytes)
  // RAM_READ response (13 bytes) includes:
  // [0..1] Header (0xFF, 0xFF)
  // [2]    Packet Size (13)
  // [3]    pID
  // [4]    CMD (0x44)
  // [5..6] Checksum 1, 2
  // [7]    Address (58)
  // [8]    Length (2)
  // [9]    Calibrated Position LSB
  // [10]   Calibrated Position MSB
  // [11]   Status Error byte
  // [12]   Status Detail byte
  // By reading Address 58 once, we obtain position, error status, and detail in a single round trip!
  std::vector<uint8_t> data = {
    ADDR_CAL_POSITION,    // Address 58 (0x3A)
    0x02                  // Length = 2 bytes
  };

  if (!sendPacket(servo_id, CMD_RAM_READ, data)) {
    return status;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  std::vector<uint8_t> response;
  if (!receivePacket(response, 13, 80)) {
    return status;
  }

  auto spec = getModelSpec(getServoModel(servo_id));
  status.position = ((response[10] & spec.position_mask_msb) << 8) | response[9];
  status.angle = (status.position - spec.center_position) * spec.deg_per_count;
  status.status_error = response[11];
  status.status_detail = response[12];

  return status;
}

// ─── 2-byte Registry Read/Write ────────────────────────────────

int HerkulexSerial::readRegistryRAM2(uint8_t servo_id, uint8_t address)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  std::vector<uint8_t> data = {
    address,
    0x02              // Length = 2 bytes
  };

  if (!sendPacket(servo_id, CMD_RAM_READ, data)) {
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  // Response: header(2) + size(1) + id(1) + cmd(1) + cs1(1) + cs2(1)
  //           + addr(1) + len(1) + data(2) + status_error(1) + status_detail(1) = 13
  std::vector<uint8_t> response;
  if (!receivePacket(response, 13)) {
    return -1;
  }

  int value = (static_cast<int>(response[10]) << 8) | response[9];
  return value;
}

int HerkulexSerial::readRegistryEEP2(uint8_t servo_id, uint8_t address)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  std::vector<uint8_t> data = {
    address,
    0x02              // Length = 2 bytes
  };

  if (!sendPacket(servo_id, CMD_EEP_READ, data)) {
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  std::vector<uint8_t> response;
  if (!receivePacket(response, 13)) {
    return -1;
  }

  int value = (static_cast<int>(response[10]) << 8) | response[9];
  return value;
}

bool HerkulexSerial::writeRegistryRAM2(uint8_t servo_id, uint8_t address, uint16_t value)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  std::vector<uint8_t> data = {
    address,
    0x02,                                // Length = 2 bytes
    static_cast<uint8_t>(value & 0xFF),  // LSB
    static_cast<uint8_t>(value >> 8)     // MSB
  };

  return sendPacket(servo_id, CMD_RAM_WRITE, data);
}

bool HerkulexSerial::writeRegistryEEP2(uint8_t servo_id, uint8_t address, uint16_t value)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  std::vector<uint8_t> data = {
    address,
    0x02,                                // Length = 2 bytes
    static_cast<uint8_t>(value & 0xFF),  // LSB
    static_cast<uint8_t>(value >> 8)     // MSB
  };

  return sendPacket(servo_id, CMD_EEP_WRITE, data);
}

// ─── PID Gain Read/Write ───────────────────────────────────────

bool HerkulexSerial::setGain(uint8_t servo_id, uint8_t memory_type, const GainValues & gains)
{
  struct GainEntry {
    uint8_t ram_addr;
    uint8_t eep_addr;
    int value;
  };

  // Position PID gains (common to all models)
  std::vector<GainEntry> entries = {
    { RAM_ADDR_KP,          EEP_ADDR_KP,          gains.kp },
    { RAM_ADDR_KD,          EEP_ADDR_KD,          gains.kd },
    { RAM_ADDR_KI,          EEP_ADDR_KI,          gains.ki },
    { RAM_ADDR_FEEDFORWARD1, EEP_ADDR_FEEDFORWARD1, gains.feedforward1 },
    { RAM_ADDR_FEEDFORWARD2, EEP_ADDR_FEEDFORWARD2, gains.feedforward2 },
  };

  // Velocity gains (DRS-0x02 series only: DRS-0102, DRS-0402, DRS-0602)
  if (supportsVelocityGain(servo_id)) {
    entries.push_back({ RAM_ADDR_VELOCITY_KP, EEP_ADDR_VELOCITY_KP, gains.velocity_kp });
    entries.push_back({ RAM_ADDR_VELOCITY_KI, EEP_ADDR_VELOCITY_KI, gains.velocity_ki });
  }

  for (const auto & entry : entries) {
    // -1 means "leave unchanged"
    if (entry.value < 0) {
      continue;
    }
    uint16_t val = static_cast<uint16_t>(entry.value & 0x7FFF);
    bool ok;
    if (memory_type == MEMORY_EEP) {
      ok = writeRegistryEEP2(servo_id, entry.eep_addr, val);
    } else {
      ok = writeRegistryRAM2(servo_id, entry.ram_addr, val);
    }
    if (!ok) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return true;
}

GainValues HerkulexSerial::getGain(uint8_t servo_id, uint8_t memory_type)
{
  GainValues gains;

  struct GainAddr {
    uint8_t ram_addr;
    uint8_t eep_addr;
    int * dest;
  };

  // Position PID gains (common to all models)
  std::vector<GainAddr> addrs = {
    { RAM_ADDR_KP,          EEP_ADDR_KP,          &gains.kp },
    { RAM_ADDR_KD,          EEP_ADDR_KD,          &gains.kd },
    { RAM_ADDR_KI,          EEP_ADDR_KI,          &gains.ki },
    { RAM_ADDR_FEEDFORWARD1, EEP_ADDR_FEEDFORWARD1, &gains.feedforward1 },
    { RAM_ADDR_FEEDFORWARD2, EEP_ADDR_FEEDFORWARD2, &gains.feedforward2 },
  };

  // Velocity gains (DRS-0x02 series only)
  if (supportsVelocityGain(servo_id)) {
    addrs.push_back({ RAM_ADDR_VELOCITY_KP, EEP_ADDR_VELOCITY_KP, &gains.velocity_kp });
    addrs.push_back({ RAM_ADDR_VELOCITY_KI, EEP_ADDR_VELOCITY_KI, &gains.velocity_ki });
  }

  for (auto & a : addrs) {
    int val;
    if (memory_type == MEMORY_EEP) {
      val = readRegistryEEP2(servo_id, a.eep_addr);
    } else {
      val = readRegistryRAM2(servo_id, a.ram_addr);
    }
    *(a.dest) = val;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  return gains;
}

}  // namespace herkulex_driver
