// Copyright (c) 2026 HerkuleX ROS2 Driver
// SPDX-License-Identifier: LGPL-2.1-or-later

#include "herkulex_driver/herkulex_node.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace herkulex_driver
{

HerkulexNode::HerkulexNode(const rclcpp::NodeOptions & options)
: Node("herkulex_driver", options)
{
  // ─── Declare parameters ──────────────────────────────────────
  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);

  rcl_interfaces::msg::ParameterDescriptor model_desc;
  model_desc.description = "Default HerkuleX servo model (e.g. '0602', 'DRS-0602', or 602)";
  model_desc.dynamic_typing = true;
  this->declare_parameter("model", rclcpp::ParameterValue("0602"), model_desc);

  this->declare_parameter<std::vector<int64_t>>("servo_ids", {0});
  this->declare_parameter<double>("status_rate", 10.0);
  this->declare_parameter<bool>("auto_initialize", true);
  this->declare_parameter<bool>("auto_torque_on", true);

  rcl_interfaces::msg::ParameterDescriptor servo_models_desc;
  servo_models_desc.description = "Per-servo model overrides (e.g. ['1:0602', '2:0602', '3:0201'])";
  servo_models_desc.dynamic_typing = true;
  this->declare_parameter("servo_models", rclcpp::ParameterValue(std::vector<std::string>{}), servo_models_desc);

  this->declare_parameter<double>("max_sync_packet_age_sec", 0.0);

  serial_port_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();

  // Model parameter: robustly handle both string ("0602") and integer (602) from YAML/CLI
  auto model_param = this->get_parameter("model");
  if (model_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%04ld", model_param.as_int());
    model_name_ = buf;
  } else if (model_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    model_name_ = model_param.as_string();
  } else if (model_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    auto list = model_param.as_string_array();
    if (!list.empty()) {
      auto sep = list[0].find(':');
      model_name_ = (sep != std::string::npos) ? list[0].substr(sep + 1) : list[0];
    }
  } else {
    model_name_ = "0602";
  }

  servo_ids_ = this->get_parameter("servo_ids").as_integer_array();
  status_rate_ = this->get_parameter("status_rate").as_double();
  auto_initialize_ = this->get_parameter("auto_initialize").as_bool();
  auto_torque_on_ = this->get_parameter("auto_torque_on").as_bool();
  max_sync_packet_age_sec_ = this->get_parameter("max_sync_packet_age_sec").as_double();

  auto model = parseModelString(model_name_);
  auto model_spec = getModelSpec(model);

  RCLCPP_INFO(this->get_logger(),
    "HerkuleX Driver - Default Model: %s, Port: %s, Baud: %d, Servos: %zu",
    model_spec.name.c_str(), serial_port_.c_str(), baud_rate_, servo_ids_.size());
  if (model_spec.has_velocity_gain) {
    RCLCPP_INFO(this->get_logger(),
      "Model %s supports velocity gains (Kp/Ki)", model_spec.name.c_str());
  }

  // ─── Initialize serial communication ─────────────────────────
  serial_ = std::make_unique<HerkulexSerial>();
  serial_->setModel(model);

  // Set per-servo model overrides if specified (e.g. ["1:0602", "2:0602", "3:0201"])
  std::vector<std::string> servo_models_list;
  auto sm_param = this->get_parameter("servo_models");
  if (sm_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    servo_models_list = sm_param.as_string_array();
  } else if (model_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    // If user passed the list directly to `model: ["1:0602", "2:0602", "3:0201"]`!
    servo_models_list = model_param.as_string_array();
  }

  for (const auto & entry : servo_models_list) {
    auto sep = entry.find(':');
    if (sep != std::string::npos) {
      try {
        uint8_t id = static_cast<uint8_t>(std::stoi(entry.substr(0, sep)));
        std::string m_str = entry.substr(sep + 1);
        auto m_enum = parseModelString(m_str);
        serial_->setServoModel(id, m_enum);
        RCLCPP_INFO(this->get_logger(),
          "Servo ID %d assigned model: %s", id, getModelSpec(m_enum).name.c_str());
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse servo_models entry '%s': %s", entry.c_str(), e.what());
      }
    }
  }
  if (!serial_->open(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to open serial port: %s", serial_port_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");

    if (auto_initialize_) {
      if (serial_->initialize()) {
        RCLCPP_INFO(this->get_logger(), "Servos initialized (clear error + torque ON)");
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to initialize servos");
      }
    }
  }

  // ─── Create publisher ────────────────────────────────────────
  status_pub_ = this->create_publisher<msg::ServoStatusArray>("herkulex/status", 10);

  // ─── Create subscriptions (Real-time synchronized control) ───
  cmd_sync_angle_sub_ = this->create_subscription<msg::SyncAngleCmd>(
    "herkulex/cmd_sync_angle",
    rclcpp::QoS(1),
    std::bind(&HerkulexNode::onCmdSyncAngle, this, _1));

  cmd_sync_position_sub_ = this->create_subscription<msg::SyncPositionCmd>(
    "herkulex/cmd_sync_position",
    rclcpp::QoS(1),
    std::bind(&HerkulexNode::onCmdSyncPosition, this, _1));

  // ─── Create timer for status publishing ──────────────────────
  if (status_rate_ > 0.0 && !servo_ids_.empty()) {
    auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / status_rate_));
    status_timer_ = this->create_wall_timer(
      period, std::bind(&HerkulexNode::statusTimerCallback, this));
  }

  // ─── Create services ─────────────────────────────────────────
  set_position_srv_ = this->create_service<srv::SetPosition>(
    "herkulex/set_position",
    std::bind(&HerkulexNode::onSetPosition, this, _1, _2));

  set_angle_srv_ = this->create_service<srv::SetAngle>(
    "herkulex/set_angle",
    std::bind(&HerkulexNode::onSetAngle, this, _1, _2));

  set_speed_srv_ = this->create_service<srv::SetSpeed>(
    "herkulex/set_speed",
    std::bind(&HerkulexNode::onSetSpeed, this, _1, _2));

  set_torque_srv_ = this->create_service<srv::SetTorque>(
    "herkulex/set_torque",
    std::bind(&HerkulexNode::onSetTorque, this, _1, _2));

  set_led_srv_ = this->create_service<srv::SetLed>(
    "herkulex/set_led",
    std::bind(&HerkulexNode::onSetLed, this, _1, _2));

  get_position_srv_ = this->create_service<srv::GetPosition>(
    "herkulex/get_position",
    std::bind(&HerkulexNode::onGetPosition, this, _1, _2));

  reboot_srv_ = this->create_service<srv::Reboot>(
    "herkulex/reboot",
    std::bind(&HerkulexNode::onReboot, this, _1, _2));

  clear_error_srv_ = this->create_service<srv::ClearError>(
    "herkulex/clear_error",
    std::bind(&HerkulexNode::onClearError, this, _1, _2));

  set_id_srv_ = this->create_service<srv::SetID>(
    "herkulex/set_id",
    std::bind(&HerkulexNode::onSetID, this, _1, _2));

  write_registry_srv_ = this->create_service<srv::WriteRegistry>(
    "herkulex/write_registry",
    std::bind(&HerkulexNode::onWriteRegistry, this, _1, _2));

  set_gain_srv_ = this->create_service<srv::SetGain>(
    "herkulex/set_gain",
    std::bind(&HerkulexNode::onSetGain, this, _1, _2));

  get_gain_srv_ = this->create_service<srv::GetGain>(
    "herkulex/get_gain",
    std::bind(&HerkulexNode::onGetGain, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "HerkuleX Driver node ready");
}

HerkulexNode::~HerkulexNode()
{
  if (serial_) {
    serial_->close();
  }
}

// ─── Timer Callback ────────────────────────────────────────────

void HerkulexNode::statusTimerCallback()
{
  if (!serial_ || !serial_->isOpen()) {
    return;
  }

  auto status_msg = msg::ServoStatusArray();
  status_msg.header.stamp = this->now();
  status_msg.header.frame_id = "herkulex";

  // Persistent tracking for unresponsive servos to prevent stalling the active servos
  static std::map<int64_t, int> fail_counts;
  static std::map<int64_t, int> backoff_ticks;

  for (auto id : servo_ids_) {
    msg::ServoStatus servo_msg;
    servo_msg.servo_id = static_cast<uint8_t>(id);

    // If servo has failed repeatedly, back off polling to avoid blocking the bus
    if (backoff_ticks[id] > 0) {
      backoff_ticks[id]--;
      // Include offline status so telemetry reflects the servo state
      servo_msg.position = -1;
      servo_msg.angle = 0.0f;
      servo_msg.speed = 0;
      servo_msg.status_error = 0xFF;
      servo_msg.status_detail = 0xFF;
      status_msg.servos.push_back(servo_msg);
      continue;
    }
    // If servo was previously unreachable (backoff just expired), re-initialize before retrying.
    // After hot-plug, the servo reboots with EEPROM defaults — ACK Policy may be 0 (no response
    // to RAM_READ), so getServoStatus() would always time out. We must restore ACK Policy (RAM Addr 1),
    // clear errors, and enable torque via TX-only commands before attempting to read status.
    if (auto_torque_on_ && fail_counts[id] >= 3) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "HerkuleX: Servo ID %ld - Re-initializing after backoff (clearError + setACK + torqueOn)...", id);
      serial_->clearError(static_cast<uint8_t>(id));
      serial_->setACK(static_cast<uint8_t>(id), 1);  // Set ACK Policy to 1 (reply to READ) for this servo
      serial_->torqueOn(static_cast<uint8_t>(id));
      fail_counts[id] = 0;  // Reset to give fresh retries before next backoff
    }

    auto sv = serial_->getServoStatus(static_cast<uint8_t>(id));

    if (sv.position < 0) {
      fail_counts[id]++;
      if (fail_counts[id] >= 3) {
        // Back off for 20 timer ticks (~2 seconds at 10Hz) before retrying
        backoff_ticks[id] = 20;
      }

      // Check if servo at least responds to STAT (which always replies by firmware spec, regardless of ACK Policy)
      int stat_res = serial_->stat(static_cast<uint8_t>(id));

      if (stat_res >= 0) {
        // Servo responds to STAT! Hardware RX wire is working. Re-apply setACK for RAM_READ.
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "HerkuleX: Servo ID %ld - STAT command OK (status_error=0x%02X), but RAM_READ failed (RX: %d bytes). Re-applying setACK(1)...",
          id, stat_res, serial_->last_rx_bytes_);
        serial_->setACK(static_cast<uint8_t>(id), 1);
      } else {
        // Servo does not even respond to STAT (0 bytes). Hardware RX communication failure.
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "HerkuleX: Servo ID %ld completely silent (STAT & RAM_READ timed out, RX: 0 bytes). Check RX line: Servo TXD pin -> USB adapter RXD pin",
          id);
      }

      servo_msg.position = -1;
      servo_msg.angle = 0.0f;
      servo_msg.speed = 0;
      servo_msg.status_error = 0xFF;
      servo_msg.status_detail = 0xFF;
    } else {
      // Success: reset fail count
      fail_counts[id] = 0;
      backoff_ticks[id] = 0;

      // Check if torque is OFF, and automatically turn it ON if auto_torque_on_ is enabled
      // In Herkulex protocol, Status Detail byte Bit 6 (0x40) represents Torque ON state (0 = Torque Free/OFF)
      if (auto_torque_on_) {
        bool torque_on = (sv.status_detail & 0x40) != 0;
        if (!torque_on) {
          RCLCPP_INFO(this->get_logger(),
            "HerkuleX: Servo ID %ld responded but torque is OFF (detail=0x%02X). Automatically turning torque ON...",
            id, sv.status_detail);
          if (sv.status_error != 0) {
            serial_->clearError(static_cast<uint8_t>(id));
          }
          if (serial_->torqueOn(static_cast<uint8_t>(id))) {
            RCLCPP_INFO(this->get_logger(), "HerkuleX: Servo ID %ld torque successfully enabled!", id);
            sv.status_detail |= 0x40;  // Reflect torque ON state
          } else {
            RCLCPP_WARN(this->get_logger(), "HerkuleX: Failed to enable torque for Servo ID %ld", id);
          }
        }
      }

      servo_msg.position = sv.position;
      servo_msg.angle = sv.angle;
      servo_msg.speed = sv.speed;
      servo_msg.status_error = sv.status_error;
      servo_msg.status_detail = sv.status_detail;
    }

    status_msg.servos.push_back(servo_msg);
  }

  // Always publish status message to keep topic rate steady and inform subscribers
  if (!status_msg.servos.empty()) {
    status_pub_->publish(status_msg);
  }
}

// ─── Service Callbacks ─────────────────────────────────────────

void HerkulexNode::onSetPosition(
  const std::shared_ptr<srv::SetPosition::Request> request,
  std::shared_ptr<srv::SetPosition::Response> response)
{
  if (!serial_ || !serial_->isOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    return;
  }

  bool ok = serial_->moveOne(
    request->servo_id, request->goal_position,
    request->playtime_ms, request->led_color);

  response->success = ok;
  response->message = ok ? "Position set" : "Failed to set position";

  RCLCPP_DEBUG(this->get_logger(),
    "SetPosition: ID=%d, Goal=%d, Time=%dms -> %s",
    request->servo_id, request->goal_position,
    request->playtime_ms, ok ? "OK" : "FAIL");
}

void HerkulexNode::onSetAngle(
  const std::shared_ptr<srv::SetAngle::Request> request,
  std::shared_ptr<srv::SetAngle::Response> response)
{
  if (!serial_ || !serial_->isOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    return;
  }

  bool ok = serial_->moveOneAngle(
    request->servo_id, request->goal_angle,
    request->playtime_ms, request->led_color);

  response->success = ok;
  response->message = ok ? "Angle set" : "Failed to set angle";
}

void HerkulexNode::onSetSpeed(
  const std::shared_ptr<srv::SetSpeed::Request> request,
  std::shared_ptr<srv::SetSpeed::Response> response)
{
  if (!serial_ || !serial_->isOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    return;
  }

  bool ok = serial_->moveSpeedOne(
    request->servo_id, request->goal_speed,
    request->playtime_ms, request->led_color);

  response->success = ok;
  response->message = ok ? "Speed set" : "Failed to set speed";
}

void HerkulexNode::onSetTorque(
  const std::shared_ptr<srv::SetTorque::Request> request,
  std::shared_ptr<srv::SetTorque::Response> response)
{
  if (!serial_ || !serial_->isOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    return;
  }

  bool ok;
  if (request->torque_on) {
    ok = serial_->torqueOn(request->servo_id);
  } else {
    ok = serial_->torqueOff(request->servo_id);
  }

  response->success = ok;
  response->message = ok ? "Torque updated" : "Failed to set torque";
}

void HerkulexNode::onSetLed(
  const std::shared_ptr<srv::SetLed::Request> request,
  std::shared_ptr<srv::SetLed::Response> response)
{
  if (!serial_ || !serial_->isOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    return;
  }

  bool ok = serial_->setLed(request->servo_id, request->led_color);

  response->success = ok;
  response->message = ok ? "LED set" : "Failed to set LED";
}

void HerkulexNode::onGetPosition(
  const std::shared_ptr<srv::GetPosition::Request> request,
  std::shared_ptr<srv::GetPosition::Response> response)
{
  if (!serial_ || !serial_->isOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    return;
  }

  int pos = serial_->getPosition(request->servo_id);
  if (pos >= 0) {
    auto spec = getModelSpec(serial_->getServoModel(request->servo_id));
    response->success = true;
    response->position = pos;
    response->angle = (pos - spec.center_position) * spec.deg_per_count;
    response->message = "OK";
  } else {
    response->success = false;
    response->message = "Failed to read position";
  }
}

void HerkulexNode::onReboot(
  const std::shared_ptr<srv::Reboot::Request> request,
  std::shared_ptr<srv::Reboot::Response> response)
{
  if (!serial_ || !serial_->isOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    return;
  }

  bool ok = serial_->reboot(request->servo_id);

  response->success = ok;
  response->message = ok ? "Reboot sent" : "Failed to reboot";
}

void HerkulexNode::onClearError(
  const std::shared_ptr<srv::ClearError::Request> request,
  std::shared_ptr<srv::ClearError::Response> response)
{
  if (!serial_ || !serial_->isOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    return;
  }

  bool ok = serial_->clearError(request->servo_id);

  response->success = ok;
  response->message = ok ? "Error cleared" : "Failed to clear error";
}

void HerkulexNode::onSetID(
  const std::shared_ptr<srv::SetID::Request> request,
  std::shared_ptr<srv::SetID::Response> response)
{
  if (!serial_ || !serial_->isOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    return;
  }

  bool ok = serial_->setID(request->old_id, request->new_id);

  response->success = ok;
  response->message = ok ? "ID changed (reboot servo to apply)" : "Failed to change ID";
}

void HerkulexNode::onWriteRegistry(
  const std::shared_ptr<srv::WriteRegistry::Request> request,
  std::shared_ptr<srv::WriteRegistry::Response> response)
{
  if (!serial_ || !serial_->isOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    return;
  }

  bool ok;
  if (request->memory_type == srv::WriteRegistry::Request::MEMORY_RAM) {
    ok = serial_->writeRegistryRAM(request->servo_id, request->address, request->value);
  } else {
    ok = serial_->writeRegistryEEP(request->servo_id, request->address, request->value);
  }

  response->success = ok;
  response->message = ok ? "Registry written" : "Failed to write registry";
}

void HerkulexNode::onSetGain(
  const std::shared_ptr<srv::SetGain::Request> request,
  std::shared_ptr<srv::SetGain::Response> response)
{
  if (!serial_ || !serial_->isOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    return;
  }

  GainValues gains;
  gains.kp = request->kp;
  gains.kd = request->kd;
  gains.ki = request->ki;
  gains.feedforward1 = request->feedforward1;
  gains.feedforward2 = request->feedforward2;
  gains.velocity_kp = request->velocity_kp;
  gains.velocity_ki = request->velocity_ki;

  bool ok = serial_->setGain(request->servo_id, request->memory_type, gains);

  response->success = ok;
  response->message = ok ? "Gain values set" : "Failed to set gain values";

  RCLCPP_DEBUG(this->get_logger(),
    "SetGain: ID=%d, Mem=%d, Model=%s, Kp=%d, Kd=%d, Ki=%d, FF1=%d, FF2=%d, VKp=%d, VKi=%d -> %s",
    request->servo_id, request->memory_type,
    getModelSpec(serial_->getModel()).name.c_str(),
    request->kp, request->kd, request->ki,
    request->feedforward1, request->feedforward2,
    request->velocity_kp, request->velocity_ki,
    ok ? "OK" : "FAIL");
}

void HerkulexNode::onGetGain(
  const std::shared_ptr<srv::GetGain::Request> request,
  std::shared_ptr<srv::GetGain::Response> response)
{
  if (!serial_ || !serial_->isOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    return;
  }

  auto gains = serial_->getGain(request->servo_id, request->memory_type);

  // Check if any read failed (all -1 means total failure)
  if (gains.kp < 0 && gains.kd < 0 && gains.ki < 0 &&
      gains.feedforward1 < 0 && gains.feedforward2 < 0) {
    response->success = false;
    response->message = "Failed to read gain values";
    return;
  }

  response->success = true;
  response->message = "OK";
  response->kp = gains.kp;
  response->kd = gains.kd;
  response->ki = gains.ki;
  response->feedforward1 = gains.feedforward1;
  response->feedforward2 = gains.feedforward2;
  response->velocity_kp = gains.velocity_kp;  // -1 if not supported
  response->velocity_ki = gains.velocity_ki;  // -1 if not supported

  RCLCPP_DEBUG(this->get_logger(),
    "GetGain: ID=%d, Mem=%d, Model=%s -> Kp=%d, Kd=%d, Ki=%d, FF1=%d, FF2=%d, VKp=%d, VKi=%d",
    request->servo_id, request->memory_type,
    getModelSpec(serial_->getModel()).name.c_str(),
    gains.kp, gains.kd, gains.ki,
    gains.feedforward1, gains.feedforward2,
    gains.velocity_kp, gains.velocity_ki);
}

// ─── Topic Callbacks (Synchronized Multi-Servo Control) ────────

void HerkulexNode::onCmdSyncAngle(const msg::SyncAngleCmd::SharedPtr msg)
{
  if (!serial_ || !serial_->isOpen()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Cannot execute sync angle command: Serial port is not open");
    return;
  }

  // 1. Timestamp filtering (if timestamp provided)
  bool has_stamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0);
  if (has_stamp) {
    rclcpp::Time msg_stamp(msg->header.stamp);

    // Filter out-of-order packets
    if (last_sync_angle_stamp_.nanoseconds() > 0 && msg_stamp < last_sync_angle_stamp_) {
      RCLCPP_DEBUG(this->get_logger(),
        "Dropping out-of-order sync angle cmd (msg stamp: %f < last stamp: %f)",
        msg_stamp.seconds(), last_sync_angle_stamp_.seconds());
      return;
    }

    // Filter stale packets (network congestion / queue delay, only if enabled > 0.0)
    if (max_sync_packet_age_sec_ > 0.0) {
      double age = (this->now() - msg_stamp).seconds();
      if (age > max_sync_packet_age_sec_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Dropping stale sync angle cmd (age: %.3fs > max: %.3fs). Check system clock synchronization!",
          age, max_sync_packet_age_sec_);
        return;
      }
    }

    last_sync_angle_stamp_ = msg_stamp;
  }

  // 2. Validate array sizes
  if (msg->servo_ids.empty() || msg->servo_ids.size() != msg->target_angles.size()) {
    RCLCPP_WARN(this->get_logger(),
      "Invalid SyncAngleCmd: servo_ids size (%zu) != target_angles size (%zu)",
      msg->servo_ids.size(), msg->target_angles.size());
    return;
  }

  // 3. Convert target_angles (float64[]) to float
  std::vector<float> angles;
  angles.reserve(msg->target_angles.size());
  for (auto a : msg->target_angles) {
    angles.push_back(static_cast<float>(a));
  }

  // 4. Send multi-servo S_JOG command
  if (!serial_->moveMultiAngle(msg->servo_ids, angles, msg->playtime_ms, msg->led_colors)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "serial_->moveMultiAngle failed");
  }
}

void HerkulexNode::onCmdSyncPosition(const msg::SyncPositionCmd::SharedPtr msg)
{
  if (!serial_ || !serial_->isOpen()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Cannot execute sync position command: Serial port is not open");
    return;
  }

  // 1. Timestamp filtering (if timestamp provided)
  bool has_stamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0);
  if (has_stamp) {
    rclcpp::Time msg_stamp(msg->header.stamp);

    // Filter out-of-order packets
    if (last_sync_pos_stamp_.nanoseconds() > 0 && msg_stamp < last_sync_pos_stamp_) {
      RCLCPP_DEBUG(this->get_logger(),
        "Dropping out-of-order sync position cmd (msg stamp: %f < last stamp: %f)",
        msg_stamp.seconds(), last_sync_pos_stamp_.seconds());
      return;
    }

    // Filter stale packets (network congestion / queue delay, only if enabled > 0.0)
    if (max_sync_packet_age_sec_ > 0.0) {
      double age = (this->now() - msg_stamp).seconds();
      if (age > max_sync_packet_age_sec_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Dropping stale sync position cmd (age: %.3fs > max: %.3fs). Check system clock synchronization!",
          age, max_sync_packet_age_sec_);
        return;
      }
    }

    last_sync_pos_stamp_ = msg_stamp;
  }

  // 2. Validate array sizes
  if (msg->servo_ids.empty() || msg->servo_ids.size() != msg->target_positions.size()) {
    RCLCPP_WARN(this->get_logger(),
      "Invalid SyncPositionCmd: servo_ids size (%zu) != target_positions size (%zu)",
      msg->servo_ids.size(), msg->target_positions.size());
    return;
  }

  // 3. Convert target_positions (int32[]) to int
  std::vector<int> goals(msg->target_positions.begin(), msg->target_positions.end());

  // 4. Send multi-servo S_JOG command
  if (!serial_->moveMulti(msg->servo_ids, goals, msg->playtime_ms, msg->led_colors)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "serial_->moveMulti failed");
  }
}

}  // namespace herkulex_driver

// ─── Main ──────────────────────────────────────────────────────

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(herkulex_driver::HerkulexNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<herkulex_driver::HerkulexNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
