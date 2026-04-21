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
  this->declare_parameter<std::string>("model", "0101");
  this->declare_parameter<std::vector<int64_t>>("servo_ids", {0});
  this->declare_parameter<double>("status_rate", 10.0);
  this->declare_parameter<bool>("auto_initialize", true);

  serial_port_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  model_name_ = this->get_parameter("model").as_string();
  servo_ids_ = this->get_parameter("servo_ids").as_integer_array();
  status_rate_ = this->get_parameter("status_rate").as_double();
  auto_initialize_ = this->get_parameter("auto_initialize").as_bool();

  auto model = parseModelString(model_name_);
  auto model_spec = getModelSpec(model);

  RCLCPP_INFO(this->get_logger(),
    "HerkuleX Driver - Model: %s, Port: %s, Baud: %d, Servos: %zu",
    model_spec.name.c_str(), serial_port_.c_str(), baud_rate_, servo_ids_.size());
  if (model_spec.has_velocity_gain) {
    RCLCPP_INFO(this->get_logger(),
      "Model %s supports velocity gains (Kp/Ki)", model_spec.name.c_str());
  }

  // ─── Initialize serial communication ─────────────────────────
  serial_ = std::make_unique<HerkulexSerial>();  serial_->setModel(model);
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

  for (auto id : servo_ids_) {
    auto sv = serial_->getServoStatus(static_cast<uint8_t>(id));

    msg::ServoStatus servo_msg;
    servo_msg.servo_id = sv.servo_id;
    servo_msg.position = sv.position;
    servo_msg.angle = sv.angle;
    servo_msg.speed = sv.speed;
    servo_msg.status_error = sv.status_error;
    servo_msg.status_detail = sv.status_detail;

    status_msg.servos.push_back(servo_msg);
  }

  status_pub_->publish(status_msg);
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
    response->success = true;
    response->position = pos;
    response->angle = (pos - 512) * 0.325f;
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
