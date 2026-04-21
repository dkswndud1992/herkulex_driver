// Copyright (c) 2026 HerkuleX ROS2 Driver
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef HERKULEX_DRIVER__HERKULEX_NODE_HPP_
#define HERKULEX_DRIVER__HERKULEX_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "herkulex_driver/herkulex_serial.hpp"

#include "herkulex_driver/msg/servo_status.hpp"
#include "herkulex_driver/msg/servo_status_array.hpp"
#include "herkulex_driver/srv/set_position.hpp"
#include "herkulex_driver/srv/set_angle.hpp"
#include "herkulex_driver/srv/set_speed.hpp"
#include "herkulex_driver/srv/set_torque.hpp"
#include "herkulex_driver/srv/set_led.hpp"
#include "herkulex_driver/srv/get_position.hpp"
#include "herkulex_driver/srv/reboot.hpp"
#include "herkulex_driver/srv/clear_error.hpp"
#include "herkulex_driver/srv/set_id.hpp"
#include "herkulex_driver/srv/write_registry.hpp"
#include "herkulex_driver/srv/set_gain.hpp"
#include "herkulex_driver/srv/get_gain.hpp"

#include <memory>
#include <string>
#include <vector>

namespace herkulex_driver
{

class HerkulexNode : public rclcpp::Node
{
public:
  explicit HerkulexNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HerkulexNode() override;

private:
  // Timer callback for periodic status publishing
  void statusTimerCallback();

  // Service callbacks
  void onSetPosition(
    const std::shared_ptr<srv::SetPosition::Request> request,
    std::shared_ptr<srv::SetPosition::Response> response);

  void onSetAngle(
    const std::shared_ptr<srv::SetAngle::Request> request,
    std::shared_ptr<srv::SetAngle::Response> response);

  void onSetSpeed(
    const std::shared_ptr<srv::SetSpeed::Request> request,
    std::shared_ptr<srv::SetSpeed::Response> response);

  void onSetTorque(
    const std::shared_ptr<srv::SetTorque::Request> request,
    std::shared_ptr<srv::SetTorque::Response> response);

  void onSetLed(
    const std::shared_ptr<srv::SetLed::Request> request,
    std::shared_ptr<srv::SetLed::Response> response);

  void onGetPosition(
    const std::shared_ptr<srv::GetPosition::Request> request,
    std::shared_ptr<srv::GetPosition::Response> response);

  void onReboot(
    const std::shared_ptr<srv::Reboot::Request> request,
    std::shared_ptr<srv::Reboot::Response> response);

  void onClearError(
    const std::shared_ptr<srv::ClearError::Request> request,
    std::shared_ptr<srv::ClearError::Response> response);

  void onSetID(
    const std::shared_ptr<srv::SetID::Request> request,
    std::shared_ptr<srv::SetID::Response> response);

  void onWriteRegistry(
    const std::shared_ptr<srv::WriteRegistry::Request> request,
    std::shared_ptr<srv::WriteRegistry::Response> response);

  void onSetGain(
    const std::shared_ptr<srv::SetGain::Request> request,
    std::shared_ptr<srv::SetGain::Response> response);

  void onGetGain(
    const std::shared_ptr<srv::GetGain::Request> request,
    std::shared_ptr<srv::GetGain::Response> response);

  // HerkuleX serial communication
  std::unique_ptr<HerkulexSerial> serial_;

  // Parameters
  std::string serial_port_;
  int baud_rate_;
  std::string model_name_;
  std::vector<int64_t> servo_ids_;
  double status_rate_;
  bool auto_initialize_;

  // Publishers
  rclcpp::Publisher<msg::ServoStatusArray>::SharedPtr status_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Services
  rclcpp::Service<srv::SetPosition>::SharedPtr set_position_srv_;
  rclcpp::Service<srv::SetAngle>::SharedPtr set_angle_srv_;
  rclcpp::Service<srv::SetSpeed>::SharedPtr set_speed_srv_;
  rclcpp::Service<srv::SetTorque>::SharedPtr set_torque_srv_;
  rclcpp::Service<srv::SetLed>::SharedPtr set_led_srv_;
  rclcpp::Service<srv::GetPosition>::SharedPtr get_position_srv_;
  rclcpp::Service<srv::Reboot>::SharedPtr reboot_srv_;
  rclcpp::Service<srv::ClearError>::SharedPtr clear_error_srv_;
  rclcpp::Service<srv::SetID>::SharedPtr set_id_srv_;
  rclcpp::Service<srv::WriteRegistry>::SharedPtr write_registry_srv_;
  rclcpp::Service<srv::SetGain>::SharedPtr set_gain_srv_;
  rclcpp::Service<srv::GetGain>::SharedPtr get_gain_srv_;
};

}  // namespace herkulex_driver

#endif  // HERKULEX_DRIVER__HERKULEX_NODE_HPP_
