// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rrbot_cam/rrbot.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rrbot_cam {
hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get Params
  serial_device_ = info_.hardware_parameters["serial_device_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["communication_baud_rate"]);
  timeout_ms_ = std::stoi(info_.hardware_parameters["communication_timeout_ms"]);

  // Check Params
  hw_states_.resize(info_.joints.size(),
                    std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(),
                      std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    // RRBotSystemPositionOnly has exactly one state and command interface on
    // each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                   "Joint '%s' has %zu state interface. 1 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                   "Joint '%s' have %s state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
RRBotSystemPositionOnlyHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
              "Configuring ...please wait...");

  if (!arduino_comms_.connected()) {
    arduino_comms_.connect(serial_device_, baud_rate_, timeout_ms_);
  }

  arduino_comms_.reset_servos_position();

  // reset values always when configuring hardware
  // Initial values are 90 degrees for pan and 60 degrees for tilt
  hw_states_ = {90.0 * M_PI /180, 60.0 * M_PI /180};
  hw_commands_ = {90.0 * M_PI /180, 60.0 * M_PI /180}; 

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
              "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemPositionOnlyHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemPositionOnlyHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
              "Activating ...please wait...");

  if (!arduino_comms_.connected()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
              "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
RRBotSystemPositionOnlyHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
              "Deactivating ...please wait...");

  arduino_comms_.disconnect();
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
              "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  double pan_angle, tilt_angle;
  arduino_comms_.read_servos_position(pan_angle, tilt_angle);

  hw_states_[0] = pan_angle ;
  hw_states_[1] = tilt_angle;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  arduino_comms_.set_servos_position(hw_commands_[0], hw_commands_[1]);

  return hardware_interface::return_type::OK;
}

}  // namespace rrbot_cam

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rrbot_cam::RRBotSystemPositionOnlyHardware,
                       hardware_interface::SystemInterface)
