#include "my_bot_control/my_diffbot_system.hpp"
#include "my_bot_control/arduino_comms.hpp" // arduino communications by serial connection
#include "my_bot_control/wheel.hpp" // wheel relevant info and calculus

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_bot_control
{
hardware_interface::CallbackReturn MyPlugin::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // the info param has all the configuration params from description/ros2_control/diffbot.ros2_control.xacro
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev_left_wheel = std::stoi(info_.hardware_parameters["enc_counts_per_rev_left_wheel"]);
  cfg_.enc_counts_per_rev_right_wheel = std::stoi(info_.hardware_parameters["enc_counts_per_rev_right_wheel"]);

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev_left_wheel);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev_right_wheel);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint. JUST CHECKING NO ERRORS
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MyPlugin"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MyPlugin"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MyPlugin"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MyPlugin"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MyPlugin"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// link our class variables to the plugin state interfaces variables. See export_state_interfaces.png
// so everytime we update the position and velocity of this wheels, the rest of the ros2 controll will know
std::vector<hardware_interface::StateInterface> MyPlugin::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));  
  
  return state_interfaces;
}

// link our class variables to the plugin command interfaces variables. See export_state_interfaces.png
// so everytime the rest of the ros2_control sets the velocity of this wheels, this cmd variable value will change
std::vector<hardware_interface::CommandInterface> MyPlugin::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn MyPlugin::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MyPlugin"), "Activating ...please wait...");
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("MyPlugin"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyPlugin::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MyPlugin"), "Deactivating ...please wait...");
  comms_.disconnect();
  RCLCPP_INFO(rclcpp::get_logger("MyPlugin"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MyPlugin::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyPlugin::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // baud = number of times a value changes per seconds. baud = counts
  //         counts/loop      =   rads/sec          rads/count               baud / sec
  int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
  int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);

  return hardware_interface::return_type::OK;
}

}  // namespace my_bot_control
// the first argument is the name that correspond to the plugin exported at my_bot_control.xml
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  my_bot_control::MyPlugin, hardware_interface::SystemInterface)
