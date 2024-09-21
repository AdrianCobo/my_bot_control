#ifndef MY_BOT_CONTROL__DIFFBOT_SYSTEM_HPP_
#define MY_BOT_CONTROL__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "my_bot_control/arduino_comms.hpp" // arduino communications by serial connection
#include "my_bot_control/wheel.hpp" // add arduino communications by serial connection

namespace my_bot_control
{
class MyPlugin : public hardware_interface::SystemInterface
{

// store all the info from description/ros2_control/diffbot.ros2_control.xacro
struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev_left_wheel = 0;
  int enc_counts_per_rev_right_wheel = 0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyPlugin);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  ArduinoComms comms_;
  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;
};

}  // namespace my_bot_control

#endif  // MY_BOT_CONTROL__DIFFBOT_SYSTEM_HPP_
