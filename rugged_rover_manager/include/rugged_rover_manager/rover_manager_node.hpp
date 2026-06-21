#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <gpiod.hpp>

#include "rugged_rover_manager/rover_state.hpp"

namespace rugged_rover_manager
{

enum class ModeRequest
{
  Teleop,
  Autonomous,
  Shutdown,
};

class RoverManagerNode : public rclcpp::Node
{
public:
  explicit RoverManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RoverManagerNode() override;

private:
  void declare_parameters();
  void load_parameters();

  void battery_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void platform_debug_callback(const std_msgs::msg::String::SharedPtr msg);

  void control_loop();

  ModeRequest read_mode_request();
  void handle_mode_request(ModeRequest request);

  void transition_to(RoverState next_state, const std::string & reason);
  void publish_state();
  void publish_motor_enable(bool enabled);
  void publish_event(const std::string & event);

  bool battery_is_warning() const;
  bool battery_is_critical() const;
  bool platform_is_stale() const;

  void start_teleop();
  void start_autonomous();
  void stop_active_launch();

  void setup_mode_switch_gpio();
  bool read_mode_switch_gpio() const;

  static std::string to_string(RoverState state);

  // Parameters
  double battery_warning_voltage_{11.4};
  double battery_critical_voltage_{10.8};
  double platform_timeout_sec_{2.0};
  double control_period_sec_{0.1};
  double double_toggle_window_sec_{2.0};

  int mode_switch_gpio_{24};

  std::string teleop_launch_package_{"rugged_rover_bringup"};
  std::string teleop_launch_file_{"teleop.launch.py"};

  std::string autonomous_launch_package_{"rugged_rover_bringup"};
  std::string autonomous_launch_file_{"bringup.launch.py"};

  bool autonomous_use_ekf_{true};
  bool autonomous_use_slam_{true};
  bool autonomous_use_rplidar_{true};

  // State
  RoverState state_{RoverState::Booting};
  std::optional<float> latest_battery_voltage_;
  rclcpp::Time last_platform_debug_time_;
  rclcpp::Time last_switch_change_time_;
  ModeRequest last_mode_request_{ModeRequest::Teleop};

  // Process tracking. This can later become a small LaunchProcess wrapper.
  std::optional<int> active_launch_pid_;
  std::string active_launch_name_;

  // GPIO handling
  std::string mode_switch_gpio_chip_{"gpiochip4"};
  int mode_switch_gpio_line_{24};
  bool mode_switch_active_high_{true};
  double mode_switch_debounce_sec_{0.15};

  bool last_raw_switch_active_{false};
  bool debounced_switch_active_{false};
  rclcpp::Time last_raw_switch_change_time_;

  std::optional<gpiod::line> mode_switch_line_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr platform_debug_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motors_enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace rugged_rover_manager