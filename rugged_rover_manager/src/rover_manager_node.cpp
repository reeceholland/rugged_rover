#include "rugged_rover_manager/rover_manager_node.hpp"

#include <cstdlib>
#include <sstream>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include <gpiod.hpp>

namespace rugged_rover_manager
{

namespace
{
  std::string bool_arg(bool value)
  {
    return value ? "true" : "false";
  }
} // namespace

RoverManagerNode::RoverManagerNode(const rclcpp::NodeOptions & options)
: Node("rover_manager_node", options)
{
  declare_parameters();
  load_parameters();
  setup_mode_switch_gpio();
  previous_debounced_switch_active_ = debounced_switch_active_;
  first_rising_edge_time_ = now();

  battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/battery/voltage", 10, std::bind(&RoverManagerNode::battery_callback, this, std::placeholders::_1));

  platform_debug_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/platform/debug", 10, std::bind(&RoverManagerNode::platform_debug_callback, this, std::placeholders::_1));

  state_pub_ = this->create_publisher<std_msgs::msg::String>("/rover/state", 10);
  motors_enabled_pub_ = this->create_publisher<std_msgs::msg::Bool>("/rover/motors_enabled", 10);
  events_pub_ = this->create_publisher<std_msgs::msg::String>("/rover/events", 10);

  last_platform_debug_time_ = this->now();
  last_switch_change_time_ = this->now();

  control_timer_ = create_wall_timer(
  std::chrono::duration<double>(control_period_sec_),
  std::bind(&RoverManagerNode::control_loop, this));

  transition_to(RoverState::Idle, "manager started");
}

RoverManagerNode::~RoverManagerNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Rover Manager Node");
  stop_active_launch();
  publish_motor_enable(false);
}

void RoverManagerNode::declare_parameters()
{
  declare_parameter("battery_warning_voltage", battery_warning_voltage_);
  declare_parameter("battery_critical_voltage", battery_critical_voltage_);
  declare_parameter("platform_timeout_sec", platform_timeout_sec_);
  declare_parameter("control_period_sec", control_period_sec_);
  declare_parameter("double_toggle_window_sec", double_toggle_window_sec_);

  declare_parameter("mode_switch_gpio", mode_switch_gpio_);

  declare_parameter("teleop_launch_package", teleop_launch_package_);
  declare_parameter("teleop_launch_file", teleop_launch_file_);

  declare_parameter("autonomous_launch_package", autonomous_launch_package_);
  declare_parameter("autonomous_launch_file", autonomous_launch_file_);

  declare_parameter("autonomous_use_ekf", autonomous_use_ekf_);
  declare_parameter("autonomous_use_slam", autonomous_use_slam_);
  declare_parameter("autonomous_use_rplidar", autonomous_use_rplidar_);

  declare_parameter("mode_switch_gpio_chip", mode_switch_gpio_chip_);
  declare_parameter("mode_switch_gpio_line", mode_switch_gpio_line_);
  declare_parameter("mode_switch_active_high", mode_switch_active_high_);
  declare_parameter("mode_switch_debounce_sec", mode_switch_debounce_sec_);
}

void RoverManagerNode::load_parameters()
{
  get_parameter("battery_warning_voltage", battery_warning_voltage_);
  get_parameter("battery_critical_voltage", battery_critical_voltage_);
  get_parameter("platform_timeout_sec", platform_timeout_sec_);
  get_parameter("control_period_sec", control_period_sec_);
  get_parameter("double_toggle_window_sec", double_toggle_window_sec_);

  get_parameter("mode_switch_gpio", mode_switch_gpio_);

  get_parameter("teleop_launch_package", teleop_launch_package_);
  get_parameter("teleop_launch_file", teleop_launch_file_);

  get_parameter("autonomous_launch_package", autonomous_launch_package_);
  get_parameter("autonomous_launch_file", autonomous_launch_file_);

  get_parameter("autonomous_use_ekf", autonomous_use_ekf_);
  get_parameter("autonomous_use_slam", autonomous_use_slam_);
  get_parameter("autonomous_use_rplidar", autonomous_use_rplidar_);

  get_parameter("mode_switch_gpio_chip", mode_switch_gpio_chip_);
  get_parameter("mode_switch_gpio_line", mode_switch_gpio_line_);
  get_parameter("mode_switch_active_high", mode_switch_active_high_);
  get_parameter("mode_switch_debounce_sec", mode_switch_debounce_sec_);
}

void RoverManagerNode::battery_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  latest_battery_voltage_ = msg->data;
}

void RoverManagerNode::platform_debug_callback(const std_msgs::msg::String::SharedPtr)
{
  last_platform_debug_time_ = now();
}

void RoverManagerNode::control_loop()
{
  if (battery_is_critical()) {
    transition_to(RoverState::LowBattery, "battery critical");
    publish_motor_enable(false);
    stop_active_launch();
    return;
  }

  const ModeRequest request = read_mode_request();

  if (request == ModeRequest::Stop) {
    handle_mode_request(request);
    publish_state();
    return;
  }

  if (platform_is_stale()) {
    transition_to(RoverState::Fault, "platform debug stale");
    publish_motor_enable(false);
    stop_active_launch();
    return;
  }

  handle_mode_request(request);

  publish_state();
}

void RoverManagerNode::setup_mode_switch_gpio()
{
  gpiod::chip chip(mode_switch_gpio_chip_, gpiod::chip::OPEN_BY_NAME);

  mode_switch_line_.emplace(
    chip.get_line(static_cast<unsigned int>(mode_switch_gpio_line_)));

  const auto bias_flags =
    mode_switch_active_high_ ?
    gpiod::line_request::FLAG_BIAS_PULL_DOWN :
    gpiod::line_request::FLAG_BIAS_PULL_UP;

  mode_switch_line_->request(
    {
      "rugged_rover_manager",
      gpiod::line_request::DIRECTION_INPUT,
      bias_flags,
    });

  last_raw_switch_active_ = read_mode_switch_gpio();
  debounced_switch_active_ = last_raw_switch_active_;
  last_raw_switch_change_time_ = now();
}

bool RoverManagerNode::read_mode_switch_gpio() const
{
  if (!mode_switch_line_.has_value()) {
    return debounced_switch_active_;
  }

  const int value = mode_switch_line_->get_value();
  const bool raw_high = value != 0;

  return mode_switch_active_high_ ? raw_high : !raw_high;
}

ModeRequest RoverManagerNode::read_mode_request()
{
  const bool raw_active = read_mode_switch_gpio();
  const rclcpp::Time now_time = now();

  if (raw_active != last_raw_switch_active_) {
    last_raw_switch_active_ = raw_active;
    last_raw_switch_change_time_ = now_time;
  }

  const double stable_for_sec = (now_time - last_raw_switch_change_time_).seconds();

  if (stable_for_sec >= mode_switch_debounce_sec_) {
    debounced_switch_active_ = raw_active;
  }

  const bool rising_edge =
    !previous_debounced_switch_active_ && debounced_switch_active_;

  const bool falling_edge =
    previous_debounced_switch_active_ && !debounced_switch_active_;

  previous_debounced_switch_active_ = debounced_switch_active_;

  const bool mode_request_pending = rising_edge_count_ == 1;
  const double since_first_edge_sec = (now_time - first_rising_edge_time_).seconds();

  if (falling_edge) {
    if (mode_request_pending && since_first_edge_sec <= double_toggle_window_sec_) {
      return ModeRequest::NoChange;
    }

    rising_edge_count_ = 0;
    return ModeRequest::Stop;
  }

  if (rising_edge) {
    if (!mode_request_pending || since_first_edge_sec > double_toggle_window_sec_) {
      rising_edge_count_ = 1;
      first_rising_edge_time_ = now_time;
      return ModeRequest::NoChange;
    }

    rising_edge_count_ = 0;
    return ModeRequest::Autonomous;
  }

  if (mode_request_pending && since_first_edge_sec > double_toggle_window_sec_) {
    rising_edge_count_ = 0;
    return ModeRequest::Teleop;
  }

  return ModeRequest::NoChange;
}

void RoverManagerNode::handle_mode_request(ModeRequest request)
{
  if (request == ModeRequest::NoChange) {
    return;
  }

  if (request != last_mode_request_) {
    last_switch_change_time_ = now();
    last_mode_request_ = request;
  }

  switch (request) {
    case ModeRequest::NoChange:
      break;

    case ModeRequest::Stop:
      stop_active_launch();
      publish_motor_enable(false);
      transition_to(RoverState::Idle, "switch low");
      break;
    case ModeRequest::Teleop:
      if (state_ != RoverState::Teleop) {
        stop_active_launch();
        start_teleop();
        publish_motor_enable(true);
        transition_to(RoverState::Teleop, "mode switch teleop");
      }
      break;

    case ModeRequest::Autonomous:
      if (state_ != RoverState::Autonomous) {
        stop_active_launch();
        start_autonomous();
        publish_motor_enable(true);
        transition_to(RoverState::Autonomous, "mode switch autonomous");
      }
      break;

    case ModeRequest::Shutdown:
      stop_active_launch();
      publish_motor_enable(false);
      transition_to(RoverState::ShutdownRequested, "shutdown requested");
      break;
  }
}

void RoverManagerNode::transition_to(RoverState next_state, const std::string & reason)
{
  if (state_ == next_state) {
    return;
  }

  const auto previous = state_;
  state_ = next_state;

  std::ostringstream event;
  event << "state " << to_string(previous) << " -> " << to_string(next_state)
        << ": " << reason;

  RCLCPP_INFO(get_logger(), "%s", event.str().c_str());
  publish_event(event.str());
  publish_state();
}

void RoverManagerNode::publish_state()
{
  std_msgs::msg::String msg;
  msg.data = to_string(state_);
  state_pub_->publish(msg);
}

void RoverManagerNode::publish_motor_enable(bool enabled)
{
  std_msgs::msg::Bool msg;
  msg.data = enabled;
  motors_enabled_pub_->publish(msg);
}

void RoverManagerNode::publish_event(const std::string & event)
{
  std_msgs::msg::String msg;
  msg.data = event;
  events_pub_->publish(msg);
}

bool RoverManagerNode::battery_is_warning() const
{
  return latest_battery_voltage_.has_value() &&
         latest_battery_voltage_.value() <= battery_warning_voltage_;
}

bool RoverManagerNode::battery_is_critical() const
{
  return latest_battery_voltage_.has_value() &&
         latest_battery_voltage_.value() <= battery_critical_voltage_;
}

bool RoverManagerNode::platform_is_stale() const
{
  if (platform_timeout_sec_ <= 0.0) {
    return false;
  }

  if (state_ != RoverState::Teleop && state_ != RoverState::Autonomous) {
    return false;
  }

  const auto age = now() - last_platform_debug_time_;
  return age.seconds() > platform_timeout_sec_;
}

void RoverManagerNode::start_teleop()
{
  const std::string command =
    "ros2 launch " + teleop_launch_package_ + " " + teleop_launch_file_;

  RCLCPP_INFO(get_logger(), "starting teleop: %s", command.c_str());
  
  start_launch_process("teleop", command);
}

void RoverManagerNode::start_autonomous()
{
  const std::string command =
    "ros2 launch " + autonomous_launch_package_ + " " + autonomous_launch_file_ +
    " use_ekf:=" + bool_arg(autonomous_use_ekf_) +
    " use_slam:=" + bool_arg(autonomous_use_slam_) +
    " use_rplidar:=" + bool_arg(autonomous_use_rplidar_);

  RCLCPP_INFO(get_logger(), "starting autonomous: %s", command.c_str());
  start_launch_process("autonomous", command);
}

std::string RoverManagerNode::to_string(RoverState state)
{
  switch (state) {
    case RoverState::Booting:
      return "booting";
    case RoverState::Teleop:
      return "teleop";
    case RoverState::Autonomous:
      return "autonomous";
    case RoverState::LowBattery:
      return "low_battery";
    case RoverState::Fault:
      return "fault";
    case RoverState::EStopped:
      return "e_stopped";
    case RoverState::Idle:
      return "idle";
    case RoverState::ShutdownRequested:
      return "shutdown_requested";
  }

  return "unknown";
}

void RoverManagerNode::start_launch_process(const std::string & name, const std::string & command)
{
  stop_active_launch();

  const pid_t pid = fork();

  if(pid < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to fork for launch process '%s': %s", name.c_str(), std::strerror(errno));
    return;
  }

  if(pid == 0)
  {
    // Child: create a new process group so the manager can stop the whole launch tree.
    setpgid(0, 0);

    const std::string exec_command =  "exec " + command;

    execl("/bin/bash", "bash", "-lc", exec_command.c_str(), nullptr);

    // Only reached if execl fails.
    _exit(127);
  }

  // Parent: track the child PID.
  setpgid(pid, pid);

  active_launch_pid_ = pid;
  active_launch_name_ = name;

  RCLCPP_INFO(get_logger(), "Started %s launch process pid=%d: %s", name.c_str(), pid, command.c_str());
}

bool RoverManagerNode::process_group_exists(pid_t pgid) const
{
  if (kill(-pgid, 0) == 0) {
    return true;
  }

  return errno != ESRCH;
}

bool RoverManagerNode::wait_for_process_group_exit(pid_t pgid, std::chrono::milliseconds timeout)
{
  const auto start_time = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start_time < timeout) {
    int status = 0;
    while (waitpid(-pgid, &status, WNOHANG) > 0) {
    }

    if (!process_group_exists(pgid)) {
      return true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return !process_group_exists(pgid);
}

void RoverManagerNode::stop_active_launch()
{
  if (!active_launch_pid_.has_value()) {
    return;
  }

  const pid_t pid = active_launch_pid_.value();

  RCLCPP_WARN(
    get_logger(),
    "stopping %s launch process group pid=%d",
    active_launch_name_.c_str(),
    pid);

  // Negative PID means signal the whole process group.
  if (kill(-pid, SIGINT) != 0 && errno != ESRCH) {
    RCLCPP_WARN(
      get_logger(),
      "failed to send SIGINT to process group %d: %s",
      pid,
      std::strerror(errno));
  }

  if (!wait_for_process_group_exit(pid, std::chrono::seconds(5))) {
    RCLCPP_WARN(
      get_logger(),
      "launch process group %d did not exit after SIGINT; sending SIGTERM",
      pid);

    if (kill(-pid, SIGTERM) != 0 && errno != ESRCH) {
      RCLCPP_WARN(
        get_logger(),
        "failed to send SIGTERM to process group %d: %s",
        pid,
        std::strerror(errno));
    }

    if (!wait_for_process_group_exit(pid, std::chrono::seconds(2))) {
      RCLCPP_WARN(
        get_logger(),
        "launch process group %d did not exit after SIGTERM; sending SIGKILL",
        pid);

      if (kill(-pid, SIGKILL) != 0 && errno != ESRCH) {
        RCLCPP_WARN(
          get_logger(),
          "failed to send SIGKILL to process group %d: %s",
          pid,
          std::strerror(errno));
      }

      wait_for_process_group_exit(pid, std::chrono::seconds(1));
    }
  }

  active_launch_pid_.reset();
  active_launch_name_.clear();
}

}  // namespace rugged_rover_manager

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(rugged_rover_manager::RoverManagerNode)


