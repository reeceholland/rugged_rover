

#ifndef RUGGED_ROVER_HARDWARE_INTERFACES_SABERTOOTH_SYSTEM_INTERFACE_HPP
#define RUGGED_ROVER_HARDWARE_INTERFACES_SABERTOOTH_SYSTEM_INTERFACE_HPP

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace rugged_rover_hardware_interfaces::sabertooth
{
  class SabertoothSystemInterface : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(SabertoothSystemInterface)

    SabertoothSystemInterface() = default;
    ~SabertoothSystemInterface() override = default;

    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time& time,
                                         const rclcpp::Duration& period) override;

    hardware_interface::return_type write(const rclcpp::Time& time,
                                          const rclcpp::Duration& period) override;

    const std::vector<double>& get_hw_positions() const { return hw_positions_; }

    const std::vector<double>& get_hw_velocities() const { return hw_velocities_; }

  private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr battery_critical_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;

    void feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_commands_;
    std::vector<std::string> joint_names_;

    rclcpp::Logger logger_ = rclcpp::get_logger("SabertoothSystemInterface");

    sensor_msgs::msg::JointState last_feedback_;
    rclcpp::Time last_feedback_time_;
    bool has_feedback_ = false;
    double feedback_timeout_seconds_ = 0.25;
    bool use_reliable_command_qos_ = false;
    rclcpp::Time last_command_publish_time_;
    double command_publish_period_seconds_ = 0.02;
    std::mutex feedback_mutex_;
    std::atomic<bool> battery_allows_motion_ = true;

    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread executor_thread_;
    std::atomic<bool> executor_running_ = false;

    friend class SabertoothInterfaceTest;
  };

} // namespace rugged_rover_hardware_interfaces::sabertooth

#endif // RUGGED_ROVER_HARDWARE_INTERFACES_SABERTOOTH_SYSTEM_INTERFACE_HPP
