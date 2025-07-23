

#ifndef RUGGED_ROVER_HARDWARE_INTERFACES_SABERTOOTH_SYSTEM_INTERFACE_HPP
#define RUGGED_ROVER_HARDWARE_INTERFACES_SABERTOOTH_SYSTEM_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rugged_rover_interfaces/msg/rover_feedback.hpp"

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

    rclcpp::Subscription<rugged_rover_interfaces::msg::RoverFeedback>::SharedPtr feedback_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;

    void feedbackCallback(const rugged_rover_interfaces::msg::RoverFeedback::SharedPtr msg);

    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_commands_;
    std::vector<std::string> joint_names_;

    rclcpp::Logger logger_ = rclcpp::get_logger("SabertoothSystemInterface");

    double battery_voltage_mv_ = 0.0;

    rugged_rover_interfaces::msg::RoverFeedback last_feedback_;
    std::mutex feedback_mutex_;

    friend class SabertoothInterfaceTest;
  };

} // namespace rugged_rover_hardware_interfaces::sabertooth

#endif // RUGGED_ROVER_HARDWARE_INTERFACES_SABERTOOTH_SYSTEM_INTERFACE_HPP
