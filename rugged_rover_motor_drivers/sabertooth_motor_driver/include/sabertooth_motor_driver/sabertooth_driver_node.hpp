
#ifndef SABERTOOTH_MOTOR_DRIVER__SABERTOOTH_DRIVER_NODE_H
#define SABERTOOTH_MOTOR_DRIVER__SABERTOOTH_DRIVER_NODE_H

#include "sabertooth_motor_driver/driver.hpp"

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace sabertooth_motor_driver
{

  class SabertoothDriverNode : public rclcpp::Node
  {
  public:
    explicit SabertoothDriverNode(const std::string& node_name);

    void cmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void feedbackTimerCallback();

  private:
    rclcpp::TimerBase::SharedPtr feedback_timer_;
    double feedback_rate_hz_ = 5.0;
    std::shared_ptr<Driver> driver_;
    std::string serial_port_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_pub_;
  };

} // namespace sabertooth_motor_driver

#endif // SABERTOOTH_MOTOR_DRIVER__SABERTOOTH_DRIVER_NODE_H