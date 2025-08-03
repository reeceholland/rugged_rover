#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

/**
 * @brief Main function for the test joint state publisher node.
 *
 * This node publishes a JointState message with wheel velocities
 * to the "platform/motors/cmd" topic at a rate of 1 Hz.
 *
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return int Exit status.
 */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("test_joint_state_publisher");

  // Default wheel velocities
  double left_velocity = 1.5;
  double right_velocity = 1.5;

  // Parse command-line arguments if provided
  if (argc >= 3)
  {
    try {
      left_velocity = std::stod(argv[1]);
      right_velocity = std::stod(argv[2]);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node->get_logger(), "Invalid arguments: %s", e.what());
      RCLCPP_INFO(node->get_logger(), "Usage: ros2 run rugged_rover_test send_joint_command <left_velocity> <right_velocity>");
      rclcpp::shutdown();
      return 1;
    }
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "No command line arguments provided, using defaults.");
    RCLCPP_INFO(node->get_logger(), "Default velocities: left=%.2f, right=%.2f", left_velocity, right_velocity);
    RCLCPP_INFO(node->get_logger(), "Usage: ros2 run rugged_rover_test send_joint_command <left_velocity> <right_velocity>");
  }

  // Create the publisher
  auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("platform/motors/cmd", rclcpp::SensorDataQoS());
  rclcpp::WallRate loop_rate(1); // 1 Hz

  while (rclcpp::ok())
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node->now();
    msg.name = {"left_front_wheel_joint", "right_front_wheel_joint"};
    msg.velocity = {left_velocity, right_velocity};

    publisher->publish(msg);

    RCLCPP_INFO(node->get_logger(), "Publishing velocities -> left: %.2f rad/s, right: %.2f rad/s",
                left_velocity, right_velocity);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}