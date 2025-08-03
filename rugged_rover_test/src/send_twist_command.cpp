#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("twist_command_sender");

  rclcpp::WallRate loop_rate(1); // 10 Hz

  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
      "micro_ros_arduino_twist_subscriber", rclcpp::QoS(10));
  while (rclcpp::ok())
  {
    geometry_msgs::msg::Twist msg;

    // Example values – you can adjust these as needed
    msg.linear.x = 0.5; // m/s
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.1; // rad/s

    RCLCPP_INFO(node->get_logger(), "Publishing Twist command...");
    publisher->publish(msg);

    // Spin the node to process callbacks
    rclcpp::spin_some(node);

    // Sleep to maintain the loop rate
    // This will block until the next iteration is due
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}