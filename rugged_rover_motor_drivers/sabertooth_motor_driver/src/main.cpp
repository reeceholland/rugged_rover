#include "rclcpp/rclcpp.hpp"
#include "sabertooth_motor_driver/sabertooth_driver_node.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<sabertooth_motor_driver::SabertoothDriverNode>("sabertooth_driver_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}