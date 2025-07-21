#include "rclcpp/rclcpp.hpp"
#include "sabertooth_motor_driver/sabertooth_driver_node.hpp"

// Main function to initialize the ROS 2 node and start the executor
int main(int argc, char* argv[])
{
  // Initialize the ROS 2 context with command line arguments
  rclcpp::init(argc, argv);

  // Create a node instance of SabertoothDriverNode
  auto node =
      std::make_shared<sabertooth_motor_driver::SabertoothDriverNode>("sabertooth_driver_node");

  // Create a single-threaded executor to manage the node
  rclcpp::executors::SingleThreadedExecutor executor;

  // Add the node to the executor
  executor.add_node(node);

  // Spin the executor to process callbacks and handle incoming messages
  executor.spin();

  // Shutdown the ROS 2 context after spinning is complete
  rclcpp::shutdown();

  // Return success
  return 0;
}