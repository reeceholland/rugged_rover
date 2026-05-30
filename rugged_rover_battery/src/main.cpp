#include "rugged_rover_battery/battery_voltage_monitor.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rugged_rover_battery::BatteryVoltageMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}