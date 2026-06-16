// Copyright 2026 Reece Holland
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RUGGED_ROVER_BATTERY__BATTERY_VOLTAGE_MONITOR_HPP
#define RUGGED_ROVER_BATTERY__BATTERY_VOLTAGE_MONITOR_HPP

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

namespace rugged_rover_battery
{

class BatteryVoltageMonitor : public rclcpp::Node
{
public:
  BatteryVoltageMonitor();

private:
  void batteryVoltageCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void publishState(float voltage);
  void publishDiagnostics(float voltage, bool is_low, bool is_critical);
  void timerCallback();

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_voltage_sub_;
  rclcpp::TimerBase::SharedPtr stale_timer_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr battery_low_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr battery_critical_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  double low_voltage_;
  double critical_voltage_;
  double stale_timeout_;

  rclcpp::Time last_voltage_time_;
  bool has_voltage_;
};
} // namespace rugged_rover_battery
#endif // RUGGED_ROVER_BATTERY__BATTERY_VOLTAGE_MONITOR_HPP
