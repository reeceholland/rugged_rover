#include "rugged_rover_battery/battery_voltage_monitor.hpp"

namespace rugged_rover_battery
{
  BatteryVoltageMonitor::BatteryVoltageMonitor() : Node("battery_voltage_monitor")
  {
    // Declare parameters with default values
    this->declare_parameter<double>("low_voltage_threshold", 11.0);
    this->declare_parameter<double>("critical_voltage_threshold", 10.5);
    this->declare_parameter<double>("stale_timeout", 5.0);

    // Get parameter values
    this->get_parameter("low_voltage_threshold", low_voltage_);
    this->get_parameter("critical_voltage_threshold", critical_voltage_);
    this->get_parameter("stale_timeout", stale_timeout_);

    // Initialize publishers
    battery_low_pub_ = this->create_publisher<std_msgs::msg::Bool>("battery_low", 10);
    battery_critical_pub_ =
        this->create_publisher<std_msgs::msg::Bool>("platform/battery/is_critical", 10);
    diagnostics_pub_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

    auto timer_period = std::chrono::milliseconds(500);

    stale_timer_ = this->create_wall_timer(
        timer_period, std::bind(&BatteryVoltageMonitor::timerCallback, this));

    // Initialize subscriber. Teensy firmware and Unity publish the raw voltage here.
    battery_voltage_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "battery/voltage", 10,
        std::bind(&BatteryVoltageMonitor::batteryVoltageCallback, this, std::placeholders::_1));

    has_voltage_ = false;
  }

  void BatteryVoltageMonitor::batteryVoltageCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float voltage = msg->data;
    last_voltage_time_ = this->now();
    has_voltage_ = true;

    publishState(voltage);
    publishDiagnostics(voltage, voltage < low_voltage_, voltage < critical_voltage_);
  }

  void BatteryVoltageMonitor::publishState(float voltage)
  {
    std_msgs::msg::Bool low_msg;
    low_msg.data = voltage < low_voltage_;
    battery_low_pub_->publish(low_msg);

    std_msgs::msg::Bool critical_msg;
    critical_msg.data = voltage < critical_voltage_;
    battery_critical_pub_->publish(critical_msg);
  }

  void BatteryVoltageMonitor::publishDiagnostics(float voltage, bool is_low, bool is_critical)
  {
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diagnostic_msgs::msg::DiagnosticStatus status;

    diag_array.header.stamp = now();

    status.name = "rugged_rover_battery: Battery Voltage";
    status.hardware_id = "battery_monitor";

    diagnostic_msgs::msg::KeyValue voltage_value;
    voltage_value.key = "voltage_v";
    voltage_value.value = std::to_string(voltage);
    status.values.push_back(voltage_value);

    if (is_critical)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Battery voltage is critical";
    }
    else if (is_low)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Battery voltage is low";
    }
    else
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Battery voltage is normal";
    }

    diag_array.status.push_back(status);
    diagnostics_pub_->publish(diag_array);
  }

  void BatteryVoltageMonitor::timerCallback()
  {
    if (!has_voltage_)
    {
      return; // No voltage received yet
    }

    auto now = this->now();
    if ((now - last_voltage_time_).seconds() > stale_timeout_)
    {
      RCLCPP_WARN(this->get_logger(), "Battery voltage data is stale!");
      publishDiagnostics(0.0, true, true); // Publish critical status for stale data
    }
  }
} // namespace rugged_rover_battery