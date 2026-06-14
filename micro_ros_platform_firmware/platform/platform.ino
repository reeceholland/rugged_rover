#include "battery_monitor.hpp"
#include "battery_safety.hpp"
#include "config.hpp"
#include "encoder_utils.hpp"
#include "motor_control.hpp"
#include "msg_utils.hpp"
#include "ros_interface.hpp"
#include "status_led.hpp"
#include <Arduino.h>

bool SERIAL_DEBUG = !USE_ROS;

namespace
{

  String serial_line;

  void print_serial_help()
  {
    Serial.println();
    Serial.println("Rugged Rover motor bench mode");
    Serial.println("Send: <left_rad_s> <right_rad_s>");
    Serial.println("Example: 1.5 1.5");
    Serial.println("Send: s");
    Serial.println("Stops both motors.");
    Serial.println();
  }

  void handle_serial_command(const String& line)
  {
    String trimmed = line;
    trimmed.trim();

    if (trimmed.length() == 0)
    {
      return;
    }

    if (trimmed == "s" || trimmed == "S" || trimmed == "stop")
    {
      stop_motors();
      Serial.println("Setpoints: left=0.000 rad/s, right=0.000 rad/s");
      return;
    }

    char buffer[64];
    trimmed.toCharArray(buffer, sizeof(buffer));

    char* end = nullptr;
    const float left = strtof(buffer, &end);

    if (end == buffer)
    {
      Serial.println("Could not parse left setpoint.");
      print_serial_help();
      return;
    }

    while (*end == ' ' || *end == ',' || *end == '\t')
    {
      ++end;
    }

    char* right_end = nullptr;
    const float right = strtof(end, &right_end);

    if (right_end == end)
    {
      Serial.println("Could not parse right setpoint.");
      print_serial_help();
      return;
    }

    front_left_velocity_setpoint = left;
    front_right_velocity_setpoint = right;
    mark_motor_command_received();

    Serial.print("Setpoints: left=");
    Serial.print(front_left_velocity_setpoint, 3);
    Serial.print(" rad/s, right=");
    Serial.print(front_right_velocity_setpoint, 3);
    Serial.println(" rad/s");
  }

  void read_serial_commands()
  {
    while (Serial.available() > 0)
    {
      const char c = static_cast<char>(Serial.read());

      if (c == '\n' || c == '\r')
      {
        handle_serial_command(serial_line);
        serial_line = "";
      }
      else if (serial_line.length() < 63)
      {
        serial_line += c;
      }
    }
  }

} // namespace

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600);

  if (USE_ROS)
  {
    ros_setup();
  }
  else
  {
    delay(1000);
    print_serial_help();
  }

  setup_pid();
  setup_battery_monitor();
  battery_safety_setup();
  status_led_setup();
}

void loop()
{

  unsigned long now = millis();
  if (now - lastEncoderSampleTime >= 50)
  {

    sample_encoders();
    if (USE_ROS && ros_is_connected())
    {
      publish_joint_state_message();
      publish_battery_voltage_message();
      publish_debug_message();
    }
    update_motors();
  }

  if (USE_ROS && ROS_SERIAL_STATUS_DEBUG)
  {
    static unsigned long last_debug_ms = 0;
    if (now - last_debug_ms >= 1000)
    {
      last_debug_ms = now;
      Serial.print("ros=");
      Serial.print(ros_is_connected() ? "connected" : "waiting");
      Serial.print(" battery_critical=");
      Serial.print(battery_is_critical() ? "true" : "false");
      Serial.print(" last_cmd_age_ms=");
      Serial.print(last_motor_command_ms == 0 ? -1 : static_cast<long>(now - last_motor_command_ms));
      Serial.print(" setpoints FL=");
      Serial.print(front_left_velocity_setpoint, 3);
      Serial.print(" FR=");
      Serial.println(front_right_velocity_setpoint, 3);
    }
  }

  if (USE_ROS)
  {
    ros_update();
  }
  else
  {
    read_serial_commands();
  }

  battery_safety_update(read_battery_voltage());

  if (battery_is_critical())
  {
    status_led_update(LedStatus::BatteryCritical);
  }
  else if (ros_is_connected())
  {
    status_led_update(LedStatus::RosConnected);
  }
  else
  {
    status_led_update(LedStatus::Normal);
  }
}
