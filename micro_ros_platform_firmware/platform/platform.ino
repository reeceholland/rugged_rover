#include <Arduino.h>
#include "ros_interface.hpp"
#include "encoder_utils.hpp"
#include "motor_control.hpp"
#include "msg_utils.hpp"
#include "config.hpp"

bool SERIAL_DEBUG = !USE_ROS;

namespace {

String serial_line;

void print_serial_help() {
  Serial.println();
  Serial.println("Rugged Rover motor bench mode");
  Serial.println("Send: <left_rad_s> <right_rad_s>");
  Serial.println("Example: 1.5 1.5");
  Serial.println("Send: s");
  Serial.println("Stops both motors.");
  Serial.println();
}

void handle_serial_command(const String &line) {
  String trimmed = line;
  trimmed.trim();

  if (trimmed.length() == 0) {
    return;
  }

  if (trimmed == "s" || trimmed == "S" || trimmed == "stop") {
    stop_motors();
    Serial.println("Setpoints: left=0.000 rad/s, right=0.000 rad/s");
    return;
  }

  char buffer[64];
  trimmed.toCharArray(buffer, sizeof(buffer));

  char *end = nullptr;
  const float left = strtof(buffer, &end);

  if (end == buffer) {
    Serial.println("Could not parse left setpoint.");
    print_serial_help();
    return;
  }

  while (*end == ' ' || *end == ',' || *end == '\t') {
    ++end;
  }

  char *right_end = nullptr;
  const float right = strtof(end, &right_end);

  if (right_end == end) {
    Serial.println("Could not parse right setpoint.");
    print_serial_help();
    return;
  }

  front_left_velocity_setpoint = left;
  front_right_velocity_setpoint = right;

  Serial.print("Setpoints: left=");
  Serial.print(front_left_velocity_setpoint, 3);
  Serial.print(" rad/s, right=");
  Serial.print(front_right_velocity_setpoint, 3);
  Serial.println(" rad/s");
}

void read_serial_commands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\n' || c == '\r') {
      handle_serial_command(serial_line);
      serial_line = "";
    } else if (serial_line.length() < 63) {
      serial_line += c;
    }
  }
}

}  // namespace

void setup() {
  Serial2.begin(9600);

  if (USE_ROS) {
    ros_setup();
  } else {
    Serial.begin(115200);
    delay(1000);
    print_serial_help();
  }

  setup_pid();
}

void loop() {

  unsigned long now = millis();
  if (now - lastEncoderSampleTime >= 50) {
    
    sample_encoders();
    if (USE_ROS) {
      publish_joint_state_message();
    }
    update_motors();
  }

  if (USE_ROS) {
    spin_ros_executor();
  } else {
    read_serial_commands();
  }
}
