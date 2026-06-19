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

#include "motor_control.hpp"
#include "battery_safety.hpp"
#include "config.hpp"

namespace
{

constexpr int LEFT_FEEDFORWARD_COMMAND = 22;
constexpr int RIGHT_FEEDFORWARD_COMMAND = 25;
constexpr float SETPOINT_DEADBAND_RAD_S = 0.05;
constexpr int DEBUG_MAX_MOTOR_COMMAND = 60;

int command_with_feedforward(float pid_output, float setpoint, int feedforward_command)
{
  if (abs(setpoint) <= SETPOINT_DEADBAND_RAD_S) {
    return 0;
  }

  const float feedforward = setpoint > 0.0 ? feedforward_command : -feedforward_command;

  return static_cast<int>(pid_output + feedforward);
}

} // namespace

//  Motor velocity setpoints
float left_side_velocity_setpoint = 0;
float right_side_velocity_setpoint = 0;

//  PID outputs
float left_side_output = 0;
float right_side_output = 0;
unsigned long last_motor_command_ms = 0;

//  PID controllers for the left and right drivetrain sides
QuickPID left_side_pid(&current_left_side_rads_sec, &left_side_output,
  &left_side_velocity_setpoint);
QuickPID right_side_pid(&current_right_side_rads_sec, &right_side_output,
  &right_side_velocity_setpoint);

/**
 * @brief Function to setup the PID controllers
 */
void setup_pid()
{
  left_side_pid.SetOutputLimits(-127.0, 127.0);
  right_side_pid.SetOutputLimits(-127.0, 127.0);

  left_side_pid.SetTunings(8.0, 0.7, 0.00);
  right_side_pid.SetTunings(8.0, 0.7, 0.00);

  left_side_pid.SetSampleTimeUs(50000);
  right_side_pid.SetSampleTimeUs(50000);

  left_side_pid.SetMode(QuickPID::Control::automatic);
  right_side_pid.SetMode(QuickPID::Control::automatic);
}

void mark_motor_command_received()
{
  last_motor_command_ms = millis();
}

void stop_motors()
{
  left_side_velocity_setpoint = 0.0;
  right_side_velocity_setpoint = 0.0;
  left_side_output = 0.0;
  right_side_output = 0.0;
  left_side_pid.Reset();
  right_side_pid.Reset();
  send_sabertooth_command(Serial2, SABERTOOTH_ADDR, 1, 0);
  send_sabertooth_command(Serial2, SABERTOOTH_ADDR, 2, 0);
}

/**
 * @brief Function to send a command to the Sabertooth motor driver.
 *
 * This function constructs a command packet and sends it to the Sabertooth
 * motor driver over the specified serial port.
 *
 * @param port The serial port to send the command on.
 * @param address The address of the Sabertooth motor driver.
 * @param motor The motor number (1 or 2).
 * @param speed The speed value to set for the motor (-127 to 127).
 */
void send_sabertooth_command(HardwareSerial & port, byte address, byte motor, int speed)
{

  //  Constrain the speed to the valid range
  speed = constrain(speed, -127, 127);

  //  Determine the command based on the motor and speed direction
  byte command = (motor == 1) ? ((speed < 0) ? 1 : 0) : ((speed < 0) ? 5 : 4);

  //  Convert speed to absolute value for the command
  byte data = abs(speed);

  //  Calculate the checksum for the command
  byte checksum = (address + command + data) & 0x7F;

  //  Construct the command packet
  byte packet[] = {address, command, data, checksum};

  //  Send the command packet over the specified serial port
  port.write(packet, sizeof(packet));
}

/**
 * @brief Function to update the motors based on the PID output.
 */
void update_motors()
{
  if (battery_is_critical()) {
    stop_motors();
    return;
  }
  if (USE_ROS &&
    (last_motor_command_ms == 0 || millis() - last_motor_command_ms > MOTOR_COMMAND_TIMEOUT_MS))
  {
    stop_motors();
    return;
  }
  if (abs(left_side_velocity_setpoint) <= SETPOINT_DEADBAND_RAD_S &&
    abs(right_side_velocity_setpoint) <= SETPOINT_DEADBAND_RAD_S)
  {
    stop_motors();
    return;
  }

  //  Compute the PID output for both motors.
  left_side_pid.Compute();
  right_side_pid.Compute();

  if (SERIAL_DEBUG) {
    Serial.print("Left setpoint: ");
    Serial.print(left_side_velocity_setpoint);
    Serial.print(" measured: ");
    Serial.print(current_left_side_rads_sec);
    Serial.print(" output: ");
    Serial.print(left_side_output);
    Serial.print(" | Right setpoint: ");
    Serial.print(right_side_velocity_setpoint);
    Serial.print(" measured: ");
    Serial.print(current_right_side_rads_sec);
    Serial.print(" output: ");
    Serial.println(right_side_output);
  }

  const int left_side_command = constrain(
      command_with_feedforward(
        left_side_output, left_side_velocity_setpoint, LEFT_FEEDFORWARD_COMMAND),
      -DEBUG_MAX_MOTOR_COMMAND, DEBUG_MAX_MOTOR_COMMAND);
  const int right_side_command = constrain(
      -command_with_feedforward(
        right_side_output, right_side_velocity_setpoint, RIGHT_FEEDFORWARD_COMMAND),
      -DEBUG_MAX_MOTOR_COMMAND, DEBUG_MAX_MOTOR_COMMAND);

  if (SERIAL_DEBUG) {
    Serial.print(" commands: left=");
    Serial.print(left_side_command);
    Serial.print(" right=");
    Serial.println(right_side_command);
  }

  send_sabertooth_command(Serial2, SABERTOOTH_ADDR, 1, right_side_command);
  send_sabertooth_command(Serial2, SABERTOOTH_ADDR, 2, left_side_command);
}
