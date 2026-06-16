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

constexpr int LEFT_FEEDFORWARD_COMMAND = 10;
constexpr int RIGHT_FEEDFORWARD_COMMAND = 13;
constexpr float SETPOINT_DEADBAND_RAD_S = 0.05;

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
float front_left_velocity_setpoint = 0;
float front_right_velocity_setpoint = 0;

//  PID outputs
float front_left_output = 0;
float front_right_output = 0;
unsigned long last_motor_command_ms = 0;

//  The PID controller for the front left motor
QuickPID front_left_pid(&current_front_left_rads_sec, &front_left_output,
  &front_left_velocity_setpoint);
QuickPID front_right_pid(&current_front_right_rads_sec, &front_right_output,
  &front_right_velocity_setpoint);

/**
 * @brief Function to setup the PID controllers
 */
void setup_pid()
{
  front_left_pid.SetOutputLimits(-127.0, 127.0);
  front_right_pid.SetOutputLimits(-127.0, 127.0);

  front_left_pid.SetTunings(8.0, 0.7, 0.00);
  front_right_pid.SetTunings(8.0, 0.7, 0.00);

  front_left_pid.SetSampleTimeUs(50000);
  front_right_pid.SetSampleTimeUs(50000);

  front_left_pid.SetMode(QuickPID::Control::automatic);
  front_right_pid.SetMode(QuickPID::Control::automatic);
}

void mark_motor_command_received()
{
  last_motor_command_ms = millis();
}

void stop_motors()
{
  front_left_velocity_setpoint = 0.0;
  front_right_velocity_setpoint = 0.0;
  front_left_output = 0.0;
  front_right_output = 0.0;
  front_left_pid.Reset();
  front_right_pid.Reset();
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
  if (abs(front_left_velocity_setpoint) <= SETPOINT_DEADBAND_RAD_S &&
    abs(front_right_velocity_setpoint) <= SETPOINT_DEADBAND_RAD_S)
  {
    stop_motors();
    return;
  }

  //  Compute the PID output for both motors.
  front_left_pid.Compute();
  front_right_pid.Compute();

  if (SERIAL_DEBUG) {
    Serial.print("FL setpoint: ");
    Serial.print(front_left_velocity_setpoint);
    Serial.print(" measured: ");
    Serial.print(current_front_left_rads_sec);
    Serial.print(" output: ");
    Serial.print(front_left_output);
    Serial.print(" | FR setpoint: ");
    Serial.print(front_right_velocity_setpoint);
    Serial.print(" measured: ");
    Serial.print(current_front_right_rads_sec);
    Serial.print(" output: ");
    Serial.println(front_right_output);
  }

  const int front_left_command = command_with_feedforward(
      front_left_output, front_left_velocity_setpoint, LEFT_FEEDFORWARD_COMMAND);
  const int front_right_command = -command_with_feedforward(
      front_right_output, front_right_velocity_setpoint, RIGHT_FEEDFORWARD_COMMAND);

  if (SERIAL_DEBUG) {
    Serial.print(" commands: FL=");
    Serial.print(front_left_command);
    Serial.print(" FR=");
    Serial.println(front_right_command);
  }

  send_sabertooth_command(Serial2, SABERTOOTH_ADDR, 1, front_left_command);
  send_sabertooth_command(Serial2, SABERTOOTH_ADDR, 2, front_right_command);
}
