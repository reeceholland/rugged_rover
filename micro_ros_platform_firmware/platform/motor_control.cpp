#include "motor_control.hpp"
#include "config.hpp"

//  Motor velocity setpoints
float front_left_velocity_setpoint = 0;
float front_right_velocity_setpoint = 0;

//  PID outputs
float front_left_output = 0;
float front_right_output = 0;

//  The PID controller for the front left motor
QuickPID front_left_pid(&current_front_left_rads_sec, &front_left_output,
                        &front_left_velocity_setpoint);

/**
 * @brief Function to setup the PID controllers
 */
void setup_pid() {
  front_left_pid.SetOutputLimits(-127.0, 127.0);
  front_left_pid.SetTunings(3.0, 0.0, 0.00);
  front_left_pid.SetSampleTimeUs(100000);
  front_left_pid.SetMode(QuickPID::Control::automatic);
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
void send_sabertooth_command(HardwareSerial &port, byte address, byte motor,
                             int speed) {

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

  if (SERIAL_DEBUG)
    Serial1.println(command);

  //  Send the command packet over the specified serial port
  port.write(packet, sizeof(packet));
}

/**
 * @brief Function to update the motors based on the PID output.
 */
void update_motors() {

  //  Compute the PID output for the front left motor
  front_left_pid.Compute();

  if (SERIAL_DEBUG) {
    Serial1.print("Front Left PID Output: ");
    Serial1.println(front_left_output);
  }

  //  Send the command to the Sabertooth motor driver for the front left motor
  send_sabertooth_command(Serial2, SABERTOOTH_ADDR, 1,
                          front_left_velocity_setpoint * 4);
}