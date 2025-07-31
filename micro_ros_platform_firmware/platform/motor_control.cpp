#include "motor_control.hpp"
#include "config.hpp"

float front_left_velocity_setpoint = 0;
float front_right_velocity_setpoint = 0;
float front_left_output = 0;
float front_right_output = 0;

QuickPID front_left_pid(&current_front_left_rads_sec, &front_left_output, &front_left_velocity_setpoint);

void setup_pid()
{
  front_left_pid.SetOutputLimits(-127.0, 127.0);
  front_left_pid.SetTunings(6.0, 0.0, 0.00);
  front_left_pid.SetSampleTimeUs(100000);
  front_left_pid.SetMode(QuickPID::Control::automatic);
}

void send_sabertooth_command(HardwareSerial &port, byte address, byte motor, int speed) {
  speed = constrain(speed, -127, 127);
  byte command = (motor == 1) ? ((speed < 0) ? 1 : 0) : ((speed < 0) ? 5 : 4);
  byte data = abs(speed);
  byte checksum = (address + command + data) & 0x7F;
  byte packet[] = { address, command, data, checksum };
  if (SERIAL_DEBUG) Serial1.println(command);
  port.write(packet, sizeof(packet));
}

void update_motors() {
  front_left_pid.Compute();
  if (SERIAL_DEBUG) {
    Serial1.print("Front Left PID Output: ");
    Serial1.println(front_left_output);
  }
  send_sabertooth_command(Serial2, SABERTOOTH_ADDR, 1, front_left_output);
}