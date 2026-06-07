#pragma once

#include <Arduino.h>
#include <QuickPID.h>

// Sabertooth settings
#define SABERTOOTH_ADDR 128

// Tuning constants
// The encoder library counts quadrature edges. A 12 PPR encoder normally
// produces 48 counts per motor revolution when both channels are decoded.
constexpr float ENCODER_COUNTS_PER_MOTOR_REV = 48.0;
constexpr float GEAR_RATIO_MULTIPLIER = 51.0;

// Control variables
extern float front_left_velocity_setpoint;
extern float front_right_velocity_setpoint;
extern float front_left_output;
extern float front_right_output;
extern unsigned long last_motor_command_ms;
extern QuickPID front_left_pid;
extern QuickPID front_right_pid;

// ROS-shared velocity measurements
extern float current_front_left_rads_sec;
extern float current_front_right_rads_sec;

void setup_pid();
void mark_motor_command_received();
void stop_motors();
void send_sabertooth_command(HardwareSerial& port, byte address, byte motor, int speed);
void update_motors();
