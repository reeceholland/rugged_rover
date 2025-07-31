#pragma once

#include <Arduino.h>
#include <QuickPID.h>

// Sabertooth settings
#define SABERTOOTH_ADDR 128

// Tuning constants
constexpr float PULSES_PER_REVOLUTION = 12.0;
constexpr float GEAR_RATIO_MULTIPLIER = 27.0;

// Control variables
extern float front_left_velocity_setpoint;
extern float front_right_velocity_setpoint;
extern float front_left_output;
extern float front_right_output;
extern QuickPID front_left_pid;

// ROS-shared velocity measurements
extern float current_front_left_rads_sec;
extern float current_front_right_rads_sec;


void setup_pid();
void send_sabertooth_command(HardwareSerial &port, byte address, byte motor, int speed);
void update_motors();
