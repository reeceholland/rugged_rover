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
extern float rear_left_velocity_setpoint;
extern float rear_right_velocity_setpoint;
extern float rear_left_output;
extern float rear_right_output;
extern unsigned long last_motor_command_ms;
extern QuickPID rear_left_pid;
extern QuickPID rear_right_pid;

// ROS-shared velocity measurements
extern float current_rear_left_rads_sec;
extern float current_rear_right_rads_sec;

void setup_pid();
void mark_motor_command_received();
void stop_motors();
void send_sabertooth_command(HardwareSerial & port, byte address, byte motor, int speed);
void update_motors();
