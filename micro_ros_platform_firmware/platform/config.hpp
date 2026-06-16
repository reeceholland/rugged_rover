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

// Set this to 1 for normal micro-ROS operation.
// Set this to 0 for USB serial bench testing without running the ROS agent.
#define USE_ROS 1

extern bool SERIAL_DEBUG;
constexpr bool ROS_SERIAL_STATUS_DEBUG = false;

#define BATTERY_VOLTAGE_PIN A9

constexpr float BATTERY_R_TOP = 100000.0f;
constexpr float BATTERY_R_BOTTOM = 33000.0f;
constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
constexpr float BATTERY_PUBLISH_PERIOD_MS = 1000.0f;
constexpr unsigned long DEBUG_PUBLISH_PERIOD_MS = 1000;
constexpr unsigned long MOTOR_COMMAND_TIMEOUT_MS = 500;
