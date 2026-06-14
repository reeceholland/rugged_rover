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

#include "config.hpp"

#if USE_ROS
#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#endif

#define MAX_JOINTS 4
#define MAX_NAME_LEN 25
#define DEBUG_MESSAGE_LEN 192

#if USE_ROS
void initialise_joint_state_message(sensor_msgs__msg__JointState & msg);
void initialise_battery_voltage_message(std_msgs__msg__Float32 & msg);
void initialise_debug_message(std_msgs__msg__String & msg);
#endif
void publish_joint_state_message();
void publish_battery_voltage_message();
void publish_debug_message();
#if USE_ROS
void subscription_callback(const void * msgin);
#endif
