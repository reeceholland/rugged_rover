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
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#endif

void ros_setup();
void ros_update();
void spin_ros_executor();
bool ros_is_connected();

#if USE_ROS
// External ROS-related objects used across modules
extern rcl_node_t node;
extern rclc_executor_t executor;
extern rcl_subscription_t joint_state_subscriber;
extern rcl_publisher_t feedback_publisher;
extern rcl_publisher_t battery_voltage_publisher;
extern rcl_publisher_t debug_publisher;
extern sensor_msgs__msg__JointState cmd_msg;
extern sensor_msgs__msg__JointState feedback_msg;
extern std_msgs__msg__Float32 battery_voltage_msg;
extern std_msgs__msg__String debug_msg;
#endif
