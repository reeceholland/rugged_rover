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

#include "msg_utils.hpp"
#include "battery_monitor.hpp"
#include "battery_safety.hpp"
#include "config.hpp"

#if USE_ROS
#include "encoder_utils.hpp"
#include "error_handling.hpp"
#include "motor_control.hpp"
#include "msg_utils.hpp"
#include "ros_interface.hpp"
#include <Arduino.h>
#include <cstring>

rosidl_runtime_c__String cmd_name_data[MAX_JOINTS];
rosidl_runtime_c__String feedback_name_data[MAX_JOINTS];
char cmd_name_buffer[MAX_JOINTS][MAX_NAME_LEN];
char feedback_name_buffer[MAX_JOINTS][MAX_NAME_LEN];
double cmd_position_data[MAX_JOINTS];
double cmd_velocity_data[MAX_JOINTS];
double feedback_position_data[MAX_JOINTS];
double feedback_velocity_data[MAX_JOINTS];
char debug_message_buffer[DEBUG_MESSAGE_LEN];

namespace
{

const char * const JOINT_NAMES[MAX_JOINTS] = {
  "front_left_joint",
  "front_right_joint",
  "rear_left_joint",
  "rear_right_joint",
};

} // namespace

/**
 * @brief Initialise the JointState message with default values.
 *
 * This function sets up the JointState message with predefined joint names and
 * initializes the position and velocity data. The effort sequence is left empty.
 *
 * @param msg The JointState message to initialize.
 */
void initialise_joint_state_message(sensor_msgs__msg__JointState & msg)
{
  const bool is_feedback_msg = (&msg == &feedback_msg);

  rosidl_runtime_c__String * names = is_feedback_msg ? feedback_name_data : cmd_name_data;
  char(*name_storage)[MAX_NAME_LEN] = is_feedback_msg ? feedback_name_buffer : cmd_name_buffer;
  double * positions = is_feedback_msg ? feedback_position_data : cmd_position_data;
  double * velocities = is_feedback_msg ? feedback_velocity_data : cmd_velocity_data;

  msg.name.data = names;
  msg.name.size = 4;
  msg.name.capacity = MAX_JOINTS;

  msg.velocity.data = velocities;
  msg.velocity.size = 4;
  msg.velocity.capacity = MAX_JOINTS;

  msg.position.data = positions;
  msg.position.size = 4;
  msg.position.capacity = MAX_JOINTS;

  msg.effort.data = NULL;
  msg.effort.size = 0;
  msg.effort.capacity = 0;

  for (size_t i = 0; i < MAX_JOINTS; ++i) {
    strncpy(name_storage[i], JOINT_NAMES[i], MAX_NAME_LEN - 1);
    name_storage[i][MAX_NAME_LEN - 1] = '\0';
    names[i].data = name_storage[i];
    names[i].size = strlen(name_storage[i]);
    names[i].capacity = MAX_NAME_LEN;
    positions[i] = 0.0;
    velocities[i] = 0.0;
  }
}

void initialise_battery_voltage_message(std_msgs__msg__Float32 & msg)
{
  msg.data = 0.0f;
}

void initialise_debug_message(std_msgs__msg__String & msg)
{
  debug_message_buffer[0] = '\0';
  msg.data.data = debug_message_buffer;
  msg.data.size = 0;
  msg.data.capacity = DEBUG_MESSAGE_LEN;
}

void publish_battery_voltage_message()
{
  static unsigned long last_publish_ms = 0;
  unsigned long now = millis();

  if (now - last_publish_ms < BATTERY_PUBLISH_PERIOD_MS) {
    return;
  }

  last_publish_ms = now;

  battery_voltage_msg.data = read_battery_voltage();
  RCSOFTCHECK(rcl_publish(&battery_voltage_publisher, &battery_voltage_msg, NULL));
}

void publish_debug_message()
{
  static unsigned long last_publish_ms = 0;
  const unsigned long now = millis();

  if (now - last_publish_ms < DEBUG_PUBLISH_PERIOD_MS) {
    return;
  }

  last_publish_ms = now;

  const long last_cmd_age_ms =
    last_motor_command_ms == 0 ? -1 : static_cast<long>(now - last_motor_command_ms);

  const int written = snprintf(
      debug_message_buffer, DEBUG_MESSAGE_LEN,
      "ros=%s battery_critical=%s battery_v=%.2f last_cmd_age_ms=%ld "
      "setpoint_left=%.3f setpoint_right=%.3f measured_left=%.3f measured_right=%.3f "
      "output_left=%.2f output_right=%.2f",
      ros_is_connected() ? "connected" : "waiting", battery_is_critical() ? "true" : "false",
      read_battery_voltage(), last_cmd_age_ms, left_side_velocity_setpoint,
      right_side_velocity_setpoint, current_left_side_rads_sec, current_right_side_rads_sec,
      left_side_output, right_side_output);

  if (written < 0) {
    return;
  }

  debug_message_buffer[DEBUG_MESSAGE_LEN - 1] = '\0';
  debug_msg.data.size =
    written >= DEBUG_MESSAGE_LEN ? DEBUG_MESSAGE_LEN - 1 : static_cast<size_t>(written);

  RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
}

/**
 * @brief Publish the JointState message with current velocity data.
 *
 * This function updates the velocity data in the JointState message with the
 * current velocities of the left and right drivetrain sides, then publishes
 * the message.
 */
void publish_joint_state_message()
{
  feedback_msg.position.data[0] = current_left_side_position_rad;
  feedback_msg.position.data[1] = current_right_side_position_rad;
  feedback_msg.position.data[2] = current_left_side_position_rad;
  feedback_msg.position.data[3] = current_right_side_position_rad;

  feedback_msg.velocity.data[0] = current_left_side_rads_sec;
  feedback_msg.velocity.data[1] = current_right_side_rads_sec;
  feedback_msg.velocity.data[2] = current_left_side_rads_sec;
  feedback_msg.velocity.data[3] = current_right_side_rads_sec;
  RCSOFTCHECK(rcl_publish(&feedback_publisher, &feedback_msg, NULL));
}

/**
 * @brief Callback function for the subscription to JointState messages.
 *
 * This function processes incoming JointState messages, extracting the joint
 * names and their corresponding velocity setpoints. It updates the global
 * velocity setpoints for the left and right drivetrain sides based on the
 * received message.
 *
 * @param msgin Pointer to the incoming JointState message.
 */
void subscription_callback(const void * msgin)
{
  if (battery_is_critical()) {
    stop_motors();
    return;
  }
  //  Cast the incoming message to the expected type
  const sensor_msgs__msg__JointState * joint_msg = (const sensor_msgs__msg__JointState *) msgin;
  bool received_motor_command = false;

  //  For each joint in the message, check the name and update the corresponding
  //  velocity setpoint
  for (size_t i = 0; i < joint_msg->name.size && i < MAX_JOINTS; i++) {
    if (i >= joint_msg->velocity.size) {
      break;
    }

    const char * name = joint_msg->name.data[i].data;
    float velocity = joint_msg->velocity.data[i];
    if (strcmp(name, "front_left_joint") == 0) {
      left_side_velocity_setpoint = velocity;
      received_motor_command = true;
    } else if (strcmp(name, "front_right_joint") == 0) {
      right_side_velocity_setpoint = velocity;
      received_motor_command = true;
    } else if (strcmp(name, "rear_left_joint") == 0) {
      left_side_velocity_setpoint = velocity;
      received_motor_command = true;
    } else if (strcmp(name, "rear_right_joint") == 0) {
      right_side_velocity_setpoint = velocity;
      received_motor_command = true;
    }
  }

  if (received_motor_command) {
    mark_motor_command_received();
  }

  if (abs(left_side_velocity_setpoint) <= 0.05 && abs(right_side_velocity_setpoint) <= 0.05) {
    stop_motors();
  }
}
#else
void publish_joint_state_message() {}
void publish_battery_voltage_message() {}
void publish_debug_message() {}
#endif
