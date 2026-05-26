#include "msg_utils.hpp"
#include "config.hpp"

#if USE_ROS
#include "encoder_utils.hpp"
#include "error_handling.hpp"
#include "motor_control.hpp"
#include "msg_utils.hpp"
#include "ros_interface.hpp"
#include <cstring>

rosidl_runtime_c__String name_data[MAX_JOINTS];
char name_buffer[MAX_JOINTS][MAX_NAME_LEN];
double position_data[MAX_JOINTS];
double velocity_data[MAX_JOINTS];

namespace
{

  const char* const JOINT_NAMES[MAX_JOINTS] = {
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
void initialise_joint_state_message(sensor_msgs__msg__JointState& msg)
{
  msg.name.data = name_data;
  msg.name.size = 4;
  msg.name.capacity = MAX_JOINTS;

  msg.velocity.data = velocity_data;
  msg.velocity.size = 4;
  msg.velocity.capacity = MAX_JOINTS;

  msg.position.data = position_data;
  msg.position.size = 4;
  msg.position.capacity = MAX_JOINTS;

  msg.effort.data = NULL;
  msg.effort.size = 0;
  msg.effort.capacity = 0;

  for (size_t i = 0; i < MAX_JOINTS; ++i)
  {
    strncpy(name_buffer[i], JOINT_NAMES[i], MAX_NAME_LEN - 1);
    name_buffer[i][MAX_NAME_LEN - 1] = '\0';
    name_data[i].data = name_buffer[i];
    name_data[i].size = strlen(name_buffer[i]);
    name_data[i].capacity = MAX_NAME_LEN;
    position_data[i] = 0.0;
    velocity_data[i] = 0.0;
  }
}

/**
 * @brief Publish the JointState message with current velocity data.
 *
 * This function updates the velocity data in the JointState message with the
 * current velocities of the front left and front right motors, then publishes
 * the message.
 */
void publish_joint_state_message()
{
  feedback_msg.position.data[0] = current_front_left_position_rad;
  feedback_msg.position.data[1] = current_front_right_position_rad;
  feedback_msg.position.data[2] = current_front_left_position_rad;
  feedback_msg.position.data[3] = current_front_right_position_rad;

  feedback_msg.velocity.data[0] = current_front_left_rads_sec;
  feedback_msg.velocity.data[1] = current_front_right_rads_sec;
  feedback_msg.velocity.data[2] = current_front_left_rads_sec;
  feedback_msg.velocity.data[3] = current_front_right_rads_sec;
  RCSOFTCHECK(rcl_publish(&feedback_publisher, &feedback_msg, NULL));
}

/**
 * @brief Callback function for the subscription to JointState messages.
 *
 * This function processes incoming JointState messages, extracting the joint
 * names and their corresponding velocity setpoints. It updates the global
 * velocity setpoints for the front left and front right motors based on the
 * received message.
 *
 * @param msgin Pointer to the incoming JointState message.
 */
void subscription_callback(const void* msgin)
{
  //  Cast the incoming message to the expected type
  const sensor_msgs__msg__JointState* joint_msg = (const sensor_msgs__msg__JointState*) msgin;

  //  For each joint in the message, check the name and update the corresponding
  //  velocity setpoint
  for (size_t i = 0; i < joint_msg->name.size && i < MAX_JOINTS; i++)
  {
    const char* name = joint_msg->name.data[i].data;
    float velocity = joint_msg->velocity.data[i];
    if (strcmp(name, "front_left_joint") == 0)
      front_left_velocity_setpoint = velocity;
    else if (strcmp(name, "front_right_joint") == 0)
      front_right_velocity_setpoint = velocity;
    else if (strcmp(name, "rear_left_joint") == 0)
      front_left_velocity_setpoint = velocity;
    else if (strcmp(name, "rear_right_joint") == 0)
      front_right_velocity_setpoint = velocity;
  }

  if (abs(front_left_velocity_setpoint) <= 0.05 && abs(front_right_velocity_setpoint) <= 0.05)
  {
    stop_motors();
  }
}
#else
void publish_joint_state_message() {}
#endif
