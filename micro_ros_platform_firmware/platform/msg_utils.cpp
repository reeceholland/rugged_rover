#include "msg_utils.hpp"
#include "config.hpp"

#if USE_ROS
#include "encoder_utils.hpp"
#include "error_handling.hpp"
#include "motor_control.hpp"
#include "msg_utils.hpp"
#include "ros_interface.hpp"
#include <cstring>

rosidl_runtime_c__String cmd_name_data[MAX_JOINTS];
rosidl_runtime_c__String feedback_name_data[MAX_JOINTS];
char cmd_name_buffer[MAX_JOINTS][MAX_NAME_LEN];
char feedback_name_buffer[MAX_JOINTS][MAX_NAME_LEN];
double cmd_position_data[MAX_JOINTS];
double cmd_velocity_data[MAX_JOINTS];
double feedback_position_data[MAX_JOINTS];
double feedback_velocity_data[MAX_JOINTS];

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
  const bool is_feedback_msg = (&msg == &feedback_msg);

  rosidl_runtime_c__String* names = is_feedback_msg ? feedback_name_data : cmd_name_data;
  char (*name_storage)[MAX_NAME_LEN] = is_feedback_msg ? feedback_name_buffer : cmd_name_buffer;
  double* positions = is_feedback_msg ? feedback_position_data : cmd_position_data;
  double* velocities = is_feedback_msg ? feedback_velocity_data : cmd_velocity_data;

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

  for (size_t i = 0; i < MAX_JOINTS; ++i)
  {
    strncpy(name_storage[i], JOINT_NAMES[i], MAX_NAME_LEN - 1);
    name_storage[i][MAX_NAME_LEN - 1] = '\0';
    names[i].data = name_storage[i];
    names[i].size = strlen(name_storage[i]);
    names[i].capacity = MAX_NAME_LEN;
    positions[i] = 0.0;
    velocities[i] = 0.0;
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
    if (i >= joint_msg->velocity.size)
    {
      break;
    }

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
