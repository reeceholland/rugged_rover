#include "msg_utils.hpp"
#include "config.hpp"
#include "encoder_utils.hpp"
#include "error_handling.hpp"
#include "motor_control.hpp"
#include "msg_utils.hpp"
#include "ros_interface.hpp"
#include <cstring>

rosidl_runtime_c__String name_data[MAX_JOINTS];
char name_buffer[MAX_JOINTS][MAX_NAME_LEN];
double velocity_data[MAX_JOINTS];

void initialise_joint_state_message(sensor_msgs__msg__JointState& msg)
{
  msg.name.data = name_data;
  msg.name.size = 2;
  msg.name.capacity = MAX_JOINTS;

  msg.velocity.data = velocity_data;
  msg.velocity.size = 2;
  msg.velocity.capacity = MAX_JOINTS;

  msg.position.data = NULL;
  msg.position.size = 0;
  msg.position.capacity = 0;

  msg.effort.data = NULL;
  msg.effort.size = 0;
  msg.effort.capacity = 0;

  name_data[0].data = (char*) "left_front_wheel_joint";
  name_data[0].size = strlen(name_data[0].data);
  name_data[0].capacity = MAX_NAME_LEN;

  name_data[1].data = (char*) "right_front_wheel_joint";
  name_data[1].size = strlen(name_data[1].data);
  name_data[1].capacity = MAX_NAME_LEN;

  // name_data[2].data = (char *)"left_rear_wheel_joint";
  // name_data[2].size = strlen(name_data[2].data);
  // name_data[2].capacity = MAX_NAME_LEN;

  // name_data[3].data = (char *)"right_rear_wheel_joint";
  // name_data[3].size = strlen(name_data[3].data);
  // name_data[3].capacity = MAX_NAME_LEN;
}

void publish_joint_state_message()
{
  feedback_msg.velocity.data[0] = current_front_left_rads_sec;
  feedback_msg.velocity.data[1] = current_front_right_rads_sec;
  RCSOFTCHECK(rcl_publish(&feedback_publisher, &feedback_msg, NULL));
}

void subscription_callback(const void* msgin)
{
  const sensor_msgs__msg__JointState* joint_msg = (const sensor_msgs__msg__JointState*) msgin;
  for (size_t i = 0; i < joint_msg->name.size && i < MAX_JOINTS; i++)
  {
    const char* name = joint_msg->name.data[i].data;
    float velocity = joint_msg->velocity.data[i];
    if (strcmp(name, "left_front_wheel_joint") == 0)
      front_left_velocity_setpoint = velocity;
    else if (strcmp(name, "right_front_wheel_joint") == 0)
      front_right_velocity_setpoint = velocity;
    // else if (strcmp(name, "left_rear_wheel_joint") == 0)
    //   front_left_velocity_setpoint = velocity; // Adjust as needed for rear
    //   wheels
    // else if (strcmp(name, "right_rear_wheel_joint") == 0)
    //   front_right_velocity_setpoint = velocity; // Adjust as needed for rear
    //   wheels
  }
}
