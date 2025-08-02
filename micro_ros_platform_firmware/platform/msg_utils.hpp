#pragma once

#include <sensor_msgs/msg/joint_state.h>

#define MAX_JOINTS 2
#define MAX_NAME_LEN 25

void initialise_joint_state_message(sensor_msgs__msg__JointState& msg);
void publish_joint_state_message();
void subscription_callback(const void* msgin);
