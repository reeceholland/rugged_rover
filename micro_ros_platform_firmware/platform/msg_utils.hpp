#pragma once

#include "config.hpp"

#if USE_ROS
#include <sensor_msgs/msg/joint_state.h>
#endif

#define MAX_JOINTS 4
#define MAX_NAME_LEN 25

#if USE_ROS
void initialise_joint_state_message(sensor_msgs__msg__JointState &msg);
#endif
void publish_joint_state_message();
#if USE_ROS
void subscription_callback(const void *msgin);
#endif
