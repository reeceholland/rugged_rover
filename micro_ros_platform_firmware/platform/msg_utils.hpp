#pragma once

#include "config.hpp"

#if USE_ROS
#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32.h>
#endif

#define MAX_JOINTS 4
#define MAX_NAME_LEN 25

#if USE_ROS
void initialise_joint_state_message(sensor_msgs__msg__JointState& msg);
void initialise_battery_voltage_message(std_msgs__msg__Float32& msg);
#endif
void publish_joint_state_message();
void publish_battery_voltage_message();
#if USE_ROS
void subscription_callback(const void* msgin);
#endif
