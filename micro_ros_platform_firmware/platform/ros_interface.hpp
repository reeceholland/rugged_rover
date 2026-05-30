#pragma once

#include "config.hpp"

#if USE_ROS
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32.h>
#endif

void ros_setup();
void spin_ros_executor();

#if USE_ROS
// External ROS-related objects used across modules
extern rcl_node_t node;
extern rclc_executor_t executor;
extern rcl_subscription_t joint_state_subscriber;
extern rcl_publisher_t feedback_publisher;
extern rcl_publisher_t battery_voltage_publisher;
extern sensor_msgs__msg__JointState cmd_msg;
extern sensor_msgs__msg__JointState feedback_msg;
extern std_msgs__msg__Float32 battery_voltage_msg;
#endif
