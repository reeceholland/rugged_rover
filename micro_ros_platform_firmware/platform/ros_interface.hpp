#pragma once

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>

void ros_setup();
void spin_ros_executor();

// External ROS-related objects used across modules
extern rcl_node_t node;
extern rclc_executor_t executor;
extern rcl_subscription_t joint_state_subscriber;
extern rcl_publisher_t feedback_publisher;
extern sensor_msgs__msg__JointState cmd_msg;
extern sensor_msgs__msg__JointState feedback_msg;
