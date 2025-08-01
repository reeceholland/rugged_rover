#include "ros_interface.hpp"
#include "config.hpp"
#include "error_handling.hpp"
#include "motor_control.hpp"
#include "msg_utils.hpp"
#include <rclc/executor.h>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/joint_state.h>

rcl_subscription_t joint_state_subscriber;
rcl_publisher_t feedback_publisher;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

sensor_msgs__msg__JointState cmd_msg;
sensor_msgs__msg__JointState feedback_msg;

void ros_setup()
{
  // Serial setup
  Serial2.begin(9600); // Sabertooth
  if (SERIAL_DEBUG)
    Serial1.begin(115200);

  // micro-ROS transport setup
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);

  // PID tuning

  // ROS node and entities
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platform_node", "", &support));

  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);

  sensor_msgs__msg__JointState__init(&cmd_msg);
  sensor_msgs__msg__JointState__init(&feedback_msg);
  initialise_joint_state_message(cmd_msg);
  initialise_joint_state_message(feedback_msg);

  const rosidl_message_type_support_t* type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);

  rcl_subscription_options_t sub_ops = rcl_subscription_get_default_options();
  sub_ops.qos = rmw_qos_profile_sensor_data;

  RCCHECK(rcl_subscription_init(&joint_state_subscriber, &node, type_support, "platform/motors/cmd",
                                &sub_ops));

  rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
  pub_ops.qos = rmw_qos_profile_sensor_data;

  RCCHECK(rcl_publisher_init(&feedback_publisher, &node, type_support, "platform/motors/feedback",
                             &pub_ops));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &joint_state_subscriber, &cmd_msg,
                                         &subscription_callback, ON_NEW_DATA));
}

void spin_ros_executor()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
