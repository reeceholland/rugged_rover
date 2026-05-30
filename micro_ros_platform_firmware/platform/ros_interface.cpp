#include "ros_interface.hpp"
#include "config.hpp"

#if USE_ROS
#include "error_handling.hpp"
#include "motor_control.hpp"
#include "msg_utils.hpp"
#include <rclc/executor.h>
#include <rmw/qos_profiles.h>
#include <rmw_microros/ping.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32.h>

rcl_subscription_t joint_state_subscriber;
rcl_publisher_t feedback_publisher;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t battery_voltage_publisher;

sensor_msgs__msg__JointState cmd_msg;
sensor_msgs__msg__JointState feedback_msg;
std_msgs__msg__Float32 battery_voltage_msg;

extern "C" bool arduino_transport_open(struct uxrCustomTransport*)
{
  Serial1.begin(115200);
  return true;
}

extern "C" bool arduino_transport_close(struct uxrCustomTransport*)
{
  Serial1.end();
  return true;
}

extern "C" size_t arduino_transport_write(struct uxrCustomTransport*, const uint8_t* buf,
                                          size_t len, uint8_t* err)
{
  (void) err;
  return Serial1.write(buf, len);
}

extern "C" size_t arduino_transport_read(struct uxrCustomTransport*, uint8_t* buf, size_t len,
                                         int timeout, uint8_t* err)
{
  (void) err;
  Serial1.setTimeout(timeout);
  return Serial1.readBytes(reinterpret_cast<char*>(buf), len);
}

/**
 * @brief Setup the micro-ROS environment and initialize the node.
 *
 * This function initializes the micro-ROS node, sets up the serial
 * communication, and prepares the JointState message for publishing and
 * subscribing.
 */
void ros_setup()
{
  // Serial2 is reserved for Teensy -> Sabertooth motor commands.
  Serial2.begin(9600);

  // Serial1 is the Raspberry Pi <-> Teensy micro-ROS transport.
  // Wire Pi TXD0 GPIO14/pin 8  -> Teensy RX1/pin 0
  // Wire Pi RXD0 GPIO15/pin 10 -> Teensy TX1/pin 1
  // Wire Pi GND                -> Teensy GND
  Serial1.begin(115200);
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Give the agent a moment to appear before creating ROS entities. This is
  // especially helpful after USB reconnects and Teensy uploads.
  while (rmw_uros_ping_agent(100, 5) != RMW_RET_OK)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }

  digitalWrite(LED_PIN, HIGH);

  // ROS node and entities
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platform_node", "", &support));

  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);

  sensor_msgs__msg__JointState__init(&cmd_msg);
  sensor_msgs__msg__JointState__init(&feedback_msg);
  initialise_joint_state_message(cmd_msg);
  initialise_joint_state_message(feedback_msg);
  initialise_battery_voltage_message(battery_voltage_msg);

  const rosidl_message_type_support_t* type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);

  rcl_subscription_options_t sub_ops = rcl_subscription_get_default_options();
  sub_ops.qos = rmw_qos_profile_sensor_data;

  rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
  pub_ops.qos = rmw_qos_profile_sensor_data;

  const rosidl_message_type_support_t* battery_type_support =
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);

  RCCHECK(rcl_publisher_init(&battery_voltage_publisher, &node, battery_type_support,
                             "battery/voltage", &pub_ops));

  RCCHECK(rcl_subscription_init(&joint_state_subscriber, &node, type_support, "platform/motors/cmd",
                                &sub_ops));

  RCCHECK(rcl_publisher_init(&feedback_publisher, &node, type_support, "platform/motors/feedback",
                             &pub_ops));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &joint_state_subscriber, &cmd_msg,
                                         &subscription_callback, ON_NEW_DATA));
}

/**
 * @brief Spin the ROS executor to process incoming messages.
 *
 * This function processes incoming messages and executes the associated
 * callbacks.
 *
 */
void spin_ros_executor()
{
  //  Spin the executor to process incoming messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
}
#else
void ros_setup() {}
void spin_ros_executor() {}
#endif
