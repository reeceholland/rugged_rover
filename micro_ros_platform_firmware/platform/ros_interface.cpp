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
#include <std_msgs/msg/string.h>

rcl_subscription_t joint_state_subscriber;
rcl_publisher_t feedback_publisher;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t battery_voltage_publisher;
rcl_publisher_t debug_publisher;

sensor_msgs__msg__JointState cmd_msg;
sensor_msgs__msg__JointState feedback_msg;
std_msgs__msg__Float32 battery_voltage_msg;
std_msgs__msg__String debug_msg;

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

namespace
{
  enum class RosConnectionState
  {
    WaitingForAgent,
    Connected,
  };

  RosConnectionState ros_state = RosConnectionState::WaitingForAgent;
  unsigned long last_agent_ping_ms = 0;
  bool support_created = false;
  bool node_created = false;
  bool battery_publisher_created = false;
  bool debug_publisher_created = false;
  bool motor_subscription_created = false;
  bool feedback_publisher_created = false;
  bool executor_created = false;
  bool ros_entities_created = false;

  constexpr unsigned long AGENT_PING_PERIOD_MS = 1000;
  constexpr int AGENT_PING_TIMEOUT_MS = 100;
  constexpr int AGENT_PING_ATTEMPTS = 1;

  void destroy_ros_entities();

  bool create_ros_entities()
  {
    destroy_ros_entities();

    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    {
      return false;
    }
    support_created = true;

    if (rclc_node_init_default(&node, "micro_ros_platform_node", "", &support) != RCL_RET_OK)
    {
      destroy_ros_entities();
      return false;
    }
    node_created = true;

    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);

    sensor_msgs__msg__JointState__init(&cmd_msg);
    sensor_msgs__msg__JointState__init(&feedback_msg);
    initialise_joint_state_message(cmd_msg);
    initialise_joint_state_message(feedback_msg);
    initialise_battery_voltage_message(battery_voltage_msg);
    initialise_debug_message(debug_msg);

    const rosidl_message_type_support_t* type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);
    const rosidl_message_type_support_t* battery_type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);
    const rosidl_message_type_support_t* debug_type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

    rcl_subscription_options_t sub_ops = rcl_subscription_get_default_options();
    sub_ops.qos = rmw_qos_profile_sensor_data;

    rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
    pub_ops.qos = rmw_qos_profile_sensor_data;

    if (rcl_publisher_init(&battery_voltage_publisher, &node, battery_type_support,
                           "battery/voltage", &pub_ops) != RCL_RET_OK)
    {
      destroy_ros_entities();
      return false;
    }
    battery_publisher_created = true;

    if (rcl_publisher_init(&debug_publisher, &node, debug_type_support, "platform/debug",
                           &pub_ops) != RCL_RET_OK)
    {
      destroy_ros_entities();
      return false;
    }
    debug_publisher_created = true;

    if (rcl_subscription_init(&joint_state_subscriber, &node, type_support,
                              "platform/motors/cmd", &sub_ops) != RCL_RET_OK)
    {
      destroy_ros_entities();
      return false;
    }
    motor_subscription_created = true;

    if (rcl_publisher_init(&feedback_publisher, &node, type_support,
                           "platform/motors/feedback", &pub_ops) != RCL_RET_OK)
    {
      destroy_ros_entities();
      return false;
    }
    feedback_publisher_created = true;

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
    {
      destroy_ros_entities();
      return false;
    }
    executor_created = true;

    if (rclc_executor_add_subscription(&executor, &joint_state_subscriber, &cmd_msg,
                                       &subscription_callback, ON_NEW_DATA) != RCL_RET_OK)
    {
      destroy_ros_entities();
      return false;
    }

    ros_entities_created = true;
    return true;
  }

  void destroy_ros_entities()
  {
    if (executor_created)
    {
      rclc_executor_fini(&executor);
      executor_created = false;
    }

    if (feedback_publisher_created)
    {
      rcl_publisher_fini(&feedback_publisher, &node);
      feedback_publisher_created = false;
    }

    if (debug_publisher_created)
    {
      rcl_publisher_fini(&debug_publisher, &node);
      debug_publisher_created = false;
    }

    if (battery_publisher_created)
    {
      rcl_publisher_fini(&battery_voltage_publisher, &node);
      battery_publisher_created = false;
    }

    if (motor_subscription_created)
    {
      rcl_subscription_fini(&joint_state_subscriber, &node);
      motor_subscription_created = false;
    }

    if (node_created)
    {
      rcl_node_fini(&node);
      node_created = false;
    }

    if (support_created)
    {
      rclc_support_fini(&support);
      support_created = false;
    }

    ros_entities_created = false;
  }

  bool agent_is_available()
  {
    return rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS) == RMW_RET_OK;
  }
} // namespace

/**
 * @brief Prepare serial transports. ROS entities are created by ros_update().
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

  ros_state = RosConnectionState::WaitingForAgent;
  last_agent_ping_ms = 0;
}

bool ros_is_connected()
{
  return ros_state == RosConnectionState::Connected;
}

void ros_update()
{
  const unsigned long now = millis();

  if (ros_state == RosConnectionState::WaitingForAgent)
  {
    if (now - last_agent_ping_ms < AGENT_PING_PERIOD_MS)
      return;

    last_agent_ping_ms = now;

    if (agent_is_available())
    {
      destroy_ros_entities();
      if (create_ros_entities())
      {
        stop_motors();
        last_motor_command_ms = 0;
        ros_state = RosConnectionState::Connected;
      }
      else
      {
        stop_motors();
        destroy_ros_entities();
      }
    }
    return;
  }

  spin_ros_executor();

  if (now - last_agent_ping_ms >= AGENT_PING_PERIOD_MS)
  {
    last_agent_ping_ms = now;

    if (!agent_is_available())
    {
      stop_motors();
      last_motor_command_ms = 0;
      destroy_ros_entities();
      ros_state = RosConnectionState::WaitingForAgent;
    }
  }
}

/**
 * @brief Spin the ROS executor to process incoming messages.
 */
void spin_ros_executor()
{
  if (!ros_entities_created)
    return;

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
}
#else
void ros_setup() {}
void ros_update() {}
void spin_ros_executor() {}
bool ros_is_connected() { return false; }
#endif
