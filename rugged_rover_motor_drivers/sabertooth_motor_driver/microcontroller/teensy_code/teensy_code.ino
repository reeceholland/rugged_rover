#include <QuickPID.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/joint_state.h>
#include <Encoder.h>

// === Hardware Config ===
#define LED_PIN 13
#define MAX_JOINTS 4
#define MAX_NAME_LEN 20
#define SABERTOOTH_ADDR 128

bool SERIAL_DEBUG = true;

Encoder frontLeftEncoder(2, 3);
Encoder frontRightEncoder(4, 5);

volatile long frontLeftLastTicks;
volatile long frontRightLastTicks;

float front_left_velocity_setpoint = 0;
float front_right_velocity_setpoint = 0;

const float PULSES_PER_REVOLUTION = 12.0;
const float GEAR_RATIO_MULTIPLIER = 27.0;

float current_front_left_rads_sec = 0;
float current_front_right_rads_sec = 0;

float front_left_output = 0;
float front_right_output = 0;

unsigned long lastEncoderSampleTime = 0;

// === ROS Entities ===
rcl_subscription_t joint_state_subscriber;
rcl_publisher_t feedback_publisher;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

sensor_msgs__msg__JointState cmd_msg;
sensor_msgs__msg__JointState feedback_msg;

// === Static Buffers for JointState ===
static rosidl_runtime_c__String name_data[MAX_JOINTS];
static char name_buffer[MAX_JOINTS][MAX_NAME_LEN];
static double velocity_data[MAX_JOINTS];

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { Serial1.println("Hard error!"); error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { Serial1.print("Soft error: "); Serial1.println(temp_rc); }}

QuickPID front_left_pid(& current_front_left_rads_sec, & front_left_output, & front_left_velocity_setpoint);

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void initialiseJointStateMessage(sensor_msgs__msg__JointState &msg)
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

  for (int i = 0; i < 2; i++) {
    name_data[i].data = name_buffer[i];
    name_data[i].size = 0;
    name_data[i].capacity = MAX_NAME_LEN;
    name_buffer[i][0] = '\0';
  }

  name_data[0].data = (char *)"front_left_joint";
  name_data[0].size = strlen(name_data[0].data);

  name_data[1].data = (char *)"front_right_joint";
  name_data[1].size = strlen(name_data[1].data);
}

void sendSabertoothCommand(HardwareSerial &port, byte address, byte motor, int speed) {
  speed = constrain(speed, -127, 127);
  byte command = (motor == 1)
                   ? ((speed < 0) ? 1 : 0)
                   : ((speed < 0) ? 5 : 4);
  byte data = abs(speed);
  byte checksum = (address + command + data) & 0x7F;
  byte packet[] = { address, command, data, checksum };
  if (SERIAL_DEBUG) Serial1.println(command);
  port.write(packet, sizeof(packet));
}

void subscription_callback(const void *msgin) {
  const sensor_msgs__msg__JointState * joint_msg = (const sensor_msgs__msg__JointState *)msgin;

  for (size_t i = 0; i < joint_msg->name.size && i < MAX_JOINTS; i++) {
    const char* name = joint_msg->name.data[i].data;
    float velocity = joint_msg->velocity.data[i];

    if (strcmp(name, "front_left_joint") == 0) {
      front_left_velocity_setpoint = velocity;
      if (SERIAL_DEBUG) {
        Serial1.print("Setpoint FL: ");
        Serial1.println(front_left_velocity_setpoint);

      }
    } else if (strcmp(name, "front_right_joint") == 0) {
      front_right_velocity_setpoint = velocity;
      if (SERIAL_DEBUG) {
        Serial1.print("Setpoint FR: ");
        Serial1.println(front_right_velocity_setpoint);
      }
    }
  }
}

void sampleEncoders() {
  unsigned long now = millis();
  long currentFL = frontLeftEncoder.read();
  long currentFR = frontRightEncoder.read();

  long deltaFL = currentFL - frontLeftLastTicks;
  long deltaFR = currentFR - frontRightLastTicks;

  float flTicksSec = deltaFL / ((now - lastEncoderSampleTime) / 1000.0);
  float frTicksSec = deltaFR / ((now - lastEncoderSampleTime) / 1000.0);

  current_front_left_rads_sec = (flTicksSec / (PULSES_PER_REVOLUTION * GEAR_RATIO_MULTIPLIER)) * -1;
  current_front_right_rads_sec = frTicksSec / (PULSES_PER_REVOLUTION * GEAR_RATIO_MULTIPLIER);

  frontLeftLastTicks = currentFL;
  frontRightLastTicks = currentFR;
  lastEncoderSampleTime = now;

  if (SERIAL_DEBUG) {
    Serial1.print("FL: ");
    Serial1.print(current_front_left_rads_sec);
    Serial1.print(" rad/s, FR: ");
    Serial1.print(current_front_right_rads_sec);
    Serial1.println(" rad/s");
  }
}

void publish_joint_state_message() {
  feedback_msg.velocity.data[0] = current_front_left_rads_sec;
  feedback_msg.velocity.data[1] = current_front_right_rads_sec;

  RCSOFTCHECK(rcl_publish(&feedback_publisher, &feedback_msg, NULL));
}

void setup() {
  Serial2.begin(9600);  // Sabertooth
  if (SERIAL_DEBUG) Serial1.begin(115200);

  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);

  front_left_pid.SetOutputLimits(-127.0, 127.0);
  front_left_pid.SetTunings(6.0, 0.01, 0.0);
  front_left_pid.SetSampleTimeUs(100000);
  front_left_pid.SetMode(QuickPID::Control::automatic);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_joint_node", "", &support));

  sensor_msgs__msg__JointState__init(&cmd_msg);
  sensor_msgs__msg__JointState__init(&feedback_msg);

  initialiseJointStateMessage(cmd_msg);
  initialiseJointStateMessage(feedback_msg);

  const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);

  rcl_subscription_options_t sub_ops = rcl_subscription_get_default_options();
  sub_ops.qos = rmw_qos_profile_sensor_data;

  RCCHECK(rcl_subscription_init(
    &joint_state_subscriber,
    &node,
    type_support,
    "platform/motors/cmd",
    &sub_ops));

  rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
  pub_ops.qos = rmw_qos_profile_sensor_data;

  RCCHECK(rcl_publisher_init(
    &feedback_publisher,
    &node,
    type_support,
    "platform/motors/feedback",
    &pub_ops));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &joint_state_subscriber, &cmd_msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  unsigned long now = millis();
  if (now - lastEncoderSampleTime >= 100) {
    sampleEncoders();
    publish_joint_state_message();
    front_left_pid.Compute();
    update_motors();
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void update_motors()
{
  if(SERIAL_DEBUG) 
  {
    Serial1.print("Front Left PID Output: ");
    Serial1.println(front_left_output);
  }
  sendSabertoothCommand(Serial2, SABERTOOTH_ADDR, 1, front_left_output);
}



