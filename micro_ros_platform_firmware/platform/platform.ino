#include <Arduino.h>
#include "ros_interface.hpp"
#include "encoder_utils.hpp"
#include "motor_control.hpp"
#include "msg_utils.hpp"
#include "config.hpp"
bool SERIAL_DEBUG = true;

void setup() {
  ros_setup();
}

void loop() {
  setup_pid();
  unsigned long now = millis();
  if (now - lastEncoderSampleTime >= 100) {
    
    sample_encoders();
    publish_joint_state_message();
    update_motors();
  }
  spin_ros_executor();
}