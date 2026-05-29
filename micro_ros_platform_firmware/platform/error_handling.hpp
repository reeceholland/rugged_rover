#pragma once

#include <Arduino.h>

#define LED_PIN 13

void error_loop();

inline void report_micro_ros_error(const char* message, int rc = 0)
{
  // Do not print on Serial1 here: Serial1 is the micro-ROS transport to the Pi.
  // The LED blink in error_loop() is the hardware-visible failure indicator.
  (void)message;
  (void)rc;
}

// Macro helpers for micro-ROS error handling
#define RCCHECK(fn)                                                                                \
  {                                                                                                \
    rcl_ret_t temp_rc = fn;                                                                        \
    if ((temp_rc != RCL_RET_OK))                                                                   \
    {                                                                                              \
      report_micro_ros_error("Hard error", temp_rc);                                               \
      error_loop();                                                                                \
    }                                                                                              \
  }
#define RCSOFTCHECK(fn)                                                                            \
  {                                                                                                \
    rcl_ret_t temp_rc = fn;                                                                        \
    if ((temp_rc != RCL_RET_OK))                                                                   \
    {                                                                                              \
      report_micro_ros_error("Soft error", temp_rc);                                               \
    }                                                                                              \
  }
