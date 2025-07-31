#pragma once

#include <Arduino.h>

#define LED_PIN 13

void error_loop();

// Macro helpers for micro-ROS error handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { Serial1.println("Hard error!"); error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { Serial1.print("Soft error: "); Serial1.println(temp_rc); }}
