#pragma once

// Set this to 1 for normal micro-ROS operation.
// Set this to 0 for USB serial bench testing without running the ROS agent.
#define USE_ROS 1

extern bool SERIAL_DEBUG;

#define BATTERY_VOLTAGE_PIN A9

constexpr float BATTERY_R_TOP = 100000.0f;
constexpr float BATTERY_R_BOTTOM = 33000.0f;
constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
constexpr float BATTERY_PUBLISH_PERIOD_MS = 1000.0f;
