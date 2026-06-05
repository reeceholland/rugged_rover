// status_led.hpp
#pragma once

enum class LedStatus
{
  Normal,
  RosConnected,
  BatteryCritical,
};

void status_led_setup();
void status_led_update(LedStatus status);