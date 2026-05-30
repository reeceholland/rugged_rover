// battery_monitor.cpp
#include "battery_monitor.hpp"
#include "config.hpp"
#include <Arduino.h>

void setup_battery_monitor()
{
  analogReadResolution(12);
}

float read_battery_voltage()
{
  constexpr float ADC_MAX = 4095.0f;

  const int raw = analogRead(BATTERY_VOLTAGE_PIN);

  const float pin_voltage = (static_cast<float>(raw) / ADC_MAX) * ADC_REFERENCE_VOLTAGE;

  return pin_voltage * (BATTERY_R_TOP + BATTERY_R_BOTTOM) / BATTERY_R_BOTTOM;
}