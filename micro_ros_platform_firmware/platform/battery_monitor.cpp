// Copyright 2026 Reece Holland
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
