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

// status_led.cpp
#include "status_led.hpp"
#include <Arduino.h>

namespace
{
constexpr int kLedPin = LED_BUILTIN;

unsigned long last_change_ms = 0;
int step = 0;

void set_led(bool on)
{
  digitalWrite(kLedPin, on ? HIGH : LOW);
}
} // namespace

void status_led_setup()
{
  pinMode(kLedPin, OUTPUT);
  set_led(false);
}

void status_led_update(LedStatus status)
{
  const unsigned long now = millis();

  if (status == LedStatus::BatteryCritical) {
    // 3 quick blinks, pause, repeat.
    constexpr unsigned long pattern[] = {120, 120, 120, 120, 120, 700};

    if (now - last_change_ms >= pattern[step]) {
      last_change_ms = now;
      step = (step + 1) % 6;
      set_led(step % 2 == 1);
    }
    return;
  }

  if (status == LedStatus::RosConnected) {
    // Double blink, pause.
    constexpr unsigned long pattern[] = {100, 100, 100, 900};

    if (now - last_change_ms >= pattern[step]) {
      last_change_ms = now;
      step = (step + 1) % 4;
      set_led(step % 2 == 1);
    }
    return;
  }

  // Normal heartbeat.
  if (now - last_change_ms >= 1000) {
    last_change_ms = now;
    step = !step;
    set_led(step);
  }
}
