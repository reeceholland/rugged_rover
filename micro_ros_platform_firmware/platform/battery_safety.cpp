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

#include "battery_safety.hpp"

namespace
{
constexpr float kCriticalVoltage = 10.5f;
constexpr float kRecoverVoltage = 11.1f;

bool critical_latched = false;
} // namespace

void battery_safety_setup()
{
  critical_latched = false;
}

void battery_safety_update(float battery_voltage)
{
  if (battery_voltage <= kCriticalVoltage) {
    critical_latched = true;
  } else if (battery_voltage >= kRecoverVoltage) {
    critical_latched = false;
  }
}

bool battery_is_critical()
{
  return critical_latched;
}

void battery_clear_latch_for_debug()
{
  critical_latched = false;
}
