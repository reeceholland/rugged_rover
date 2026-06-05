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
  if (battery_voltage <= kCriticalVoltage)
  {
    critical_latched = true;
  }
  else if (battery_voltage >= kRecoverVoltage)
  {
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
