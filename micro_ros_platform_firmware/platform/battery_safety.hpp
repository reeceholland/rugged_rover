#pragma once

void battery_safety_setup();
void battery_safety_update(float battery_voltage);

bool battery_is_critical();
void battery_clear_latch_for_debug();