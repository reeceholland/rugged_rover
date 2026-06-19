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

#pragma once

#include <Encoder.h>

extern volatile long leftSideLastTicks;
extern volatile long rightSideLastTicks;
extern float current_left_side_position_rad;
extern float current_right_side_position_rad;
extern float current_left_side_rads_sec;
extern float current_right_side_rads_sec;
extern unsigned long lastEncoderSampleTime;

//  Sample the encoders and calculate the current speed in radians per second.
void sample_encoders();
