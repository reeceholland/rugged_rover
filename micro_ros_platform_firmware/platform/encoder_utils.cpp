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

#include "encoder_utils.hpp"
#include "config.hpp"
#include "motor_control.hpp"

namespace
{

constexpr float VELOCITY_FILTER_ALPHA = 0.35;

} // namespace

//  Encoder instances for rear left and rear right motors
Encoder rearLeftEncoder(2, 3);
Encoder rearRightEncoder(4, 5);

volatile long rearLeftLastTicks = 0;
volatile long rearRightLastTicks = 0;
float current_rear_left_position_rad = 0;
float current_rear_right_position_rad = 0;
float current_rear_left_rads_sec = 0;
float current_rear_right_rads_sec = 0;
unsigned long lastEncoderSampleTime = 0;

/**
 * @brief  Sample the encoders and calculate the current speed in radians per
 * second.
 *
 * This function reads the encoder values, calculates the change in ticks since
 * the last sample, and converts that to radians per second based on the defined
 * gear ratio and pulses per revolution.
 */
void sample_encoders()
{
  unsigned long now = millis();
  unsigned long deltaTimeMs = now - lastEncoderSampleTime;

  if (deltaTimeMs == 0) {
    return;
  }

  //  Read the current encoder values
  long currentRL = rearLeftEncoder.read();
  long currentRR = rearRightEncoder.read();

  current_rear_left_position_rad =
    (currentRL / (ENCODER_COUNTS_PER_MOTOR_REV * GEAR_RATIO_MULTIPLIER)) * TWO_PI * -1;
  current_rear_right_position_rad =
    (currentRR / (ENCODER_COUNTS_PER_MOTOR_REV * GEAR_RATIO_MULTIPLIER)) * TWO_PI;

  //  Calculate the change in ticks since the last sample
  long deltaRL = currentRL - rearLeftLastTicks;
  long deltaRR = currentRR - rearRightLastTicks;

  //  Calculate the speed in ticks per second
  float rlTicksSec = deltaRL / (deltaTimeMs / 1000.0);
  float rrTicksSec = deltaRR / (deltaTimeMs / 1000.0);

  //  Convert ticks per second to radians per second
  //  The gear ratio multiplier is used to adjust the ticks based on the gear
  //  ratio of the motors
  const float raw_rear_left_rads_sec =
    (rlTicksSec / (ENCODER_COUNTS_PER_MOTOR_REV * GEAR_RATIO_MULTIPLIER)) * TWO_PI * -1;
  const float raw_rear_right_rads_sec =
    (rrTicksSec / (ENCODER_COUNTS_PER_MOTOR_REV * GEAR_RATIO_MULTIPLIER)) * TWO_PI;

  current_rear_left_rads_sec +=
    VELOCITY_FILTER_ALPHA * (raw_rear_left_rads_sec - current_rear_left_rads_sec);
  current_rear_right_rads_sec +=
    VELOCITY_FILTER_ALPHA * (raw_rear_right_rads_sec - current_rear_right_rads_sec);

  //  Update the last ticks for the next sample
  rearLeftLastTicks = currentRL;
  rearRightLastTicks = currentRR;

  //  Update the last sample time
  lastEncoderSampleTime = now;

  if (SERIAL_DEBUG) {
    Serial.print("dt: ");
    Serial.print(deltaTimeMs);
    Serial.print(" ms, deltaRL: ");
    Serial.print(deltaRL);
    Serial.print(", deltaRR: ");
    Serial.print(deltaRR);
    Serial.print(" | ");
    Serial.print("RL: ");
    Serial.print(current_rear_left_rads_sec);
    Serial.print(" rad/s, RR: ");
    Serial.print(current_rear_right_rads_sec);
    Serial.println(" rad/s");
  }
}
