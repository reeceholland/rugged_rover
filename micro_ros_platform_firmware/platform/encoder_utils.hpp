#pragma once

#include <Encoder.h>

extern volatile long frontLeftLastTicks;
extern volatile long frontRightLastTicks;
extern float current_front_left_rads_sec;
extern float current_front_right_rads_sec;
extern unsigned long lastEncoderSampleTime;

//  Sample the encoders and calculate the current speed in radians per second.
void sample_encoders();
