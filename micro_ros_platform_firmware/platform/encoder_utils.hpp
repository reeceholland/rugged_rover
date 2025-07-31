#pragma once

#include <Encoder.h>

extern volatile long frontLeftLastTicks;
extern volatile long frontRightLastTicks;
extern float current_front_left_rads_sec;
extern float current_front_right_rads_sec;
extern unsigned long lastEncoderSampleTime;

void sample_encoders();
