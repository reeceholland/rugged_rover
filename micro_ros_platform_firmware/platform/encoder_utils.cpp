#include "encoder_utils.hpp"
#include "config.hpp"
#include "motor_control.hpp"

//  Encoder instances for front left and front right motors
Encoder frontLeftEncoder(2, 3);
Encoder frontRightEncoder(4, 5);

volatile long frontLeftLastTicks = 0;
volatile long frontRightLastTicks = 0;
float current_front_left_rads_sec = 0;
float current_front_right_rads_sec = 0;
unsigned long lastEncoderSampleTime = 0;

void sample_encoders() {
  unsigned long now = millis();
  long currentFL = frontLeftEncoder.read();
  long currentFR = frontRightEncoder.read();

  long deltaFL = currentFL - frontLeftLastTicks;
  long deltaFR = currentFR - frontRightLastTicks;

  float flTicksSec = deltaFL / ((now - lastEncoderSampleTime) / 1000.0);
  float frTicksSec = deltaFR / ((now - lastEncoderSampleTime) / 1000.0);

  current_front_left_rads_sec =
      (flTicksSec / (PULSES_PER_REVOLUTION * GEAR_RATIO_MULTIPLIER)) * -1;
  current_front_right_rads_sec =
      frTicksSec / (PULSES_PER_REVOLUTION * GEAR_RATIO_MULTIPLIER);

  frontLeftLastTicks = currentFL;
  frontRightLastTicks = currentFR;
  lastEncoderSampleTime = now;

  if (SERIAL_DEBUG) {
    Serial1.print("FL: ");
    Serial1.print(current_front_left_rads_sec);
    Serial1.print(" rad/s, FR: ");
    Serial1.print(current_front_right_rads_sec);
    Serial1.println(" rad/s");
  }
}
