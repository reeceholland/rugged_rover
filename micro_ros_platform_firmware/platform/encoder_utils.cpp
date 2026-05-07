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

  //  Read the current encoder values
  long currentFL = frontLeftEncoder.read();
  long currentFR = frontRightEncoder.read();

  //  Calculate the change in ticks since the last sample
  long deltaFL = currentFL - frontLeftLastTicks;
  long deltaFR = currentFR - frontRightLastTicks;

  //  Calculate the speed in ticks per second
  float flTicksSec = deltaFL / ((now - lastEncoderSampleTime) / 1000.0);
  float frTicksSec = deltaFR / ((now - lastEncoderSampleTime) / 1000.0);

  //  Convert ticks per second to radians per second
  //  The gear ratio multiplier is used to adjust the ticks based on the gear
  //  ratio of the motors
  current_front_left_rads_sec = (flTicksSec / (PULSES_PER_REVOLUTION * GEAR_RATIO_MULTIPLIER)) * -1;
  current_front_right_rads_sec = frTicksSec / (PULSES_PER_REVOLUTION * GEAR_RATIO_MULTIPLIER);

  //  Update the last ticks for the next sample
  frontLeftLastTicks = currentFL;
  frontRightLastTicks = currentFR;

  //  Update the last sample time
  lastEncoderSampleTime = now;

  if (SERIAL_DEBUG)
  {
    Serial1.print("FL: ");
    Serial1.print(current_front_left_rads_sec);
    Serial1.print(" rad/s, FR: ");
    Serial1.print(current_front_right_rads_sec);
    Serial1.println(" rad/s");
  }
}
