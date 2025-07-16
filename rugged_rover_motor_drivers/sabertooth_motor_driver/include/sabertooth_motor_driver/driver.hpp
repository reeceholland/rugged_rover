
#ifndef SABERTOOTH_MOTOR_DRIVER__DRIVER_H
#define SABERTOOTH_MOTOR_DRIVER__DRIVER_H

#include "rclcpp/logger.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <termios.h>
#include <vector>

namespace sabertooth_motor_driver
{

  struct JointFeedback
  {
    double front_left_velocity_rad_s;
    double front_left_position_rad;
    double front_right_velocity_rad_s;
    double front_right_position_rad;
  };

  class Driver
  {
  public:
    explicit Driver(const std::string& serial_port, const rclcpp::Logger& logger,
                    int baudrate = 115200);
    ~Driver();

    bool sendVelocityCommand(double left_rad_s, double right_rad_s);
    std::optional<JointFeedback> requestFeedback();
    static uint8_t computeChecksum(const std::vector<uint8_t>& data);
    static void packInt16LE(std::vector<uint8_t>& buf, int16_t val);

  private:
    rclcpp::Logger logger_;
    int fd_;
    struct termios tty_;
    std::string port_;
    int baudrate_;

    static constexpr uint8_t START_BYTE = 0xFF;
    static constexpr uint8_t CMD_VELOCITY_RAD = 0x02;
    static constexpr uint8_t CMD_FEEDBACK_REQUEST = 0x03;
    static constexpr uint8_t CMD_FEEDBACK_RESPONSE = 0x04;
    static constexpr uint8_t CMD_FEEDBACK_RESPONSE_SIZE = 12;
    static constexpr int SCALE = 1000; // rad/s scaled to int16_t

    bool openPort();
    void closePort();
  };

} // namespace sabertooth_motor_driver

#endif // SABERTOOTH_MOTOR_DRIVER__DRIVER_H