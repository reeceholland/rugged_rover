#include "sabertooth_motor_driver/driver.hpp"

#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "rclcpp/logging.hpp"

namespace sabertooth_motor_driver
{

  Driver::Driver(const std::string& port, const rclcpp::Logger& logger, int baudrate)
      : fd_(-1), port_(port), logger_(logger), baudrate_(baudrate)
  {
    openPort();
  }

  /**
   * @brief Destructor for the Driver class.
   */
  Driver::~Driver()
  {
    closePort();
  }

  /**
   * @brief Opens the serial port for communication.
   *
   * This function configures the serial port with the specified baud rate and settings.
   *
   * @return true if the port was opened successfully, false otherwise.
   */
  bool Driver::openPort()
  {

    // Open the serial port
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0)
    {
      RCLCPP_ERROR(logger_, "Failed to open port '%s': %s", port_.c_str(), strerror(errno));
      return false;
    }

    // Wait for the microcontroller to reset
    // This is necessary to ensure the device is ready for communication
    usleep(1000000); // 1 second delay

    // Configure the serial port settings
    // Clear the termios structure
    memset(&tty_, 0, sizeof tty_);
    if (tcgetattr(fd_, &tty_) != 0)
    {
      RCLCPP_ERROR(logger_, "Error from tcgetattr: %s", strerror(errno));
      return false;
    }

    // Set the baud rate
    cfsetospeed(&tty_, B115200);
    cfsetispeed(&tty_, B115200);

    // Configure the serial port attributes
    tty_.c_cflag = (tty_.c_cflag & ~CSIZE) | CS8;
    tty_.c_iflag = 0;
    tty_.c_oflag = 0;
    tty_.c_lflag = 0;
    tty_.c_cflag |= CLOCAL | CREAD;
    tty_.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty_.c_cc[VMIN] = 0;
    tty_.c_cc[VTIME] = 10;

    // Apply the settings to the serial port
    if (tcsetattr(fd_, TCSANOW, &tty_) != 0)
    {
      RCLCPP_ERROR(logger_, "Error from tcsetattr: %s", strerror(errno));
      return false;
    }

    return true;
  }
  /**
   * @brief Closes the serial port.
   *
   * This function closes the file descriptor associated with the serial port.
   *
   * @return void
   */
  void Driver::closePort()
  {
    if (fd_ >= 0)
    {
      close(fd_);
      fd_ = -1;
    }
  }

  /**
   * @brief Computes the checksum for a given data vector.
   *
   * This function calculates the checksum by XORing all bytes in the data vector.
   *
   * @param data The vector of bytes to compute the checksum for.
   * @return The computed checksum as a single byte.
   */
  uint8_t Driver::computeChecksum(const std::vector<uint8_t>& data)
  {
    uint8_t chk = 0;
    for (uint8_t b : data)
      chk ^= b;
    return chk;
  }

  /**
   * @brief Packs a 16-bit integer into a vector in little-endian format.
   *
   * This function appends the low byte and high byte of the integer to the buffer.
   *
   * @param buf The vector to append the bytes to.
   * @param val The 16-bit integer value to pack.
   */
  void Driver::packInt16LE(std::vector<uint8_t>& buf, int16_t val)
  {
    buf.push_back(static_cast<uint8_t>(val & 0xFF));
    buf.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
  }

  /**
   * @brief Sends a velocity command to the motor driver.
   *
   * This function sends a command to set the left and right wheel velocities.
   *
   * @param left_rad_s The desired velocity for the left wheel in radians per second.
   * @param right_rad_s The desired velocity for the right wheel in radians per second.
   * @return true if the command was sent successfully, false otherwise.
   */
  bool Driver::sendVelocityCommand(double left_rad_s, double right_rad_s)
  {
    if (fd_ < 0)
      return false;

    int16_t left_scaled = static_cast<int16_t>(left_rad_s * SCALE);
    int16_t right_scaled = static_cast<int16_t>(right_rad_s * SCALE);

    std::vector<uint8_t> packet = {START_BYTE,
                                   0x08, // length
                                   CMD_VELOCITY_RAD};

    packInt16LE(packet, left_scaled);
    packInt16LE(packet, right_scaled);

    uint8_t checksum = computeChecksum(packet);
    packet.push_back(checksum);

    ssize_t written = write(fd_, packet.data(), packet.size());
    RCLCPP_DEBUG(logger_, "Sent velocity command: FL: %.2f, FR: %.2f", left_rad_s, right_rad_s);
    return (written == static_cast<ssize_t>(packet.size()));
  }

  /**
   * @brief Requests feedback from the motor driver.
   *
   * This function sends a request for feedback and reads the response.
   *
   * @return An optional JointFeedback structure containing the feedback data, or std::nullopt on
   * failure.
   */
  std::optional<Feedback> Driver::requestFeedback()
  {
    if (fd_ < 0)
    {
      RCLCPP_ERROR(logger_, "Port not open for feedback request");
      return std::nullopt;
    }

    std::vector<uint8_t> packet = {START_BYTE,
                                   0x04, // length
                                   CMD_FEEDBACK_REQUEST};
    packet.push_back(computeChecksum(packet));

    RCLCPP_DEBUG(logger_, "Sending feedback request...");
    ssize_t written = write(fd_, packet.data(), packet.size());
    if (written != static_cast<ssize_t>(packet.size()))
    {
      RCLCPP_ERROR(logger_, "Failed to send feedback request");
      return std::nullopt;
    }

    uint8_t response[CMD_FEEDBACK_RESPONSE_SIZE];
    ssize_t bytes_read = read(fd_, response, CMD_FEEDBACK_RESPONSE_SIZE);
    RCLCPP_DEBUG(logger_, "Read %zd bytes from feedback response", bytes_read);

    if (bytes_read != CMD_FEEDBACK_RESPONSE_SIZE)
    {
      RCLCPP_ERROR(logger_, "Incomplete feedback response received");
      return std::nullopt;
    }

    if (response[0] != START_BYTE || response[2] != CMD_FEEDBACK_RESPONSE)
    {
      std::ostringstream oss;
      oss << "Invalid feedback response: ";
      for (size_t i = 0; i < CMD_FEEDBACK_RESPONSE_SIZE; ++i)
        oss << "0x" << std::hex << std::uppercase << static_cast<int>(response[i]) << " ";
      RCLCPP_ERROR(logger_, "%s", oss.str().c_str());
      return std::nullopt;
    }

    uint8_t received_checksum = response[CMD_FEEDBACK_RESPONSE_SIZE - 1];
    uint8_t computed_checksum =
        computeChecksum(std::vector<uint8_t>(response, response + CMD_FEEDBACK_RESPONSE_SIZE - 1));
    if (received_checksum != computed_checksum)
    {
      RCLCPP_ERROR(logger_, "Checksum mismatch in feedback response");
      return std::nullopt;
    }

    auto parseInt16LE = [](uint8_t lo, uint8_t hi) -> int16_t
    { return static_cast<int16_t>((hi << 8) | lo); };

    Feedback feedback;
    feedback.front_left_velocity_rad_s =
        parseInt16LE(response[3], response[4]) / static_cast<double>(SCALE);
    feedback.front_left_position_rad =
        parseInt16LE(response[5], response[6]) / static_cast<double>(SCALE);
    feedback.front_right_velocity_rad_s =
        parseInt16LE(response[7], response[8]) / static_cast<double>(SCALE);
    feedback.front_right_position_rad =
        parseInt16LE(response[9], response[10]) / static_cast<double>(SCALE);
    feedback.battery_voltage_mv = (response[11] | (response[12] << 8));

    return feedback;
  }

} // namespace sabertooth_motor_driver
