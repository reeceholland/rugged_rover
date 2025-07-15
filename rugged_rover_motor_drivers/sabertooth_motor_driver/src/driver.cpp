#include "sabertooth_motor_driver/driver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <cstring>

#include "rclcpp/logging.hpp"

namespace sabertooth_motor_driver {

Driver::Driver(const std::string& port, const rclcpp::Logger& logger, int baudrate)
: fd_(-1), port_(port), baudrate_(baudrate), logger_(logger) {
  openPort();
}

/**
 * @brief Destructor for the Driver class.
 */
Driver::~Driver() {
  closePort();
}

/**
 * 
 * @brief Opens the serial port for communication with microcontroller.
 * 
 * This function sets up the serial port with the specified baud rate and
 * configures the terminal attributes for communication.
 */
bool Driver::openPort() {

  // Open a serial port in read/write mode, no controlling terminal, synchronous I/O
  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

  // Check if the port was opened successfully
  if (fd_ < 0) {
    RCLCPP_ERROR(logger_, "Failed to open port '%s': %s", port_.c_str(), strerror(errno));
    return false;
  }

  // Clear the tty structure
  memset(&tty_, 0, sizeof tty_);
  if (tcgetattr(fd_, &tty_) != 0) {
    RCLCPP_ERROR(logger_,"Error from tcgetattr: %s", strerror(errno));
    return false;
  }

  // Set the baud rate
  cfsetospeed(&tty_, B115200);
  cfsetispeed(&tty_, B115200);

  // Configure the tty structure
  tty_.c_cflag = (tty_.c_cflag & ~CSIZE) | CS8; // 8 data bits
  tty_.c_iflag = tty_.c_oflag = tty_.c_lflag = 0; // Set into raw mode
  tty_.c_cflag |= CLOCAL | CREAD;
  tty_.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS); // No parity, 1 stop bit, no flow control

  if (tcsetattr(fd_, TCSANOW, &tty_) != 0) {
    RCLCPP_ERROR(logger_,"Error from tcgetattr: %s", strerror(errno));
    return false;
  }

  return true;
}

/**
 * @brief Closes the serial port.
 * 
 * This function closes the file descriptor for the serial port if it is open.
 */
void Driver::closePort() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

/**
 * @brief Computes the checksum for a given data vector.
 * 
 * This function calculates the checksum by XORing all bytes in the data vector.
 * 
 * @param data The data vector for which to compute the checksum.
 * @return The computed checksum byte.
 */
uint8_t Driver::computeChecksum(const std::vector<uint8_t>& data) {
    uint8_t chk = 0;
    for (auto b : data) chk ^= b;
    return chk;
}

/**
 * @brief Packs a 16-bit integer into a vector in little-endian format.
 * 
 * This function appends the low byte and high byte of the integer to the vector.
 * 
 * @param buf The vector to which the bytes will be appended.
 * @param val The 16-bit integer value to pack.
 */
void Driver::packInt16LE(std::vector<uint8_t>& buf, int16_t val) {
  buf.push_back(val & 0xFF);
  buf.push_back((val >> 8) & 0xFF);
}

/**
 * @brief Sends a velocity command to the microcontroller.
 * 
 * This function constructs a command packet with the specified left and right wheel velocities,
 * computes the checksum, and sends the packet over the serial port.
 * 
 * @param left_rad_s The desired velocity for the left wheel in radians per second.
 * @param right_rad_s The desired velocity for the right wheel in radians per second.
 * @return True if the command was sent successfully, false otherwise.
 */
bool Driver::sendVelocityCommand(double left_rad_s, double right_rad_s) {

  // Check if the file descriptor is valid
  if (fd_ < 0) return false;

  // Scale the velocities to int16_t
  int16_t left_scaled = static_cast<int16_t>(left_rad_s * SCALE);
  int16_t right_scaled = static_cast<int16_t>(right_rad_s * SCALE);

  // Create the command packet
  std::vector<uint8_t> packet;
  packet.push_back(START_BYTE);
  packet.push_back(0x08);  // length
  packet.push_back(CMD_VELOCITY_RAD);
  packInt16LE(packet, left_scaled);
  packInt16LE(packet, right_scaled);

  // Compute the checksum and append it to the packet
  uint8_t checksum = computeChecksum(packet);
  packet.push_back(checksum);

  // Write thee packet to the serial port
  ssize_t written = write(fd_, packet.data(), packet.size());

  // Return write success status
  return (written == static_cast<ssize_t>(packet.size()));
}

std::optional<JointFeedback> Driver::requestFeedback() {

  // Check if the file descriptor is valid
  if (fd_ <0 ) {
    RCLCPP_ERROR(logger_, "Port not open for feedback request");
    return std::nullopt;
  }

  // Create the feedback request packet
  // The packet format is: [START_BYTE, LENGTH, CMD_FEEDBACK_REQUEST, CHECKSUM
  std::vector<uint8_t> packet;
  packet.push_back(START_BYTE);
  packet.push_back(0x04);  // length
  packet.push_back(CMD_FEEDBACK_REQUEST);
  uint8_t checksum = computeChecksum(packet);
  packet.push_back(checksum);

  // Write the packet to the serial port
  ssize_t written = write(fd_, packet.data(), packet.size());
  if (written != static_cast<ssize_t>(packet.size())) {
    RCLCPP_ERROR(logger_, "Failed to send feedback request");
    return std::nullopt;
  }

  // Read the response from the serial port
  // Expecting a response of size CMD_FEEDBACK_RESPONSE_SIZE
  uint8_t response[CMD_FEEDBACK_RESPONSE_SIZE]; 
  ssize_t bytes_read = read(fd_, response, CMD_FEEDBACK_RESPONSE_SIZE);

  // Check if the read was successful and the response is complete
  if (bytes_read != CMD_FEEDBACK_RESPONSE_SIZE) {
    RCLCPP_ERROR(logger_, "Incomplete feedback response received");
    return std::nullopt;
  }

  // Validate the response format
  if (response[0] != START_BYTE || response[2] != CMD_FEEDBACK_RESPONSE) {
  std::ostringstream oss;
  oss << "Invalid feedback response format. Received: ";
  for (size_t i = 0; i < CMD_FEEDBACK_RESPONSE_SIZE; ++i) {
    oss << "0x" << std::hex << std::uppercase << static_cast<int>(response[i]) << " ";
    }
  RCLCPP_ERROR(logger_, "%s", oss.str().c_str());
  return std::nullopt;
  }

  // TODO(reece): Validate checksum if needed
  uint8_t received_checksum = response[CMD_FEEDBACK_RESPONSE_SIZE - 1];
  uint8_t computed_checksum = computeChecksum(std::vector<uint8_t>(response, response + CMD_FEEDBACK_RESPONSE_SIZE - 1));
  if (received_checksum != computed_checksum) { 
    RCLCPP_ERROR(logger_, "Checksum mismatch in feedback response");
    return std::nullopt;
  }

  auto parseInt16LE = [](uint8_t lo, uint8_t hi) -> int16_t {
    return static_cast<int16_t>((hi << 8) | lo);
  };

  JointFeedback feedback;
  feedback.front_left_velocity_rad_s = parseInt16LE(response[3], response[4]) / static_cast<double>(SCALE);
  feedback.front_left_position_rad = parseInt16LE(response[5], response[6]) / static_cast<double>(SCALE);
  feedback.front_right_velocity_rad_s = parseInt16LE(response[7], response[8]) / static_cast<double>(SCALE);
  feedback.front_right_position_rad = parseInt16LE(response[9], response[10]) / static_cast<double>(SCALE);

  return feedback;
}

}