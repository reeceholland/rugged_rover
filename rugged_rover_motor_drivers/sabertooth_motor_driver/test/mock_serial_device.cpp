#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

constexpr uint8_t START_BYTE = 0xFF;
constexpr uint8_t PACKET_TYPE_VELOCITY = 0x02;
constexpr uint8_t PACKET_TYPE_FEEDBACK_REQUEST = 0x03;
constexpr uint8_t PACKET_TYPE_FEEDBACK_RESPONSE = 0x04;

constexpr size_t VELOCITY_PACKET_LENGTH = 8;
constexpr size_t FEEDBACK_RESPONSE_LENGTH = 12;
constexpr float SCALE = 1000.0f;

float simulatedLeftRadSec = 2.5;
float simulatedRightRadSec = 2.5;

/**
 * @brief Computes the checksum for a given data vector.
 *
 * This function calculates the checksum by XORing all bytes in the data vector
 *
 * @param data The vector of bytes to compute the checksum for.
 *
 * @return The computed checksum byte.
 */
uint8_t computeChecksum(const std::vector<uint8_t>& data)
{
  uint8_t chk = 0;
  for (size_t i = 0; i < data.size() - 1; ++i)
    chk ^= data[i];
  return chk;
}

/**
 * @brief Sends a feedback response packet over the serial port.
 *
 * This function constructs a feedback response packet with the current simulated
 * wheel velocities and sends it to the specified file descriptor.
 *
 * @param fd The file descriptor of the serial port to send the packet to.
 */
void sendFeedbackResponse(int fd)
{
  int16_t left = static_cast<int16_t>(simulatedLeftRadSec * SCALE);
  int16_t right = static_cast<int16_t>(simulatedRightRadSec * SCALE);

  std::vector<uint8_t> packet;
  packet.push_back(START_BYTE);
  packet.push_back(FEEDBACK_RESPONSE_LENGTH);
  packet.push_back(PACKET_TYPE_FEEDBACK_RESPONSE);

  packet.push_back(left & 0xFF);
  packet.push_back((left >> 8) & 0xFF);
  packet.push_back(right & 0xFF);
  packet.push_back((right >> 8) & 0xFF);

  // Dummy position bytes
  for (int i = 0; i < 4; ++i)
    packet.push_back(0);

  uint8_t checksum = 0;
  for (size_t i = 0; i < packet.size(); ++i)
    checksum ^= packet[i];
  packet.push_back(checksum);

  write(fd, packet.data(), packet.size());
}

/**
 * @brief Main function for the Arduino emulator.
 *
 * This function sets up a pseudo-serial port and listens for incoming packets.
 * It processes velocity commands and sends feedback responses as needed.
 *
 * @return int Exit status.
 */
int main()
{

  int master_fd;
  char slave_name[100];

  // Open a pseudo-terminal master
  master_fd = posix_openpt(O_RDWR | O_NOCTTY);

  // Grant access to the slave side
  grantpt(master_fd);

  // Unlock the slave side
  unlockpt(master_fd);

  // Get the file path for the slave PTY
  ptsname_r(master_fd, slave_name, sizeof(slave_name));

  std::cout << "Arduino emulator listening on: " << slave_name << std::endl;
  std::cout << "Point your ROS node to this port.\n";

  std::vector<uint8_t> buffer;
  buffer.reserve(64);

  while (true)
  {

    uint8_t b;

    // Read a byte from the master file descriptor
    ssize_t n = read(master_fd, &b, 1);
    if (n <= 0)
      continue;

    // Add the byte to the buffer
    buffer.push_back(b);

    // Check for header and valid length
    if (buffer.size() == 2)
    {
      size_t len = buffer[1];
      if (len < 4 || len > 32)
      {
        buffer.clear();
      }
    }

    // Full packet received
    if (!buffer.empty() && buffer.size() == buffer[1])
    {
      if (buffer[0] != START_BYTE)
      {
        buffer.clear();
        continue;
      }

      // Checksum validation
      uint8_t expected = computeChecksum(buffer);
      if (expected != buffer.back())
      {
        std::cerr << "Checksum failed.\n";
        buffer.clear();
        continue;
      }
      // Process the packet based on its type
      uint8_t type = buffer[2];
      if (type == PACKET_TYPE_FEEDBACK_REQUEST)
      {
        sendFeedbackResponse(master_fd);
      }
      else if (type == PACKET_TYPE_VELOCITY)
      {
        int16_t l = buffer[3] | (buffer[4] << 8);
        int16_t r = buffer[5] | (buffer[6] << 8);
        simulatedLeftRadSec = l / SCALE;
        simulatedRightRadSec = r / SCALE;
        std::cout << "Velocity set: L=" << simulatedLeftRadSec
                  << " rad/s, R=" << simulatedRightRadSec << "\n";
      }
      // Clear the buffer after processing
      buffer.clear();
    }
  }

  close(master_fd);
  return 0;
}
