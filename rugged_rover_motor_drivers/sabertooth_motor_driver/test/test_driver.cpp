#include "sabertooth_motor_driver/driver.hpp"
#include <gtest/gtest.h>

using sabertooth_motor_driver::Driver;

TEST(DriverTests, ComputeChecksumWorks)
{
  std::vector<uint8_t> data = {0xFF, 0x06, 0x01, 0x00, 0x00};
  uint8_t checksum = Driver::computeChecksum(data);
  EXPECT_EQ(checksum, 0xF8);
}

TEST(DriverTests, PacketPackingLE)
{
  std::vector<uint8_t> buffer;
  Driver::packInt16LE(buffer, 0x1234);
  ASSERT_EQ(buffer.size(), 2);
  EXPECT_EQ(buffer[0], 0x34);
  EXPECT_EQ(buffer[1], 0x12);
}