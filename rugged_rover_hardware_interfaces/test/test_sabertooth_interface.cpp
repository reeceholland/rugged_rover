#include "rugged_rover_hardware_interfaces/sabertooth/sabertooth_system_interface.hpp"
#include "rugged_rover_interfaces/msg/rover_feedback.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using rugged_rover_hardware_interfaces::sabertooth::SabertoothSystemInterface;
using rugged_rover_interfaces::msg::RoverFeedback;

namespace rugged_rover_hardware_interfaces::sabertooth
{

  class SabertoothInterfaceTest : public ::testing::Test
  {
  protected:
    void SetUp() override
    {
      interface_ = std::make_shared<SabertoothSystemInterface>();

      // Simulate initialization
      hardware_interface::HardwareInfo info;
      hardware_interface::ComponentInfo joint;
      joint.name = "left_front_joint";
      info.joints.push_back(joint);
      joint.name = "right_front_joint";
      info.joints.push_back(joint);

      ASSERT_EQ(interface_->on_init(info), hardware_interface::CallbackReturn::SUCCESS);

      interface_->joint_names_ = {"left_front_joint", "right_front_joint"};
      interface_->hw_positions_ = {0.0, 0.0};
      interface_->hw_velocities_ = {0.0, 0.0};

      // Prepare dummy feedback
      RoverFeedback feedback;
      feedback.joint_state.name = {"left_front_joint", "right_front_joint"};
      feedback.joint_state.position = {1.23, 4.56};
      feedback.joint_state.velocity = {7.89, 0.12};

      {
        std::lock_guard<std::mutex> lock(interface_->feedback_mutex_);
        interface_->last_feedback_ = feedback;
      }
    }

    std::shared_ptr<SabertoothSystemInterface> interface_;
  };

  TEST_F(SabertoothInterfaceTest, ReadUpdatesJointStatesCorrectly)
  {
    auto ret = interface_->read(rclcpp::Time(0), rclcpp::Duration(0, 0));
    EXPECT_EQ(ret, hardware_interface::return_type::OK);

    EXPECT_DOUBLE_EQ(interface_->get_hw_positions()[0], 1.23);
    EXPECT_DOUBLE_EQ(interface_->get_hw_positions()[1], 4.56);
    EXPECT_DOUBLE_EQ(interface_->get_hw_velocities()[0], 7.89);
    EXPECT_DOUBLE_EQ(interface_->get_hw_velocities()[1], 0.12);
  }

  TEST_F(SabertoothInterfaceTest, ExportsStateInterfaces)
  {
    auto states = interface_->export_state_interfaces();

    ASSERT_EQ(states.size(), 5);

    EXPECT_EQ(states[0].get_prefix_name(), "left_front_joint");
    EXPECT_EQ(states[0].get_interface_name(), hardware_interface::HW_IF_POSITION);

    EXPECT_EQ(states[4].get_prefix_name(), "battery");
    EXPECT_EQ(states[4].get_interface_name(), "voltage_mv");
  }

  TEST_F(SabertoothInterfaceTest, ExportsCommandInterfaces)
  {
    auto commands = interface_->export_command_interfaces();

    ASSERT_EQ(commands.size(), 2);

    EXPECT_EQ(commands[0].get_prefix_name(), "left_front_joint");
    EXPECT_EQ(commands[0].get_interface_name(), hardware_interface::HW_IF_VELOCITY);

    EXPECT_EQ(commands[1].get_prefix_name(), "right_front_joint");
    EXPECT_EQ(commands[1].get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  }

} // namespace rugged_rover_hardware_interfaces::sabertooth
