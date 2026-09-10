// Copyright 2026 Reece Holland
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rugged_rover_hardware_interfaces/sabertooth/sabertooth_system_interface.hpp"
#include "rugged_rover_interfaces/msg/rover_feedback.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using rugged_rover_hardware_interfaces::sabertooth::SabertoothSystemInterface;
using sensor_msgs::msg::JointState;

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
    joint.name = "front_left_joint";
    info.joints.push_back(joint);
    joint.name = "front_right_joint";
    info.joints.push_back(joint);

    ASSERT_EQ(interface_->on_init(info), hardware_interface::CallbackReturn::SUCCESS);

    interface_->joint_names_ = {"front_left_joint", "front_right_joint"};
    interface_->hw_positions_ = {0.0, 0.0};
    interface_->hw_velocities_ = {0.0, 0.0};

      // Prepare dummy feedback
    JointState feedback;
    feedback.name = {"front_left_joint", "front_right_joint"};
    feedback.position = {1.23, 4.56};
    feedback.velocity = {7.89, 0.12};

    {
      std::lock_guard<std::mutex> lock(interface_->feedback_mutex_);
      interface_->last_feedback_ = feedback;
      interface_->has_feedback_ = true;
      interface_->feedback_received_ns_ = interface_->steady_ns();
    }
  }


  void exercise_ros_stopping()
  {
    rclcpp::init(0, nullptr);
    auto peer = rclcpp::Node::make_shared("stop_test_peer");
    auto feedback = peer->create_publisher<JointState>(
      "/platform/motors/feedback", rclcpp::SensorDataQoS());
    auto battery = peer->create_publisher<std_msgs::msg::Bool>(
      "/platform/battery/is_critical", 10);
    auto enable = peer->create_publisher<std_msgs::msg::Bool>("/rover/motors_enabled", 1);
    std::vector<double> observed;
    auto commands = peer->create_subscription<JointState>(
      "/platform/motors/cmd", rclcpp::SensorDataQoS(),
      [&observed](const JointState::SharedPtr msg) {observed = msg->velocity;});
    interface_->on_activate(rclcpp_lifecycle::State());
    interface_->hw_commands_ = {1.0, 1.0};
    auto pump = [&](bool enabled, bool send_enable, bool send_feedback, double seconds) {
        const auto until = std::chrono::steady_clock::now() +
          std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < until) {
          JointState fb;
          fb.name = {"front_left_joint", "front_right_joint"};
          fb.position = {0.0, 0.0};
          fb.velocity = {0.0, 0.0};
          if (send_feedback) {feedback->publish(fb);}
          std_msgs::msg::Bool battery_msg;
          battery_msg.data = false;
          battery->publish(battery_msg);
          std_msgs::msg::Bool enable_msg;
          enable_msg.data = enabled;
          if (send_enable) {enable->publish(enable_msg);}
          interface_->write(rclcpp::Time(0), rclcpp::Duration(0, 0));
          rclcpp::spin_some(peer);
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      };
    pump(false, true, true, 1.0);
    ASSERT_EQ(observed.size(), 2u);
    EXPECT_DOUBLE_EQ(observed[0], 0.0);
    pump(true, true, true, 0.5);
    EXPECT_DOUBLE_EQ(observed[0], 1.0);
    pump(false, true, true, 0.3);
    EXPECT_DOUBLE_EQ(observed[0], 0.0);
    pump(true, true, true, 0.3);
    EXPECT_DOUBLE_EQ(observed[0], 1.0);
    pump(true, false, true, 0.8);
    EXPECT_DOUBLE_EQ(observed[0], 0.0);
    pump(true, true, true, 0.3);
    EXPECT_DOUBLE_EQ(observed[0], 1.0);
    pump(true, true, false, 0.5);
    EXPECT_DOUBLE_EQ(observed[0], 0.0);
    interface_->on_deactivate(rclcpp_lifecycle::State());
    rclcpp::shutdown();
  }

  void invalid_feedback() {interface_->feedbackCallback(std::make_shared<JointState>());}

  void healthy()
  {
    interface_->motor_enabled_.store(true);
    interface_->enable_received_ns_.store(interface_->steady_ns());
    interface_->battery_received_ns_.store(interface_->steady_ns());
    interface_->feedback_received_ns_ = interface_->steady_ns();
  }
  bool allowed() {return interface_->motion_allowed();}
  void disable() {interface_->motor_enabled_.store(false);}
  void expire_enable() {interface_->enable_received_ns_.store(interface_->steady_ns() - 600000000);}
  void expire_feedback() {interface_->feedback_received_ns_ = interface_->steady_ns() - 300000000;}
  void expire_battery()
  {
    interface_->battery_received_ns_.store(interface_->steady_ns() - 3000000000);
  }
  void critical() {interface_->battery_allows_motion_.store(false);}
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

    ASSERT_EQ(states.size(), 4);

    EXPECT_EQ(states[0].get_prefix_name(), "front_left_joint");
    EXPECT_EQ(states[0].get_interface_name(), hardware_interface::HW_IF_POSITION);
}

TEST_F(SabertoothInterfaceTest, ExportsCommandInterfaces)
  {
    auto commands = interface_->export_command_interfaces();

    ASSERT_EQ(commands.size(), 2);

    EXPECT_EQ(commands[0].get_prefix_name(), "front_left_joint");
    EXPECT_EQ(commands[0].get_interface_name(), hardware_interface::HW_IF_VELOCITY);

    EXPECT_EQ(commands[1].get_prefix_name(), "front_right_joint");
    EXPECT_EQ(commands[1].get_interface_name(), hardware_interface::HW_IF_VELOCITY);
}


TEST_F(SabertoothInterfaceTest, RosCommandsStopOnDisableHeartbeatLossAndFeedbackLoss)
{
 exercise_ros_stopping();
}

TEST_F(SabertoothInterfaceTest, InvalidFeedbackStopsMotion)
{
  healthy(); invalid_feedback(); EXPECT_FALSE(allowed());
}

TEST_F(SabertoothInterfaceTest, StartsDisabled) {
                                                 EXPECT_FALSE(allowed());
}
TEST_F(SabertoothInterfaceTest, HealthyTelemetryAndEnableAllowMotion)
{
  healthy();
  EXPECT_TRUE(allowed());
  disable();
  EXPECT_FALSE(allowed());
}
TEST_F(SabertoothInterfaceTest, ManagerLossStopsMotion)
{
  healthy(); expire_enable(); EXPECT_FALSE(allowed());
}
TEST_F(SabertoothInterfaceTest, FeedbackLossStopsMotion)
{
  healthy(); expire_feedback(); EXPECT_FALSE(allowed());
  interface_->read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  EXPECT_DOUBLE_EQ(interface_->get_hw_velocities()[0], 0.0);
}
TEST_F(SabertoothInterfaceTest, BatteryMonitorLossStopsMotion)
{
  healthy(); expire_battery(); EXPECT_FALSE(allowed());
}
TEST_F(SabertoothInterfaceTest, CriticalBatteryStopsMotion)
{
  healthy(); critical(); EXPECT_FALSE(allowed());
}
} // namespace rugged_rover_hardware_interfaces::sabertooth
