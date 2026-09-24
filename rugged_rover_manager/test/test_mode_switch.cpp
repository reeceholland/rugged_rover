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

#include <gtest/gtest.h>
#include "rugged_rover_manager/mode_switch.hpp"
using rugged_rover_manager::ModeRequest;
using rugged_rover_manager::ModeSwitch;

TEST(ModeSwitch, SingleToggleWaitsThenTeleop)
{
  ModeSwitch s;
  EXPECT_EQ(s.update(true, 0, 2), ModeRequest::NoChange);
  EXPECT_EQ(s.update(true, 1, 2), ModeRequest::NoChange);
  EXPECT_EQ(s.update(true, 2.1, 2), ModeRequest::Teleop);
  EXPECT_EQ(s.update(true, 3, 2), ModeRequest::NoChange);
}
TEST(ModeSwitch, FallingEdgeStopsAndCannotStartWhileLow)
{
  ModeSwitch s;
  s.update(true, 0, 2);
  EXPECT_EQ(s.update(false, 0.5, 2), ModeRequest::Stop);
  EXPECT_EQ(s.update(false, 2.1, 2), ModeRequest::NoChange);
  EXPECT_EQ(s.update(false, 3, 2), ModeRequest::NoChange);
}
TEST(ModeSwitch, DoubleToggleStillSelectsAutonomous)
{
  ModeSwitch s;
  s.update(true, 0, 2);
  EXPECT_EQ(s.update(false, 0.5, 2), ModeRequest::Stop);
  EXPECT_EQ(s.update(true, 1, 2), ModeRequest::Autonomous);
  EXPECT_EQ(s.update(false, 1.2, 2), ModeRequest::Stop);
  EXPECT_EQ(s.update(false, 4, 2), ModeRequest::NoChange);
}
TEST(ModeSwitch, HighAtStartupStartsTeleopAfterWindow)
{
  ModeSwitch s;
  EXPECT_EQ(s.update(true, 3, 2), ModeRequest::NoChange);
  EXPECT_EQ(s.update(true, 5.1, 2), ModeRequest::Teleop);
  EXPECT_EQ(s.update(false, 6, 2), ModeRequest::Stop);
}
TEST(ModeSwitch, LateSecondToggleStartsNewWindow)
{
  ModeSwitch s;
  s.update(true, 0, 2);
  s.update(false, 0.5, 2);
  EXPECT_EQ(s.update(true, 3, 2), ModeRequest::NoChange);
  EXPECT_EQ(s.update(true, 5.1, 2), ModeRequest::Teleop);
}
