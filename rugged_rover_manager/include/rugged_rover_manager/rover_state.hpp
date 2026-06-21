#pragma once

namespace rugged_rover_manager
{
  enum class RoverState
  {
      Booting,
      Teleop,
      Autonomous,
      LowBattery,
      Fault,
      EStopped,
      ShutdownRequested,
    };

} // namespace rugged_rover_manager