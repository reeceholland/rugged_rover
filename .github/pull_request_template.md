# Pull Request: [Short Description]

## Summary

<!-- What changed, and which rover workflow does it affect? -->
- [ ] Real rover bringup/control
- [ ] Unity simulation
- [ ] Teensy firmware / micro-ROS
- [ ] SLAM / Nav2
- [ ] Documentation / bootstrap / CI

## Motivation

<!-- Why is this change needed? Link issues or notes if useful. -->
Resolves: #<issue_number>

## Testing Strategy

- [ ] Built with `colcon build --symlink-install --packages-up-to rugged_rover_bringup`
- [ ] Ran relevant launch file(s)
- [ ] Checked relevant topics, TF, or diagnostics
- [ ] Ran ROS 2 lint checks (`ament_uncrustify`, `ament_copyright`)

**Tested platforms:**
- [ ] Raspberry Pi 5 + Ubuntu Server 24.04 + ROS 2 Jazzy
- [ ] WSL/desktop Ubuntu 24.04 + ROS 2 Jazzy
- [ ] Unity simulation
- [ ] Other: ____

## Implementation Notes

<!-- Mention assumptions, calibration values, topic changes, parameters, or hardware caveats. -->

## Affected Packages

<!-- Delete entries that do not apply. -->
- `rugged_rover_bringup`
- `rugged_rover_control`
- `rugged_rover_hardware_interfaces`
- `rugged_rover_robot_description`
- `rugged_rover_battery`
- `rugged_rover_interfaces`
- `rugged_rover_test`
- `micro_ros_platform_firmware`
- `rugged_rover_imu`
- `sllidar_ros2`

## How to Test

```bash
cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to rugged_rover_bringup
source install/setup.bash

# Real rover baseline
ros2 launch rugged_rover_bringup bringup.launch.py

# Optional Nav2 on the physical rover
ros2 launch rugged_rover_bringup nav2.launch.py use_sim_time:=false
```
