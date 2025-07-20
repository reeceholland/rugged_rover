# Pull Request: [Short Description of Your Change]

## Summary

<!-- Provide a concise summary of what this PR does. -->
This pull request introduces the following changes:
- [x] Feature or fix description
- [x] Changes to specific packages or nodes
- [x] Relevant ROS 2 integration (topics, parameters, etc.)

---

## Motivation

<!-- Explain why this change is needed. Reference issues or feature requests if applicable. -->
Resolves: #<issue_number>

---

## Testing Strategy

- [x] Built and tested with `colcon build` on ROS 2 Humble
- [x] Ran `ros2 run` or launch files to verify functionality
- [x] Unit tests added or updated
- [x] Linter and format tools passed (`ament_lint`, `clang-format`, etc.)

**Tested platforms:**
- [ ] Ubuntu 22.04 + ROS 2 Humble
- [ ] Ubuntu 20.04 + ROS 2 Foxy
- [ ] Other: ____

---

## Implementation Notes

<!-- Detail any design decisions, protocol changes, or assumptions. -->
- Introduced new serial protocol format with checksum
- Abstracted motor driver as ROS 2 node
- Added parameterization for port and baudrate

---

## Affected Packages

- `sabertooth_motor_driver`
- `driver`
- `motor_controller_node`

---

## How to Test

```bash
# Build
colcon build --packages-select sabertooth_motor_driver

# Source
. install/setup.bash

# Run the node
ros2 run sabertooth_motor_driver motor_controller_node

# Or launch
ros2 launch sabertooth_motor_driver motor_controller.launch.py
