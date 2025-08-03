# Rugged Rover - A Work in Progress

**Rugged Rover** is a ROS 2-based autonomous mobile robot platform designed to run on the Lynxmotion A4WD3 Rugged Wheeled Rover.

---

## Features

- ROS 2 `ros2_control` integration with custom hardware interface for Sabertooth motor drivers
- Differential drive controller with 4-wheel support
- Encoder feedback and velocity control via Teensy 4.1 + micro-ROS
- Teleoperation via Logitech joystick or keyboard (TODO)
- Realistic URDF + RViz visualization (Currently Basic)
- Launch system for hardware and simulation
- Modular packages for control, description, and hardware

---

## Packages

| Package Name | Description |
|--------------|-------------|
| `rugged_rover_hardware_interfaces` | Custom hardware interface for the Sabertooth motor driver |
| `rugged_rover_robot_description`   | URDF/Xacro model of the rover |
| `rugged_rover_control`             | Controller YAML configs and `ros2_control` parameters |
| `rugged_rover_bringup`             | Launch files for bringup and system orchestration |
| `rugged_rover_joy`                 | Joystick teleoperation node (TODO) |
| `rugged_rover_interfaces`         | Custom message definitions (e.g., `RoverFeedback`) |

---

## Getting Started

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Colcon workspace setup

### Clone and Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/reeceholland/rugged_rover.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## Bringup

### Hardware Launch

```bash
ros2 launch rugged_rover_bringup bringup.launch.py
```



---


## Topics Overview

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command input |
| `/joint_states` | `sensor_msgs/JointState` | Joint feedback |
| `/platform/motors/cmd` | `JointState` or custom | Motor velocity command |
| `/platform/motors/feedback` | `sensor_msgs/JointState` | Velocity feedback |
| `/odom` | `nav_msgs/Odometry` | Odometry published by diff drive controller |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |


---

## Contributing

Pull requests are welcome! Please format code with:

```bash
find . \( -name "*.cpp" -o -name "*.hpp" \) -exec clang-format -i {} +
```

Run linters and tests before submitting.

---

## License

This project is licensed under the MIT License. See `LICENSE` file for details.

---

## Roadmap

- [x] Hardware interface integration
- [x] URDF and visualization
- [x] Differential drive controller
- [ ] Teleoperation
- [ ] EKF-based localization
- [ ] SLAM and navigation stack
- [ ] Autonomous waypoint following

---

## 👤 Maintainer

**Reece Holland**  
https://github.com/reeceholland  
[reeceholland.github.io](http://reeceholland.github.io/)