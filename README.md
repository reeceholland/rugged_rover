# Rugged Rover

Rugged Rover is a ROS 2 Jazzy rover stack for the Lynxmotion A4WD3 platform. It supports both the real rover hardware path and a Unity simulation path, using the same robot description, ros2_control controllers, and navigation stack where possible.

The current system is built around:

- ROS 2 Jazzy on Ubuntu 24.04
- `ros2_control` with a custom Sabertooth hardware interface
- Teensy 4.1 firmware using micro-ROS for motor command and encoder feedback
- A4WD3 chassis and wheel meshes in the URDF
- Unity simulation through `ros_tcp_endpoint`
- SLAM Toolbox, Nav2, RViz, and differential drive control

## Repository Layout

| Path | Purpose |
| --- | --- |
| `rugged_rover_bringup` | Hardware, Unity sim, SLAM, and Nav2 launch files plus shared configs. |
| `rugged_rover_robot_description` | Xacro/URDF, meshes, RViz configs, and robot visualization launches. |
| `rugged_rover_hardware_interfaces` | Custom `ros2_control` system interface that bridges controller commands to `/platform/motors/cmd` and reads `/platform/motors/feedback`. |
| `rugged_rover_control` | Controller configuration and standalone control launch support. |
| `rugged_rover_interfaces` | Custom message and service definitions. |
| `rugged_rover_test` | Small test publishers for command and joint-state experiments. |
| `micro_ros_platform_firmware` | Teensy 4.1 firmware for Sabertooth motor control, encoder feedback, and micro-ROS communication. |
| `micro_ros_agent` | micro-ROS Agent submodule. |

## Main ROS Topics

| Topic | Type | Direction | Notes |
| --- | --- | --- | --- |
| `/platform/motors/cmd` | `sensor_msgs/msg/JointState` | ROS -> Teensy/Unity | Wheel velocity commands from the hardware interface. |
| `/platform/motors/feedback` | `sensor_msgs/msg/JointState` | Teensy/Unity -> ROS | Wheel position and velocity feedback consumed by `ros2_control`. |
| `/joint_states` | `sensor_msgs/msg/JointState` | ROS | Published by `joint_state_broadcaster`. |
| `/diff_drive_controller/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Nav2/manual -> controller | Command input for the Jazzy diff drive controller. |
| `/diff_drive_controller/odom` | `nav_msgs/msg/Odometry` | ROS | Wheel odometry from `diff_drive_controller`. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Sensor/sim -> SLAM/Nav2 | 2D lidar scan input. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Unity -> ROS | Required when running the Unity simulation with `use_sim_time`. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | ROS | Robot, odom, map, and sensor transforms. |

## Prerequisites

Install ROS 2 Jazzy and the rover dependencies on Ubuntu 24.04:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-topic-tools
```

Initialize `rosdep` once if needed:

```bash
sudo rosdep init
rosdep update
```

## Clone and Build

```bash
mkdir -p ~/rugged_rover_ws/src
cd ~/rugged_rover_ws/src
git clone --branch support-unity-sim --recurse-submodules git@github.com:reeceholland/rugged_rover.git
git clone --branch main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd rugged_rover
git submodule update --init --recursive

cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
```

If only changing this repo, a focused build is usually enough:

```bash
cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to rugged_rover_bringup
source install/setup.bash
```

## Hardware Bringup

The hardware path expects a Teensy 4.1 running the firmware in `micro_ros_platform_firmware/platform` and a micro-ROS Agent connected over USB serial.

Attach the Teensy to WSL if working from Windows:

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

Then in WSL:

```bash
ls /dev/ttyACM*
sudo chmod a+rw /dev/ttyACM0
```

Launch the rover hardware stack:

```bash
cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rugged_rover_bringup bringup.launch.py
```

Useful manual motor test:

```bash
ros2 topic pub --once /platform/motors/cmd sensor_msgs/msg/JointState "
name:
- front_left_joint
- front_right_joint
- rear_left_joint
- rear_right_joint
velocity:
- 2.0
- 2.0
- 2.0
- 2.0
"
```

Stop command:

```bash
ros2 topic pub --once /platform/motors/cmd sensor_msgs/msg/JointState "
name:
- front_left_joint
- front_right_joint
- rear_left_joint
- rear_right_joint
velocity:
- 0.0
- 0.0
- 0.0
- 0.0
"
```

## Unity Simulation

The Unity simulation publishes sensor data and motor feedback through `ros_tcp_endpoint`. The `ros_tcp_endpoint` package comes from the sibling `ROS-TCP-Endpoint` checkout in the workspace. Start the ROS side first:

```bash
cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rugged_rover_bringup unity_sim.launch.py
```

Then press Play in Unity. The ROS TCP endpoint is configured for:

```text
ROS_IP=127.0.0.1
ROS_TCP_PORT=10000
```

The simulation launch remaps Unity odometry and TF away from the navigation-owned topics so wheel odometry, robot state publisher, SLAM, and Nav2 can own the normal ROS navigation frames.

## Viewing the Robot

View the robot model by itself:

```bash
ros2 launch rugged_rover_robot_description view_robot.launch.py
```

Open the simulation RViz config:

```bash
ros2 launch rugged_rover_robot_description sim_rviz.launch.py
```

For Unity simulation, RViz should normally use `use_sim_time:=true` and a fixed frame appropriate to the current stack, usually `odom` before SLAM and `map` after SLAM is active.

## SLAM

Start SLAM Toolbox:

```bash
ros2 launch rugged_rover_bringup slam.launch.py
```

This launch starts `async_slam_toolbox_node` and transitions it through configure and activate automatically. It expects:

- `/clock` when `use_sim_time:=true`
- `/scan` from Unity or a real lidar
- a valid TF chain from `odom` to `base_link` and from `base_link` to the laser frame

Check that a map is being published:

```bash
ros2 topic echo /map --once
```

## Nav2

Start Nav2:

```bash
ros2 launch rugged_rover_bringup nav2.launch.py
```

The Nav2 launch includes a `topic_tools transform` bridge from Nav2's `/cmd_vel` output to the Jazzy diff drive controller's stamped command topic:

```text
/cmd_vel -> /diff_drive_controller/cmd_vel
```

If Nav2 plans but the rover does not move, check:

```bash
ros2 topic info /diff_drive_controller/cmd_vel -v
ros2 topic echo /diff_drive_controller/cmd_vel --once
ros2 topic echo /platform/motors/cmd --once
ros2 control list_controllers
```

## Teensy Firmware Notes

The firmware lives in:

```text
micro_ros_platform_firmware/platform
```

Important modes are controlled from `config.hpp`:

```cpp
#define USE_ROS 1
```

Use `USE_ROS 1` for micro-ROS operation. Use `USE_ROS 0` for direct Arduino Serial Monitor testing, where the firmware accepts left/right wheel speed commands in radians per second.

The current hardware wiring expects the Sabertooth serial command line on Teensy `Serial2`:

```text
Teensy 4.1 pin 8  TX2 -> level shifter -> Sabertooth S1
Teensy GND             -> Sabertooth 0V / battery ground
```

## Formatting and Checks

Format C++ code with:

```bash
find . \( -name "*.cpp" -o -name "*.hpp" \) -exec clang-format -i {} +
```

Build the main stack:

```bash
cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to rugged_rover_bringup
```

Run the hardware interface test package when needed:

```bash
colcon test --packages-select rugged_rover_hardware_interfaces
colcon test-result --verbose
```

## Current Development Focus

- Robust real rover motor control through Teensy + Sabertooth + encoder feedback
- Unity simulation parity with the physical rover
- SLAM and Nav2 tuning for the rover footprint and available sensors
- Raspberry Pi deployment on Ubuntu 24.04 + ROS 2 Jazzy
- Future real-world perception with camera/depth/lidar inputs

## Maintainer

Reece Holland

- GitHub: https://github.com/reeceholland
- Website: http://reeceholland.github.io/
