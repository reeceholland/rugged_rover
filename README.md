# Rugged Rover

Rugged Rover is a ROS 2 Jazzy stack for a Lynxmotion A4WD3 skid-steer rover. The
current target platform is a Raspberry Pi 5 running Ubuntu Server 24.04, with a
Teensy 4.1 handling the motor controller, encoder feedback, battery monitoring,
and micro-ROS transport.

The stack supports:

- `ros2_control` differential drive control
- Teensy 4.1 + Sabertooth motor control over micro-ROS
- A4WD3 URDF with chassis, wheel, lidar, camera, and IMU frames
- RPLIDAR S2 laser scans
- SparkFun Razor IMU
- Intel RealSense D435 depth camera support
- SLAM Toolbox mapping
- Nav2 autonomous navigation
- Unity simulation support through `ros_tcp_endpoint`

## Repository Layout

| Path | Purpose |
| --- | --- |
| `rugged_rover_bringup` | Real rover, Unity sim, SLAM, Nav2, joystick, lidar, camera, and EKF launch files. |
| `rugged_rover_robot_description` | Xacro/URDF, meshes, RViz configs, and robot visualization launch files. |
| `rugged_rover_control` | `ros2_control` controller configuration. |
| `rugged_rover_hardware_interfaces` | Custom Sabertooth `ros2_control` hardware interface. |
| `rugged_rover_battery` | Battery voltage monitor and diagnostics node. |
| `rugged_rover_interfaces` | Custom message and service definitions. |
| `rugged_rover_test` | Small command/test publishers. |
| `micro_ros_platform_firmware` | Teensy firmware for the A4WD3 platform. |
| `micro_ros_agent` | micro-ROS Agent submodule. |
| `rugged_rover_imu` | Razor IMU ROS 2 driver and firmware submodule. |
| `sllidar_ros2` | Slamtec RPLIDAR ROS 2 driver submodule. |
| `rugged_rover_bootstrap` | Raspberry Pi 5 bootstrap installer for Ubuntu 24.04, ROS 2 Jazzy, udev rules, and workspace setup. |

## Hardware Baseline

The current A4WD3 setup assumes:

- Raspberry Pi 5 running Ubuntu Server 24.04
- ROS 2 Jazzy
- Teensy 4.1
- Sabertooth motor driver
- A4WD3 motors with 51:1 gearbox and quadrature encoders
- RPLIDAR S2 on `/dev/rplidar`
- SparkFun Razor IMU on `/dev/razor_imu`
- Teensy micro-ROS UART on `/dev/ttyAMA0`
- Optional RealSense D435 depth camera
- Common ground between the Pi, Teensy, motor driver, battery monitor, and motor power system

The controller calibration currently uses:

```yaml
wheel_separation: 0.475
wheel_radius: 0.082
```

These are effective odometry values, not simply the measured wheel-to-wheel
distance. The physical rear wheel center-to-center distance is about `0.355 m`,
but the skid-steer odometry needed a larger effective separation after rotation
testing.

## Fresh Raspberry Pi 5 Setup

Start from Ubuntu Server 24.04 on the Raspberry Pi 5. SSH into the Pi, then run:

```bash
git clone --branch main git@github.com:reeceholland/rugged_rover.git ~/rugged_rover_bootstrap
cd ~/rugged_rover_bootstrap
bash rugged_rover_bootstrap/install_pi5_ubuntu24_ros_jazzy.sh
```

The installer:

- repairs the Ubuntu noble-updates apt source if it is missing
- installs bzip2/libbz2 after the apt source repair
- installs ROS 2 Jazzy, rover dependencies, and ROS lint tools
- configures the ROS apt repository
- configures rosdep
- clones this repository into ~/rugged_rover_ws/src/rugged_rover
- initializes submodules
- creates udev aliases for the Teensy UART, RPLIDAR, and Razor IMU
- builds the workspace
- adds ROS setup lines to ~/.bashrc

After the script finishes, reboot:

```bash
sudo reboot
```

Reconnect and verify:

```bash
cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ls -l /dev/teensy_uart /dev/rplidar /dev/razor_imu 2>/dev/null
ros2 pkg list | grep rugged_rover
```

If a device symlink is missing, check the raw USB devices:

```bash
lsusb
ls -l /dev/ttyAMA* /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

## Manual Workspace Setup

If you do not use the Pi installer:

```bash
mkdir -p ~/rugged_rover_ws/src
cd ~/rugged_rover_ws/src
git clone --branch main git@github.com:reeceholland/rugged_rover.git
cd rugged_rover
git submodule update --init --recursive

cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

Useful focused rebuild:

```bash
cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to rugged_rover_bringup
source install/setup.bash
```

## Formatting and Linting

This repository uses the ROS 2 lint tools for formatting and license checks:

```bash
cd ~/rugged_rover_ws/src/rugged_rover
source /opt/ros/jazzy/setup.bash

ament_uncrustify \
  micro_ros_platform_firmware/platform \
  rugged_rover_battery \
  rugged_rover_bringup \
  rugged_rover_control \
  rugged_rover_hardware_interfaces \
  rugged_rover_interfaces \
  rugged_rover_robot_description \
  rugged_rover_test

ament_copyright \
  micro_ros_platform_firmware/platform \
  rugged_rover_battery \
  rugged_rover_bringup \
  rugged_rover_control \
  rugged_rover_hardware_interfaces \
  rugged_rover_interfaces \
  rugged_rover_robot_description \
  rugged_rover_test
```

To reformat C/C++ files in place:

```bash
ament_uncrustify --reformat \
  micro_ros_platform_firmware/platform \
  rugged_rover_battery \
  rugged_rover_bringup \
  rugged_rover_control \
  rugged_rover_hardware_interfaces \
  rugged_rover_interfaces \
  rugged_rover_robot_description \
  rugged_rover_test
```

To add missing ROS-style copyright headers:

```bash
ament_copyright --add-missing "Reece Holland" apache2 \
  micro_ros_platform_firmware/platform \
  rugged_rover_battery \
  rugged_rover_bringup \
  rugged_rover_control \
  rugged_rover_hardware_interfaces \
  rugged_rover_interfaces \
  rugged_rover_robot_description \
  rugged_rover_test
```

The parent repository intentionally excludes external/submodule code from these checks.

## Teensy Firmware

Firmware lives in:

```text
micro_ros_platform_firmware/platform
```

For normal rover operation, `config.hpp` should contain:

```cpp
#define USE_ROS 1
```

For Arduino Serial Monitor bench testing without ROS:

```cpp
#define USE_ROS 0
```

In bench mode the firmware accepts:

```text
<left_rad_s> <right_rad_s>
```

Example:

```text
2.0 2.0
```

Use the Arduino IDE with Teensyduino to flash:

1. Open `micro_ros_platform_firmware/platform/platform.ino`.
2. Select the Teensy 4.1 board.
3. Confirm `USE_ROS` is set correctly.
4. Build and upload.
5. Power-cycle the Sabertooth if the motor driver does not respond after firmware changes.

The firmware publishes:

- `/platform/motors/feedback`
- `/battery/voltage`
- `/platform/debug`

and subscribes to:

- `/platform/motors/cmd`

## Wiring Summary

Teensy to Raspberry Pi UART:

```text
Pi GPIO14 / TXD0 / pin 8  -> Teensy RX1 / pin 0
Pi GPIO15 / RXD0 / pin 10 -> Teensy TX1 / pin 1
Pi GND                   -> Teensy GND
```

Teensy to Sabertooth:

```text
Teensy TX2 / pin 8 -> level shifter -> Sabertooth S1
Teensy GND         -> Sabertooth 0V / battery ground
```

Battery voltage monitor:

```text
Battery + -> 100k resistor -> Teensy A9 -> 33k resistor -> GND
Teensy A9 -> 100 nF capacitor -> GND
Battery - -> common ground
```

Important electrical notes:

- The Pi, Teensy, Sabertooth signal ground, battery monitor ground, and sensor grounds must share a common reference.
- Do not power high-current USB devices from an overloaded Pi USB bus.
- The D435 should run from a stable USB3 path. If the kernel reports `over-current change`, move the D435 or RPLIDAR power to a powered hub or separate 5 V supply.

## Device Checks

Check the Pi is not throttling:

```bash
vcgencmd get_throttled
```

Expected:

```text
throttled=0x0
```

Check USB topology:

```bash
lsusb -t
```

The D435 should appear on a `5000M` USB3 bus. If it appears on `480M`, change the cable or USB port.

Watch kernel USB events:

```bash
sudo dmesg -w
```

## Bring Up the A4WD3

Launch the real rover baseline:

```bash
cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rugged_rover_bringup bringup.launch.py
```

By default this launches:

- micro-ROS Agent on `/dev/ttyAMA0`
- battery voltage monitor
- robot state publisher
- Razor IMU driver
- RPLIDAR S2 driver
- SLAM Toolbox
- `ros2_control`
- `joint_state_broadcaster`
- `diff_drive_controller`

EKF is off by default while validating raw wheel odometry. Enable it with:

```bash
ros2 launch rugged_rover_bringup bringup.launch.py use_ekf:=true
```

Disable SLAM if you only want low-level control testing:

```bash
ros2 launch rugged_rover_bringup bringup.launch.py use_slam:=false
```

Disable the RPLIDAR:

```bash
ros2 launch rugged_rover_bringup bringup.launch.py use_rplidar:=false
```

Use an explicit RPLIDAR port:

```bash
ros2 launch rugged_rover_bringup bringup.launch.py rplidar_serial_port:=/dev/ttyUSB0
```

## Validate the Stack

Check nodes:

```bash
ros2 node list
```

Check controllers:

```bash
ros2 control list_controllers
```

Expected:

```text
joint_state_broadcaster active
diff_drive_controller active
```

Check main topics:

```bash
ros2 topic list
ros2 topic hz /platform/motors/feedback
ros2 topic hz /joint_states
ros2 topic hz /diff_drive_controller/odom
ros2 topic hz /scan
ros2 topic echo /battery/voltage --once
ros2 topic echo /platform/debug --once
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_tools view_frames
```

Check SLAM:

```bash
ros2 topic echo /map --once
```

## Manual Driving

The diff drive controller accepts `TwistStamped` commands:

```bash
ros2 topic info /diff_drive_controller/cmd_vel -v
```

Slow forward command:

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "
header:
  frame_id: base_link
twist:
  linear:
    x: 0.1
  angular:
    z: 0.0
"
```

Turn on the spot:

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "
header:
  frame_id: base_link
twist:
  linear:
    x: 0.0
  angular:
    z: 0.4
"
```

Stop:

```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: base_link
twist:
  linear:
    x: 0.0
  angular:
    z: 0.0
"
```

Direct Teensy motor test:

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

Direct stop:

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

If `/diff_drive_controller/cmd_vel` receives commands but the rover does not move, check:

```bash
ros2 topic echo /platform/motors/cmd
ros2 topic echo /platform/debug
ros2 topic echo /battery/voltage
```

Look for stale commands, `battery_critical=true`, missing micro-ROS connection, or setpoints that reach the Teensy while measured velocity remains zero.

## SLAM

`bringup.launch.py` starts SLAM by default. To launch SLAM separately:

```bash
ros2 launch rugged_rover_bringup slam.launch.py
```

SLAM expects:

- `/scan` from the RPLIDAR
- `odom -> base_link`
- `base_link -> laser`
- `use_sim_time:=false` on the real rover

Useful checks:

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_monitor odom base_link
ros2 run tf2_ros tf2_monitor map odom
ros2 topic echo /map --once
```

Loop closure can be disabled in `rugged_rover_bringup/config/slam_toolbox.yaml`
while tuning odometry and scan matching.

## Nav2

Launch Nav2 after bringup is healthy:

```bash
ros2 launch rugged_rover_bringup nav2.launch.py use_sim_time:=false
```

The Nav2 launch file defaults to Unity/simulation time. Always pass
`use_sim_time:=false` on the physical rover.

Before sending goals, confirm:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_link
```

If Nav2 plans but does not move the rover:

```bash
ros2 topic info /diff_drive_controller/cmd_vel -v
ros2 topic echo /diff_drive_controller/cmd_vel --once
ros2 topic echo /platform/motors/cmd --once
ros2 control list_controllers
```

If the controller reports old TF data, reduce CPU load, check scan rate, and verify
that all real-rover nodes have `use_sim_time:=false`.

## Joystick Teleop

Launch joystick teleop:

```bash
ros2 launch rugged_rover_bringup joy.launch.py
```

Default Xbox-style controls:

- Hold `LB` to enable driving
- Hold `RB` as well for turbo speed
- Left stick vertical controls forward/reverse
- Right stick horizontal controls rotation

Use a different joystick device id:

```bash
ros2 launch rugged_rover_bringup joy.launch.py joy_dev:=1
```

Debug joystick messages and generated velocity commands:

```bash
ros2 launch rugged_rover_bringup joy_debug.launch.py
```

## RPLIDAR S2

Launch only the RPLIDAR:

```bash
ros2 launch rugged_rover_bringup rplidar_s2.launch.py
```

Check scan output:

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

If the lidar stops when the D435 starts, inspect USB power events:

```bash
sudo dmesg -w
```

`over-current change` or `USB disconnect` means the Pi USB bus is being overloaded.
Use a powered USB3 hub or separate sensor power.

## RealSense D435

Launch the D435:

```bash
ros2 launch rugged_rover_bringup d435.launch.py
```

This publishes depth camera data and remaps `depthimage_to_laserscan` to:

```text
/depth_scan
```

The RPLIDAR should remain the owner of:

```text
/scan
```

Check ownership:

```bash
ros2 topic info /scan -v
ros2 topic info /depth_scan -v
```

The D435 should enumerate as USB3:

```bash
lsusb -t
```

Look for `5000M`. If it disconnects while streaming, use a better USB3 cable,
another USB3 port, or a powered USB3 hub.

## Unity Simulation

Unity simulation support lives in `rugged_rover_bringup/launch/unity_sim.launch.py`.
Start the ROS side:

```bash
cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rugged_rover_bringup unity_sim.launch.py
```

Then press Play in Unity.

The ROS TCP endpoint binds to `0.0.0.0:10000` by default so Unity can run
on the same machine or another laptop. In Unity, set the ROS TCP Connector
address to the ROS machine's IP address and port `10000`.

For local-only testing, bind the endpoint to localhost:

```bash
ros2 launch rugged_rover_bringup unity_sim.launch.py ros_tcp_ip:=127.0.0.1
```

When testing fault injection, some Unity topics may be remapped with a `_raw`
postfix. Make sure SLAM subscribes to the topic that Unity is actually publishing:

```bash
ros2 topic info /scan -v
ros2 topic info /scan_raw -v
```

## Remote ROS 2 Tools from WSL

WSL2 can see the Pi over normal IP networking, but DDS discovery may not work
reliably unless both machines can exchange multicast traffic. If `ping` works
but WSL shows no rover topics, use one of these approaches:

- run RViz and ROS CLI tools directly on the ROS laptop/Pi network
- enable WSL mirrored networking on Windows 11
- use a Fast DDS discovery server and set the same `ROS_DISCOVERY_SERVER` on
  the Pi and WSL

Example discovery-server environment:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER=<pi-or-ros-laptop-ip>:11811
```

Then restart the ROS nodes and use:

```bash
ros2 topic list --no-daemon
```

## Common Troubleshooting

No `/platform/motors/cmd` subscriber:

```bash
ros2 node list | grep micro
ros2 topic info /platform/motors/cmd -v
```

If the Teensy is not connected, check the agent:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 115200
```

Permission denied on `/dev/ttyAMA0`:

```bash
groups
ls -l /dev/ttyAMA0
sudo chmod 666 /dev/ttyAMA0
```

The `chmod` command is a temporary fix. The persistent fix is the udev rule in
`install_pi5_ubuntu24_ros_jazzy.sh`, followed by a reboot.

Lidar or camera disappears:

```bash
sudo dmesg -w
lsusb
lsusb -t
```

Rover moves in RViz but not physically:

```bash
ros2 topic echo /platform/motors/cmd
ros2 topic echo /platform/debug
```

Physical rover moves but odom is wrong:

- verify encoder counts and gearbox ratio in the Teensy firmware
- verify wheel radius in `rugged_rover_control/config/controllers.yaml`
- verify effective wheel separation with a 360 degree rotation test

SLAM map jumps or duplicates rooms:

- validate `/diff_drive_controller/odom` first
- check `odom -> base_link` TF age
- disable loop closure while tuning
- confirm the laser frame transform is correct
- keep CPU load low enough that scan processing does not fall behind

Stop common ROS processes:

```bash
pkill -f ros_tcp_endpoint
pkill -f slam_toolbox
pkill -f rviz2
pkill -f ros2_control_node
pkill -f robot_state_publisher
pkill -f ekf_node
pkill -f micro_ros_agent
ros2 daemon stop
ros2 daemon start
```

## Formatting and Tests

Format C++ files:

```bash
ament_uncrustify --reformat \
  micro_ros_platform_firmware/platform \
  rugged_rover_battery \
  rugged_rover_bringup \
  rugged_rover_control \
  rugged_rover_hardware_interfaces \
  rugged_rover_interfaces \
  rugged_rover_robot_description \
  rugged_rover_test
```

Build the main stack:

```bash
cd ~/rugged_rover_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to rugged_rover_bringup
```

Run hardware interface tests:

```bash
colcon test --packages-select rugged_rover_hardware_interfaces
colcon test-result --verbose
```

## Maintainer

Reece Holland

- GitHub: https://github.com/reeceholland
- Website: http://reeceholland.github.io/
