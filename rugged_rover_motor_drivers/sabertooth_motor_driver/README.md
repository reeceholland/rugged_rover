# Sabertooth Motor Driver
C++ Driver and ROS 2 node for a Sabertooth motor driver controlled through an Ardunino

## Usage

This driver will be automatically launched from the Rugged Rover system.

### Manual launch

`ros2 launch sabertooth_motor_driver motor_driver_launch.launch.py`

## Node

The sabertooth driver node will create a driver for for communicating with the arduino, that in turn communicates with the Sabertooth motor controller. It will subscribe to velocity commands coming from ROS 2 controls, and publish feedback.

Additionally, the sabertooth driver node will be used to configure the Sabertooth motor drivers.

### Subscribers

- `platform/motors/cmd`: Velocity commands to send to the motor controller.
  - Type: `sensor_msgs/msg/JointState`

### Publishers
- `platform/motors/feedback`: Motor feedback such as velocity and position.
  - Type: `clearpath_motor_msgs/msg/LynxMultiFeedback`

### Actions
TODO