#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelToStamped(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_stamped")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/diff_drive_controller/cmd_vel")
        self.declare_parameter("frame_id", "base_link")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.publisher = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            self.input_topic,
            self.on_twist,
            10,
        )

        self.get_logger().info(
            f"Bridging {self.input_topic} Twist -> "
            f"{self.output_topic} TwistStamped"
        )

    def on_twist(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.publisher.publish(stamped)


def main():
    rclpy.init()
    node = CmdVelToStamped()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
