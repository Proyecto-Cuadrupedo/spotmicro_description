#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SpotMicroForceTest(Node):
    def __init__(self):
        super().__init__("spotmicro_force_test")
        self.declare_parameter("topic", "/front_left_leg_cmd")
        self.declare_parameter("amplitude", 120.0)
        self.declare_parameter("frequency", 0.5)

        self.topic = self.get_parameter("topic").value
        self.amplitude = float(self.get_parameter("amplitude").value)
        self.frequency = float(self.get_parameter("frequency").value)
        self.publisher = self.create_publisher(Float64, self.topic, 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.01, self.publish_force)
        self.get_logger().info(
            f"Publishing direct force on {self.topic}: amplitude={self.amplitude}, frequency={self.frequency} Hz"
        )

    def publish_force(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        msg = Float64()
        msg.data = self.amplitude * math.sin(2.0 * math.pi * self.frequency * elapsed)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpotMicroForceTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()