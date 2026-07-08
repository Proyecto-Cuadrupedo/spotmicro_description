#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class SpotMicroOdometryPublisher(Node):
    def __init__(self):
        super().__init__("spotmicro_odometry_publisher")
        self.last_pose = None
        self.last_time = None
        self.latest_odom = Odometry()
        self.latest_odom.header.frame_id = "odom"
        self.latest_odom.child_frame_id = "base_footprint"

        self.create_subscription(TFMessage, "/tf", self.tf_callback, 50)
        self.publisher = self.create_publisher(Odometry, "/spotmicro/odometry", 10)
        self.timer = self.create_timer(0.02, self.publish_odometry)

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id not in (
                "base_footprint",
                "spotmicro/base_footprint",
                "base_link",
                "spotmicro/base_link",
            ):
                continue

            now = self.get_clock().now()
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            self.latest_odom.header.stamp = now.to_msg()
            self.latest_odom.pose.pose.position.x = translation.x
            self.latest_odom.pose.pose.position.y = translation.y
            self.latest_odom.pose.pose.position.z = translation.z
            self.latest_odom.pose.pose.orientation = rotation

            if self.last_pose is not None and self.last_time is not None:
                dt = (now - self.last_time).nanoseconds / 1e9
                if dt > 0.0:
                    self.latest_odom.twist.twist.linear.x = (translation.x - self.last_pose[0]) / dt
                    self.latest_odom.twist.twist.linear.y = (translation.y - self.last_pose[1]) / dt
                    self.latest_odom.twist.twist.linear.z = (translation.z - self.last_pose[2]) / dt
                    self.latest_odom.twist.twist.angular.z = self._yaw_delta(rotation, self.last_pose[3]) / dt

            self.last_pose = (translation.x, translation.y, translation.z, rotation)
            self.last_time = now
            break

    def publish_odometry(self):
        if self.latest_odom.header.stamp.sec == 0 and self.latest_odom.header.stamp.nanosec == 0:
            self.latest_odom.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.latest_odom)

    def _yaw_delta(self, current, previous):
        current_yaw = self._yaw_from_quaternion(current)
        previous_yaw = self._yaw_from_quaternion(previous)
        return math.atan2(math.sin(current_yaw - previous_yaw), math.cos(current_yaw - previous_yaw))

    def _yaw_from_quaternion(self, quaternion):
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = SpotMicroOdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()