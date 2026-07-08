#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState

from spotmicro_high_level_controller import JOINT_NAMES
from spotmicro_reference import JOINT_LIMITS, SpotMicroReferenceController


class SpotMicroCrawlController(Node):
    def __init__(self):
        super().__init__("spotmicro_crawl_controller")
        self.freq = 60
        self.reference_controller = SpotMicroReferenceController()
        self.reference_stand_theta = self.reference_controller.stand_joint_angles()
        self.positions = self.reference_controller.stand_joint_angles()
        self.goal_pos = np.array(self.reference_controller.stand_joint_angles(), dtype=float)
        self.goal_yaw = self.yaw = 0.0
        self.command_deadband = 0.01
        self.walk_blend = 0.0

        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        self.create_subscription(Pose, "/goal_pose", self.goal_pos_callback, 10)
        self.pub = self.create_publisher(JointState, "/high_level/joint_cmd", 10)
        self.msg = JointState()
        self.msg.name = JOINT_NAMES

        ok, first_pos = wait_for_message(JointState, self, "/joint_states", time_to_wait=5)
        if ok:
            self.positions = self._ordered_positions(first_pos)
        self.hold_theta = list(self.positions)
        self.goal_pos = np.array(self.hold_theta, dtype=float)
        self.publish_joint_command(self.goal_pos)
        self.timer = self.create_timer(1.0 / self.freq, self.update)

    def update(self):
        if abs(self.goal_yaw) < self.command_deadband:
            self.walk_blend = 0.0
            self.goal_pos = np.array(self.hold_theta, dtype=float)
            self.publish_joint_command(self.goal_pos)
            return

        yaw_error = (self.goal_yaw - self.yaw) / 20.0
        self.yaw += yaw_error
        reference_theta = self.reference_controller.step(0.0, 0.0, yaw_error)
        self.walk_blend = min(1.0, self.walk_blend + 0.02)
        self.goal_pos = np.array(self.apply_reference_delta(reference_theta), dtype=float)
        self.publish_joint_command(self.goal_pos)

    def apply_reference_delta(self, reference_theta):
        command = []
        for index, (hold, reference, reference_stand) in enumerate(zip(self.hold_theta, reference_theta, self.reference_stand_theta)):
            delta = (reference - reference_stand) * self.walk_blend
            command.append(self.clamp_joint(index, hold + delta))
        return command

    def clamp_joint(self, index, value):
        lower, upper = JOINT_LIMITS[index]
        return max(lower, min(upper, value))

    def publish_joint_command(self, position):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.position = list(position)
        self.pub.publish(self.msg)

    def joint_states_callback(self, msg):
        self.positions = self._ordered_positions(msg)

    def goal_pos_callback(self, msg):
        self.goal_yaw = msg.orientation.z

    def _ordered_positions(self, msg):
        if msg.name:
            position_by_name = dict(zip(msg.name, msg.position))
            return [position_by_name.get(name, self.positions[index]) for index, name in enumerate(JOINT_NAMES)]
        return list(msg.position[:12])


def main(args=None):
    rclpy.init(args=args)
    controller = SpotMicroCrawlController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()