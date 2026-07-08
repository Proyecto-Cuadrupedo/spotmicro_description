#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState

from spotmicro_high_level_controller import JOINT_NAMES
from spotmicro_kinematics import standing_angles


LEG_LIMITS = [(-0.548, 0.548), (-2.666, 1.548), (-2.6, 0.1)] * 4


class SpotMicroPoseController(Node):
    def __init__(self):
        super().__init__("spotmicro_pose_controller")
        self.freq = 120
        self.positions = list(standing_angles())
        self.goal_pos = standing_angles()
        self.hold_theta = list(standing_angles())
        self.goal_roll = self.roll = 0.0
        self.goal_pitch = self.pitch = 0.0
        self.goal_yaw = self.yaw = 0.0

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
        self.publish_joint_command(self.hold_theta)
        self.timer = self.create_timer(1.0 / self.freq, self.update)

    def update(self):
        roll_error = (self.goal_roll - self.roll) / 100.0
        pitch_error = (self.goal_pitch - self.pitch) / 100.0
        yaw_error = (self.goal_yaw - self.yaw) / 100.0
        command = list(self.hold_theta)

        for leg_index in range(4):
            base = leg_index * 3
            side_sign = -1.0 if leg_index % 2 == 0 else 1.0
            front_sign = 1.0 if leg_index < 2 else -1.0
            command[base] += side_sign * roll_error * 16.0 + side_sign * yaw_error * 8.0
            command[base + 1] += front_sign * pitch_error * 16.0
            command[base + 2] -= abs(pitch_error) * 6.0

        self.roll += roll_error
        self.pitch += pitch_error
        self.yaw += yaw_error
        self.goal_pos = np.array([self.clamp_joint(index, value) for index, value in enumerate(command)], dtype=float)
        self.publish_joint_command(self.goal_pos)

    def clamp_joint(self, index, value):
        lower, upper = LEG_LIMITS[index]
        return max(lower, min(upper, value))

    def publish_joint_command(self, position):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.position = list(position)
        self.pub.publish(self.msg)

    def joint_states_callback(self, msg):
        self.positions = self._ordered_positions(msg)

    def goal_pos_callback(self, msg):
        self.goal_roll = msg.orientation.x
        self.goal_pitch = msg.orientation.y
        self.goal_yaw = msg.orientation.z

    def _ordered_positions(self, msg):
        if msg.name:
            position_by_name = dict(zip(msg.name, msg.position))
            return [position_by_name.get(name, self.positions[index]) for index, name in enumerate(JOINT_NAMES)]
        return list(msg.position[:12])


def main(args=None):
    rclpy.init(args=args)
    controller = SpotMicroPoseController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()