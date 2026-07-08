#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState

from spotmicro_reference import JOINT_LIMITS, SpotMicroReferenceController


JOINT_NAMES = [
    "front_left_shoulder", "front_left_leg", "front_left_foot",
    "front_right_shoulder", "front_right_leg", "front_right_foot",
    "rear_left_shoulder", "rear_left_leg", "rear_left_foot",
    "rear_right_shoulder", "rear_right_leg", "rear_right_foot",
]


class SpotMicroHighLevelController(Node):
    def __init__(self):
        super().__init__("spotmicro_high_level_controller")
        self.freq = 120
        self.reference_controller = SpotMicroReferenceController()
        self.reference_stand_theta = self.reference_controller.stand_joint_angles()
        self.positions = self.reference_controller.stand_joint_angles()
        self.velocities = [0.0] * 12
        self.desired_theta = self.reference_controller.stand_joint_angles()
        self.hold_theta = self.reference_controller.stand_joint_angles()
        self.linear_vel = [0.0, 0.0]
        self.linear_cmd_vel = [0.0, 0.0]
        self.angular_cmd_vel = 0.0
        self.tick = 0
        self.command_deadband = 0.01
        self.walk_blend = 0.0

        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, "/spotmicro/odometry", self.odom_callback, 10)
        self.pub = self.create_publisher(JointState, "/high_level/joint_cmd", 10)
        self.msg = JointState()
        self.msg.name = JOINT_NAMES

        ok, first_state = wait_for_message(JointState, self, "/joint_states", time_to_wait=2)
        if ok:
            self.joint_states_callback(first_state)
        self.hold_theta = list(self.positions)
        self.desired_theta = list(self.hold_theta)
        self.publish_joint_command(self.desired_theta)
        self.timer = self.create_timer(1.0 / self.freq, self.update)

    def update(self):
        if self.is_standing_command():
            self.walk_blend = 0.0
            self.desired_theta = list(self.hold_theta)
            self.publish_joint_command(self.desired_theta)
            return

        reference_theta = self.reference_controller.step(
            self.linear_cmd_vel[0],
            self.linear_cmd_vel[1],
            self.angular_cmd_vel,
        )
        self.walk_blend = min(1.0, self.walk_blend + 0.02)
        self.desired_theta = self.apply_reference_delta(reference_theta)
        self.publish_joint_command(self.desired_theta)

    def apply_reference_delta(self, reference_theta):
        command = []
        for index, (hold, reference, reference_stand) in enumerate(zip(self.hold_theta, reference_theta, self.reference_stand_theta)):
            delta = (reference - reference_stand) * self.walk_blend
            command.append(self.clamp_joint(index, hold + delta))
        return command

    def clamp_joint(self, index, value):
        lower, upper = JOINT_LIMITS[index]
        return max(lower, min(upper, value))

    def is_standing_command(self):
        return (
            abs(self.linear_cmd_vel[0]) < self.command_deadband
            and abs(self.linear_cmd_vel[1]) < self.command_deadband
            and abs(self.angular_cmd_vel) < self.command_deadband
        )

    def publish_joint_command(self, position):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.position = list(position)
        self.pub.publish(self.msg)

    def joint_states_callback(self, msg):
        if msg.name:
            position_by_name = dict(zip(msg.name, msg.position))
            velocity_by_name = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
            self.positions = [position_by_name.get(name, self.positions[index]) for index, name in enumerate(JOINT_NAMES)]
            self.velocities = [velocity_by_name.get(name, 0.0) for name in JOINT_NAMES]
        else:
            self.positions = list(msg.position[:12])
            self.velocities = list(msg.velocity[:12]) if msg.velocity else [0.0] * 12

    def cmd_vel_callback(self, msg):
        self.linear_cmd_vel = [msg.linear.x, msg.linear.y]
        self.angular_cmd_vel = msg.angular.z

    def odom_callback(self, msg):
        self.linear_vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y]


def main(args=None):
    rclpy.init(args=args)
    controller = SpotMicroHighLevelController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()