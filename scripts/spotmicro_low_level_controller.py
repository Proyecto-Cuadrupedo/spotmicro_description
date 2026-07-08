#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


JOINT_NAMES = [
    "front_left_shoulder", "front_left_leg", "front_left_foot",
    "front_right_shoulder", "front_right_leg", "front_right_foot",
    "rear_left_shoulder", "rear_left_leg", "rear_left_foot",
    "rear_right_shoulder", "rear_right_leg", "rear_right_foot",
]

COMMAND_TOPICS = [f"/{joint_name}_cmd" for joint_name in JOINT_NAMES]


class SpotMicroLowLevelController(Node):
    def __init__(self):
        super().__init__("spotmicro_low_level_controller")
        self.freq = 500
        self.positions = [0.0] * len(JOINT_NAMES)
        self.velocities = [0.0] * len(JOINT_NAMES)
        self.desired_thetas = [0.0] * len(JOINT_NAMES)
        self.cmd_thetas = [0.0] * len(JOINT_NAMES)

        self.create_subscription(JointState, "/high_level/joint_cmd", self.joint_cmd_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)

        ok, msg = wait_for_message(JointState, self, "/joint_states", time_to_wait=5)
        if ok:
            self._update_state_from_msg(msg)
            self.desired_thetas = list(self.positions)

        self.pubs = [self.create_publisher(Float64, topic, 10) for topic in COMMAND_TOPICS]
        self.kp = [70.0, 180.0, 120.0] * 4
        self.kd = [1.0, 2.0, 1.5] * 4
        self.get_logger().info(f"Using SpotMicro PD gains kp={self.kp}, kd={self.kd}")
        self.timer = self.create_timer(1.0 / self.freq, self.update)

    def update(self):
        for index, pub in enumerate(self.pubs):
            self.cmd_thetas[index] = (
                self.kp[index] * (self.desired_thetas[index] - self.positions[index])
                - self.kd[index] * self.velocities[index]
            )
            motor_cmd = Float64()
            motor_cmd.data = self.cmd_thetas[index]
            pub.publish(motor_cmd)

    def joint_cmd_callback(self, msg):
        if msg.name:
            desired_by_name = dict(zip(msg.name, msg.position))
            self.desired_thetas = [desired_by_name.get(name, current) for name, current in zip(JOINT_NAMES, self.desired_thetas)]
        else:
            self.desired_thetas = list(msg.position[:len(JOINT_NAMES)])

    def joint_states_callback(self, msg):
        self._update_state_from_msg(msg)

    def _update_state_from_msg(self, msg):
        if msg.name:
            position_by_name = dict(zip(msg.name, msg.position))
            velocity_by_name = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
            self.positions = [position_by_name.get(name, self.positions[index]) for index, name in enumerate(JOINT_NAMES)]
            self.velocities = [velocity_by_name.get(name, 0.0) for name in JOINT_NAMES]
        else:
            self.positions = list(msg.position[:len(JOINT_NAMES)])
            self.velocities = list(msg.velocity[:len(JOINT_NAMES)]) if msg.velocity else [0.0] * len(JOINT_NAMES)


def main(args=None):
    rclpy.init(args=args)
    controller = SpotMicroLowLevelController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()