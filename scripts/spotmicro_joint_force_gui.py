#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


@dataclass(frozen=True)
class JointConfig:
    name: str
    label: str
    minimum: float
    maximum: float
    kp: float
    kd: float
    max_force: float


JOINTS = [
    JointConfig("front_left_shoulder", "FL shoulder", -0.548, 0.548, 0.25, 0.08, 0.12),
    JointConfig("front_left_leg", "FL leg", -2.666, 1.548, 0.35, 0.10, 0.18),
    JointConfig("front_left_foot", "FL foot", -2.600, 0.100, 0.30, 0.10, 0.15),
    JointConfig("front_right_shoulder", "FR shoulder", -0.548, 0.548, 0.25, 0.08, 0.12),
    JointConfig("front_right_leg", "FR leg", -2.666, 1.548, 0.35, 0.10, 0.18),
    JointConfig("front_right_foot", "FR foot", -2.600, 0.100, 0.30, 0.10, 0.15),
    JointConfig("rear_left_shoulder", "RL shoulder", -0.548, 0.548, 0.25, 0.08, 0.12),
    JointConfig("rear_left_leg", "RL leg", -2.666, 1.548, 0.35, 0.10, 0.18),
    JointConfig("rear_left_foot", "RL foot", -2.600, 0.100, 0.30, 0.10, 0.15),
    JointConfig("rear_right_shoulder", "RR shoulder", -0.548, 0.548, 0.25, 0.08, 0.12),
    JointConfig("rear_right_leg", "RR leg", -2.666, 1.548, 0.35, 0.10, 0.18),
    JointConfig("rear_right_foot", "RR foot", -2.600, 0.100, 0.30, 0.10, 0.15),
]


class SpotMicroJointGui(Node):
    def __init__(self) -> None:
        super().__init__("spotmicro_joint_force_gui")

        self.declare_parameter("gain_scale", 1.0)
        self.declare_parameter("torque_scale", 1.0)
        self.declare_parameter("force_ramp", 0.01)

        self.position: Dict[str, float] = {}
        self.velocity: Dict[str, float] = {}
        self.target: Dict[str, float] = {
            joint.name: self._clamp(0.0, joint.minimum, joint.maximum) for joint in JOINTS
        }
        self.active: Dict[str, bool] = {joint.name: False for joint in JOINTS}
        self.last_force: Dict[str, float] = {joint.name: 0.0 for joint in JOINTS}

        self.joint_publishers = {
            joint.name: self.create_publisher(Float64, f"/{joint.name}_cmd", 10)
            for joint in JOINTS
        }
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)

    def _joint_state_callback(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self.position[name] = msg.position[index]
                if name in self.target and not self.active[name]:
                    joint = self._joint_config(name)
                    self.target[name] = self._clamp(msg.position[index], joint.minimum, joint.maximum)
            if index < len(msg.velocity):
                self.velocity[name] = msg.velocity[index]

    def set_target(self, joint: JointConfig, value: float) -> None:
        self.target[joint.name] = self._clamp(value, joint.minimum, joint.maximum)
        self.active[joint.name] = True

    def release_all(self) -> None:
        for joint in JOINTS:
            self.active[joint.name] = False
            self.last_force[joint.name] = 0.0
            self.joint_publishers[joint.name].publish(Float64(data=0.0))

    def center_targets_without_force(self) -> None:
        for joint in JOINTS:
            self.target[joint.name] = self._clamp(0.0, joint.minimum, joint.maximum)
            self.active[joint.name] = False

    def publish_forces(self) -> None:
        gain_scale = float(self.get_parameter("gain_scale").value)
        torque_scale = float(self.get_parameter("torque_scale").value)
        force_ramp = abs(float(self.get_parameter("force_ramp").value))

        for joint in JOINTS:
            desired_force = 0.0
            if self.active[joint.name] and joint.name in self.position:
                error = self.target[joint.name] - self.position[joint.name]
                damping = self.velocity.get(joint.name, 0.0)
                desired_force = (joint.kp * gain_scale * error) - (joint.kd * gain_scale * damping)
                desired_force = self._clamp(
                    desired_force,
                    -joint.max_force * torque_scale,
                    joint.max_force * torque_scale,
                )

            previous_force = self.last_force[joint.name]
            force_delta = self._clamp(desired_force - previous_force, -force_ramp, force_ramp)
            force = previous_force + force_delta
            if not self.active[joint.name] and abs(force) < force_ramp:
                force = 0.0

            self.last_force[joint.name] = force
            self.joint_publishers[joint.name].publish(Float64(data=force))

    def active_count(self) -> int:
        return sum(1 for enabled in self.active.values() if enabled)

    @staticmethod
    def _joint_config(name: str) -> JointConfig:
        for joint in JOINTS:
            if joint.name == name:
                return joint
        raise KeyError(name)

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))


class Slider:
    def __init__(self, joint: JointConfig, rect: pygame.Rect) -> None:
        self.joint = joint
        self.rect = rect

    def hit(self, point: Tuple[int, int]) -> bool:
        return self.rect.inflate(12, 18).collidepoint(point)

    def value_from_x(self, x_position: int) -> float:
        ratio = (x_position - self.rect.left) / self.rect.width
        ratio = max(0.0, min(1.0, ratio))
        return self.joint.minimum + ratio * (self.joint.maximum - self.joint.minimum)

    def knob_x(self, value: float) -> int:
        ratio = (value - self.joint.minimum) / (self.joint.maximum - self.joint.minimum)
        return int(self.rect.left + ratio * self.rect.width)


def draw_text(surface, font, text, position, color=(225, 225, 225)) -> None:
    surface.blit(font.render(text, True, color), position)


def draw_gui(screen, fonts, node: SpotMicroJointGui, sliders) -> None:
    screen.fill((25, 27, 31))
    title_font, body_font, small_font = fonts

    status = f"{node.active_count()} active"
    draw_text(screen, title_font, "SpotMicro Joint Sliders", (24, 16))
    draw_text(screen, body_font, status, (500, 23), (116, 217, 137))
    draw_text(
        screen,
        small_font,
        "Drag one slider slowly. M: release all forces   R: center sliders only   Esc: quit",
        (24, 48),
        (170, 174, 184),
    )

    for slider in sliders:
        joint = slider.joint
        target = node.target[joint.name]
        measured = node.position.get(joint.name)
        knob_x = slider.knob_x(target)
        is_active = node.active[joint.name]

        track_color = (82, 91, 105) if is_active else (62, 66, 76)
        fill_color = (107, 160, 255) if is_active else (97, 105, 119)
        pygame.draw.rect(screen, track_color, slider.rect, border_radius=3)
        pygame.draw.rect(
            screen,
            fill_color,
            pygame.Rect(slider.rect.left, slider.rect.top, knob_x - slider.rect.left, slider.rect.height),
            border_radius=3,
        )
        pygame.draw.circle(screen, (238, 239, 242), (knob_x, slider.rect.centery), 8)

        if measured is not None:
            measured_x = slider.knob_x(SpotMicroJointGui._clamp(measured, joint.minimum, joint.maximum))
            pygame.draw.line(
                screen,
                (255, 214, 102),
                (measured_x, slider.rect.top - 7),
                (measured_x, slider.rect.bottom + 7),
                2,
            )

        draw_text(screen, body_font, joint.label, (24, slider.rect.top - 5))
        draw_text(screen, small_font, f"{joint.minimum:.2f}", (176, slider.rect.top + 20), (160, 164, 173))
        draw_text(screen, small_font, f"{joint.maximum:.2f}", (520, slider.rect.top + 20), (160, 164, 173))

        measured_text = "--" if measured is None else f"{measured:.2f}"
        force_text = f"force {node.last_force[joint.name]:.3f}"
        value_text = f"target {target:.2f}   actual {measured_text} rad   {force_text}"
        draw_text(screen, small_font, value_text, (620, slider.rect.top - 1), (204, 207, 214))

    pygame.display.flip()


def main() -> None:
    rclpy.init()
    node = SpotMicroJointGui()

    pygame.init()
    pygame.display.set_caption("SpotMicro Joint Sliders")
    screen = pygame.display.set_mode((940, 670))
    fonts = (
        pygame.font.SysFont("sans", 26, bold=True),
        pygame.font.SysFont("sans", 18),
        pygame.font.SysFont("sans", 15),
    )

    sliders = [
        Slider(joint, pygame.Rect(180, 88 + index * 46, 380, 8))
        for index, joint in enumerate(JOINTS)
    ]

    clock = pygame.time.Clock()
    dragging: Optional[Slider] = None
    running = True

    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_m:
                        node.release_all()
                    elif event.key == pygame.K_r:
                        node.center_targets_without_force()
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    for slider in sliders:
                        if slider.hit(event.pos):
                            dragging = slider
                            node.set_target(slider.joint, slider.value_from_x(event.pos[0]))
                            break
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    dragging = None
                elif event.type == pygame.MOUSEMOTION and dragging is not None:
                    node.set_target(dragging.joint, dragging.value_from_x(event.pos[0]))

            node.publish_forces()
            draw_gui(screen, fonts, node, sliders)
            clock.tick(60)
    finally:
        node.release_all()
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
