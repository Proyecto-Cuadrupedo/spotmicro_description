#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

try:
    import pygame
except ModuleNotFoundError:
    pygame = None


class SpotMicroPosePubGUI(Node):
    def __init__(self, width=400, height=300):
        super().__init__("spotmicro_pose_pub_gui")
        pygame.init()
        pygame.font.init()

        self.font = pygame.font.SysFont(None, 24)
        self.running = True
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("SpotMicro Pose")

        self.slider_radius = 50
        self.joystick_radius = 15
        self.centers = [(width * 1 / 5, height / 2), (width / 2, height / 2), (width * 4 / 5, height / 2)]
        self.joystick_positions = list(self.centers)
        self.active_joystick = None

        self.base_height = 0.185
        self.base_width = 0.055
        self.slider_height = self.base_height
        self.slider_width = self.base_width
        self.slider_length = 0.0
        self.slider_roll = 0.0
        self.slider_pitch = 0.0
        self.slider_yaw = 0.0

        self.publisher = self.create_publisher(Pose, "/goal_pose", 10)
        self.timer = self.create_timer(0.04, self.publish_and_update)

    def draw_joystick(self, index, color, label):
        center = self.centers[index]
        pygame.draw.circle(self.screen, color, center, self.slider_radius, 2)
        pygame.draw.circle(self.screen, color, self.joystick_positions[index], self.joystick_radius)
        text_surface = self.font.render(label, True, color)
        text_rect = text_surface.get_rect(center=(center[0], center[1] - self.slider_radius - 20))
        self.screen.blit(text_surface, text_rect)

    def get_joystick_position(self, center, mouse_pos):
        dx = mouse_pos[0] - center[0]
        dy = mouse_pos[1] - center[1]
        distance = np.hypot(dx, dy)
        max_distance = self.slider_radius - self.joystick_radius
        if distance > max_distance:
            angle = np.arctan2(dy, dx)
            dx = max_distance * np.cos(angle)
            dy = max_distance * np.sin(angle)
        return (center[0] + dx, center[1] + dy)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.MOUSEBUTTONDOWN:
                for index, position in enumerate(self.joystick_positions):
                    if np.hypot(event.pos[0] - position[0], event.pos[1] - position[1]) <= self.joystick_radius:
                        self.active_joystick = index
                        break
            elif event.type == pygame.MOUSEBUTTONUP:
                self.active_joystick = None
                self.slider_roll = self.slider_pitch = self.slider_yaw = 0.0
                self.slider_height = self.base_height
                self.slider_width = self.base_width
                self.slider_length = 0.0
                self.joystick_positions = list(self.centers)
            elif event.type == pygame.MOUSEMOTION and self.active_joystick is not None:
                center = self.centers[self.active_joystick]
                position = self.get_joystick_position(center, event.pos)
                self.joystick_positions[self.active_joystick] = position
                max_distance = self.slider_radius - self.joystick_radius
                if self.active_joystick == 0:
                    self.slider_roll = ((position[0] - center[0]) / max_distance) / 2.0
                    self.slider_length = ((position[1] - center[1]) / max_distance) / 20.0
                elif self.active_joystick == 1:
                    self.slider_yaw = ((position[0] - center[0]) / max_distance) / 2.0
                    self.slider_height = self.base_height - ((position[1] - center[1]) / max_distance) / 10.0
                else:
                    self.slider_pitch = ((position[0] - center[0]) / max_distance) / 2.0
                    self.slider_width = self.base_width + ((position[1] - center[1]) / max_distance) / 20.0
        return True

    def publish_and_update(self):
        pose_msg = Pose()
        pose_msg.position.x = self.slider_width
        pose_msg.position.y = self.slider_height
        pose_msg.position.z = self.slider_length
        pose_msg.orientation.x = self.slider_roll
        pose_msg.orientation.y = self.slider_pitch
        pose_msg.orientation.z = self.slider_yaw
        pose_msg.orientation.w = 0.0
        self.publisher.publish(pose_msg)

        self.running = self.handle_events()
        if not self.running:
            rclpy.shutdown()
            return
        self.screen.fill((255, 255, 255))
        self.draw_joystick(0, (0, 0, 220), "Roll/Move X")
        self.draw_joystick(1, (220, 0, 0), "Yaw/Height")
        self.draw_joystick(2, (0, 160, 0), "Pitch/Width")
        pygame.display.flip()


class SpotMicroNeutralPosePublisher(Node):
    def __init__(self):
        super().__init__("spotmicro_neutral_pose_pub")
        self.publisher = self.create_publisher(Pose, "/goal_pose", 10)
        self.get_logger().info("pygame is not available. Publishing neutral /goal_pose; install pygame for the GUI.")
        self.timer = self.create_timer(0.1, self.publish_neutral_pose)

    def publish_neutral_pose(self):
        pose_msg = Pose()
        pose_msg.position.x = 0.055
        pose_msg.position.y = 0.185
        pose_msg.position.z = 0.0
        pose_msg.orientation.w = 0.0
        self.publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = SpotMicroPosePubGUI() if pygame is not None else SpotMicroNeutralPosePublisher()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    if pygame is not None:
        pygame.quit()
    rclpy.shutdown()


if __name__ == "__main__":
    main()