#!/usr/bin/env python3

import numpy as np
import rclpy
import select
import sys
import termios
import tty
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    import pygame
except ModuleNotFoundError:
    pygame = None


class SpotMicroCMDVelPubGUI(Node):
    def __init__(self, width=400, height=400):
        super().__init__("spotmicro_cmdvel_pub_gui")
        pygame.init()
        pygame.font.init()

        self.font = pygame.font.SysFont(None, 24)
        self.running = True
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("SpotMicro CMDVel")

        self.slider_radius = 150
        self.joystick_radius = 20
        self.center = (width / 2, height / 2)
        self.joystick_pos = self.center
        self.active_joystick = False
        self.slider_x_vel = 0.0
        self.slider_yaw_vel = 0.0
        self.max_linear_x = 0.12
        self.max_yaw_rate = 0.25

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.04, self.publish_and_update)

    def draw_joystick(self):
        color = (0, 0, 220)
        pygame.draw.circle(self.screen, color, self.center, self.slider_radius, 2)
        pygame.draw.circle(self.screen, color, self.joystick_pos, self.joystick_radius)
        text_surface = self.font.render("Up/Down: Linear | Left/Right: Angular", True, color)
        text_rect = text_surface.get_rect(center=(self.center[0], self.center[1] - self.slider_radius - 20))
        self.screen.blit(text_surface, text_rect)

    def get_joystick_position(self, mouse_pos):
        dx = mouse_pos[0] - self.center[0]
        dy = mouse_pos[1] - self.center[1]
        distance = np.hypot(dx, dy)
        max_distance = self.slider_radius - self.joystick_radius
        if distance > max_distance:
            angle = np.arctan2(dy, dx)
            dx = max_distance * np.cos(angle)
            dy = max_distance * np.sin(angle)
        return (self.center[0] + dx, self.center[1] + dy)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if np.hypot(event.pos[0] - self.joystick_pos[0], event.pos[1] - self.joystick_pos[1]) <= self.joystick_radius:
                    self.active_joystick = True
            elif event.type == pygame.MOUSEBUTTONUP:
                self.active_joystick = False
                self.slider_x_vel = 0.0
                self.slider_yaw_vel = 0.0
                self.joystick_pos = self.center
            elif event.type == pygame.MOUSEMOTION and self.active_joystick:
                self.joystick_pos = self.get_joystick_position(event.pos)
                max_distance = self.slider_radius - self.joystick_radius
                self.slider_yaw_vel = (self.joystick_pos[0] - self.center[0]) / max_distance
                self.slider_x_vel = (self.joystick_pos[1] - self.center[1]) / max_distance
        return True

    def publish_and_update(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = -self.slider_x_vel * self.max_linear_x
        cmd_vel_msg.angular.z = -self.slider_yaw_vel * self.max_yaw_rate
        self.publisher.publish(cmd_vel_msg)

        self.running = self.handle_events()
        if not self.running:
            rclpy.shutdown()
            return
        self.screen.fill((255, 255, 255))
        self.draw_joystick()
        pygame.display.flip()


class SpotMicroKeyboardCMDVelPublisher(Node):
    def __init__(self):
        super().__init__("spotmicro_keyboard_cmdvel_pub")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.get_logger().info("pygame is not available. Keyboard mode: w/s linear, a/d angular, space stop, q quit.")
        self.timer = self.create_timer(0.05, self.read_keyboard_and_publish)

    def read_keyboard_and_publish(self):
        if select.select([sys.stdin], [], [], 0.0)[0]:
            event_key = sys.stdin.read(1)
            if event_key == "w":
                self.linear_x = min(self.linear_x + 0.05, 0.5)
            elif event_key == "s":
                self.linear_x = max(self.linear_x - 0.05, -0.5)
            elif event_key == "a":
                self.angular_z = min(self.angular_z + 0.1, 1.0)
            elif event_key == "d":
                self.angular_z = max(self.angular_z - 0.1, -1.0)
            elif event_key == " ":
                self.linear_x = 0.0
                self.angular_z = 0.0
            elif event_key == "q":
                rclpy.shutdown()
                return

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.linear_x
        cmd_vel_msg.angular.z = self.angular_z
        self.publisher.publish(cmd_vel_msg)

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = SpotMicroCMDVelPubGUI() if pygame is not None else SpotMicroKeyboardCMDVelPublisher()
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