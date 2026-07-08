#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

try:
    import pygame
except ModuleNotFoundError:
    pygame = None


JOINTS = [
    ("front_left_shoulder", "FL shoulder"),
    ("front_left_leg", "FL leg"),
    ("front_left_foot", "FL foot"),
    ("front_right_shoulder", "FR shoulder"),
    ("front_right_leg", "FR leg"),
    ("front_right_foot", "FR foot"),
    ("rear_left_shoulder", "RL shoulder"),
    ("rear_left_leg", "RL leg"),
    ("rear_left_foot", "RL foot"),
    ("rear_right_shoulder", "RR shoulder"),
    ("rear_right_leg", "RR leg"),
    ("rear_right_foot", "RR foot"),
]


class SpotMicroJointForceGUI(Node):
    def __init__(self, width=900, height=560):
        super().__init__("spotmicro_joint_force_gui")
        self.declare_parameter("max_force", 120.0)
        self.declare_parameter("publish_rate", 25.0)

        self.max_force = float(self.get_parameter("max_force").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.width = width
        self.height = height
        self.running = True
        self.active_joint = None

        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("SpotMicro Joint Forces")
        self.font = pygame.font.SysFont(None, 24)
        self.small_font = pygame.font.SysFont(None, 20)

        self.values = {joint_name: 0.0 for joint_name, _ in JOINTS}
        self.joint_publishers = {
            joint_name: self.create_publisher(Float64, f"/{joint_name}_cmd", 10)
            for joint_name, _ in JOINTS
        }
        self.slider_rects = {}
        self.zero_button = pygame.Rect(self.width - 150, 20, 120, 34)

        self.get_logger().info(
            f"Publishing joint force commands with max_force={self.max_force} N on /<joint>_cmd topics"
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_and_update)

    def slider_geometry(self, index):
        column = index // 6
        row = index % 6
        x = 70 + column * 430
        y = 100 + row * 64
        width = 300
        return pygame.Rect(x, y, width, 8)

    def value_from_mouse(self, slider_rect, mouse_x):
        clamped_x = max(slider_rect.left, min(mouse_x, slider_rect.right))
        normalized = (clamped_x - slider_rect.left) / slider_rect.width
        return (normalized * 2.0 - 1.0) * self.max_force

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.zero_all()
                return False
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    self.zero_all()
                    return False
                if event.key in (pygame.K_SPACE, pygame.K_z):
                    self.zero_all()
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if self.zero_button.collidepoint(event.pos):
                    self.zero_all()
                    continue
                for joint_name, slider_rect in self.slider_rects.items():
                    grab_rect = slider_rect.inflate(26, 32)
                    if grab_rect.collidepoint(event.pos):
                        self.active_joint = joint_name
                        self.values[joint_name] = self.value_from_mouse(slider_rect, event.pos[0])
                        break
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self.active_joint = None
            elif event.type == pygame.MOUSEMOTION and self.active_joint is not None:
                slider_rect = self.slider_rects[self.active_joint]
                self.values[self.active_joint] = self.value_from_mouse(slider_rect, event.pos[0])
        return True

    def zero_all(self):
        for joint_name in self.values:
            self.values[joint_name] = 0.0
        self.publish_values()

    def publish_values(self):
        for joint_name, value in self.values.items():
            msg = Float64()
            msg.data = float(value)
            self.joint_publishers[joint_name].publish(msg)

    def draw_button(self):
        pygame.draw.rect(self.screen, (20, 20, 20), self.zero_button, border_radius=6)
        label = self.font.render("Zero all", True, (255, 255, 255))
        label_rect = label.get_rect(center=self.zero_button.center)
        self.screen.blit(label, label_rect)

    def draw_slider(self, index, joint_name, label):
        slider_rect = self.slider_geometry(index)
        self.slider_rects[joint_name] = slider_rect
        value = self.values[joint_name]
        normalized = (value / self.max_force + 1.0) / 2.0 if self.max_force else 0.5
        knob_x = slider_rect.left + normalized * slider_rect.width
        center_y = slider_rect.centery

        text = self.font.render(label, True, (35, 35, 35))
        self.screen.blit(text, (slider_rect.left, slider_rect.top - 28))
        value_text = self.small_font.render(f"{value:7.1f}", True, (35, 35, 35))
        self.screen.blit(value_text, (slider_rect.right + 14, slider_rect.top - 6))

        pygame.draw.rect(self.screen, (185, 185, 185), slider_rect, border_radius=4)
        pygame.draw.line(self.screen, (80, 80, 80), (slider_rect.centerx, center_y - 10), (slider_rect.centerx, center_y + 10), 2)
        pygame.draw.circle(self.screen, (25, 95, 210), (int(knob_x), center_y), 12)

    def draw(self):
        self.screen.fill((245, 246, 248))
        title = self.font.render("SpotMicro joint force commands", True, (20, 20, 20))
        self.screen.blit(title, (30, 28))
        hint = self.small_font.render("Drag sliders. Space/Z resets. Esc/Q exits after zeroing commands.", True, (75, 75, 75))
        self.screen.blit(hint, (30, 54))
        self.draw_button()

        for index, (joint_name, label) in enumerate(JOINTS):
            self.draw_slider(index, joint_name, label)
        pygame.display.flip()

    def publish_and_update(self):
        self.running = self.handle_events()
        self.publish_values()
        if not self.running:
            rclpy.shutdown()
            return
        self.draw()


class SpotMicroJointForceNoGUI(Node):
    def __init__(self):
        super().__init__("spotmicro_joint_force_no_gui")
        self.get_logger().error("pygame is not available. Install pygame to use the SpotMicro joint force GUI.")
        self.timer = self.create_timer(0.5, rclpy.shutdown)


def main(args=None):
    rclpy.init(args=args)
    controller = SpotMicroJointForceGUI() if pygame is not None else SpotMicroJointForceNoGUI()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        if isinstance(controller, SpotMicroJointForceGUI):
            controller.zero_all()
    controller.destroy_node()
    if pygame is not None:
        pygame.quit()
    rclpy.shutdown()


if __name__ == "__main__":
    main()