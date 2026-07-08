#!/usr/bin/env python3

import math
from dataclasses import dataclass


JOINT_LIMITS = [(-0.548, 0.548), (-2.666, 1.548), (-2.6, 0.1)] * 4


@dataclass
class Point:
    x: float
    y: float
    z: float


class SpotMicroReferenceController:
    def __init__(self):
        self.hip_link_length = 0.055
        self.upper_leg_link_length = 0.1075
        self.lower_leg_link_length = 0.130
        self.body_width = 0.078
        self.body_length = 0.186

        self.default_stand_height = 0.155
        self.stand_front_x_offset = 0.015
        self.stand_back_x_offset = 0.0
        self.max_fwd_velocity = 0.4
        self.max_side_velocity = 0.4
        self.max_yaw_rate = 0.35

        self.z_clearance = 0.050
        self.alpha = 0.5
        self.beta = 0.5
        self.dt = 0.02
        self.swing_time = 0.36
        self.swing_ticks = round(self.swing_time / self.dt)
        self.stance_ticks = 7 * self.swing_ticks
        self.phase_length = 8 * self.swing_ticks
        self.foot_height_time_constant = 0.02

        self.rb_contact_phases = [1, 0, 1, 1, 1, 1, 1, 1]
        self.rf_contact_phases = [1, 1, 1, 0, 1, 1, 1, 1]
        self.lf_contact_phases = [1, 1, 1, 1, 1, 1, 1, 0]
        self.lb_contact_phases = [1, 1, 1, 1, 1, 0, 1, 1]

        self.ticks = 0
        self.foot_positions = self.neutral_stance()

    def neutral_stance(self):
        half_length = self.body_length / 2.0
        half_width = self.body_width / 2.0
        lateral = half_width + self.hip_link_length
        return {
            "rear_right": Point(-half_length + self.stand_back_x_offset, 0.0, lateral),
            "front_right": Point(half_length + self.stand_front_x_offset, 0.0, lateral),
            "front_left": Point(half_length + self.stand_front_x_offset, 0.0, -lateral),
            "rear_left": Point(-half_length + self.stand_back_x_offset, 0.0, -lateral),
        }

    def stand_joint_angles(self):
        return self.foot_positions_to_joint_commands(self.neutral_stance())

    def step(self, x_speed, y_speed, yaw_rate):
        x_speed = self._clip(x_speed, -self.max_fwd_velocity, self.max_fwd_velocity)
        y_speed = self._clip(y_speed, -self.max_side_velocity, self.max_side_velocity)
        yaw_rate = self._clip(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)

        if abs(x_speed) < 1e-3 and abs(y_speed) < 1e-3 and abs(yaw_rate) < 1e-3:
            self.ticks = 0
            self.foot_positions = self.neutral_stance()
            return self.stand_joint_angles()

        phase_index = (self.ticks % self.phase_length) // self.swing_ticks
        subphase_ticks = (self.ticks % self.phase_length) % self.swing_ticks
        default_stance = self.neutral_stance()

        contact_phase_map = {
            "rear_right": self.rb_contact_phases,
            "front_right": self.rf_contact_phases,
            "front_left": self.lf_contact_phases,
            "rear_left": self.lb_contact_phases,
        }

        new_positions = {}
        for leg_name, foot_pos in self.foot_positions.items():
            in_swing = contact_phase_map[leg_name][phase_index] == 0
            if in_swing:
                swing_proportion = subphase_ticks / max(1, self.swing_ticks)
                new_positions[leg_name] = self._swing_leg_controller(
                    foot_pos,
                    default_stance[leg_name],
                    x_speed,
                    y_speed,
                    yaw_rate,
                    swing_proportion,
                )
            else:
                new_positions[leg_name] = self._stance_controller(foot_pos, x_speed, y_speed, yaw_rate)

        self.foot_positions = new_positions
        self.ticks = (self.ticks + 1) % self.phase_length
        return self.foot_positions_to_joint_commands(self.foot_positions)

    def _stance_controller(self, foot_pos, x_speed, y_speed, yaw_rate):
        delta_x = -x_speed * self.dt
        delta_y = (1.0 / self.foot_height_time_constant) * (0.0 - foot_pos.y) * self.dt
        delta_z = -y_speed * self.dt
        rotated = self._rotate_y(foot_pos, yaw_rate * self.dt)
        return Point(rotated.x + delta_x, rotated.y + delta_y, rotated.z + delta_z)

    def _swing_leg_controller(self, foot_pos, default_foot_pos, x_speed, y_speed, yaw_rate, swing_proportion):
        if swing_proportion < 0.5:
            swing_height = (swing_proportion / 0.5) * self.z_clearance
        else:
            swing_height = self.z_clearance * (1.0 - (swing_proportion - 0.5) / 0.5)

        delta = Point(
            self.alpha * self.stance_ticks * self.dt * x_speed,
            0.0,
            self.alpha * self.stance_ticks * self.dt * y_speed,
        )
        touchdown = self._rotate_y(default_foot_pos, self.beta * self.stance_ticks * self.dt * -yaw_rate)
        touchdown = Point(touchdown.x + delta.x, touchdown.y + delta.y, touchdown.z + delta.z)

        time_left = max(self.dt, self.dt * self.swing_ticks * (1.0 - swing_proportion))
        return Point(
            foot_pos.x + ((touchdown.x - foot_pos.x) / time_left) * self.dt,
            swing_height,
            foot_pos.z + ((touchdown.z - foot_pos.z) / time_left) * self.dt,
        )

    def foot_positions_to_joint_commands(self, feet):
        leg_order = ["front_left", "front_right", "rear_left", "rear_right"]
        commands = []
        for leg_name in leg_order:
            commands.extend(self._leg_ik_to_urdf_joints(leg_name, feet[leg_name]))
        return [self._clip(value, *JOINT_LIMITS[index]) for index, value in enumerate(commands)]

    def _leg_ik_to_urdf_joints(self, leg_name, foot_pos):
        shoulder_x = self.body_length / 2.0 if "front" in leg_name else -self.body_length / 2.0
        side = 1.0 if "right" in leg_name else -1.0
        shoulder_z = side * self.body_width / 2.0

        local_x = foot_pos.x - shoulder_x
        local_y = foot_pos.y - self.default_stand_height
        local_z = foot_pos.z - shoulder_z

        vertical = max(0.02, -local_y)
        lateral_error = local_z - side * self.hip_link_length
        shoulder = side * math.atan2(lateral_error, vertical)

        sagittal_height = math.sqrt(vertical * vertical + lateral_error * lateral_error)
        leg = self._sagittal_hip_angle(local_x, sagittal_height)
        foot = self._sagittal_knee_angle(local_x, sagittal_height)
        return [shoulder, leg, foot]

    def _sagittal_knee_angle(self, x, height):
        upper = self.upper_leg_link_length
        lower = self.lower_leg_link_length
        cos_knee = (x * x + height * height - upper * upper - lower * lower) / (2.0 * upper * lower)
        cos_knee = self._clip(cos_knee, -1.0, 1.0)
        return -math.acos(cos_knee)

    def _sagittal_hip_angle(self, x, height):
        upper = self.upper_leg_link_length
        lower = self.lower_leg_link_length
        knee = self._sagittal_knee_angle(x, height)
        return -(
            math.atan2(x, height)
            - math.atan2(lower * math.sin(knee), upper + lower * math.cos(knee))
        )

    def _rotate_y(self, point, yaw):
        return Point(
            math.cos(yaw) * point.x + math.sin(yaw) * point.z,
            point.y,
            -math.sin(yaw) * point.x + math.cos(yaw) * point.z,
        )

    def _clip(self, value, lower, upper):
        return max(lower, min(upper, value))