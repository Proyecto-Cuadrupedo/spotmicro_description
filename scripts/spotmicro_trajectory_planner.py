#!/usr/bin/env python3

import math

import numpy as np


class TrajectoryPlanner:
    def __init__(self, base_height=0.185, base_width=0.055, leg_offset_x=0.093, leg_offset_z=0.039):
        self.base_height = base_height
        self.base_width = base_width
        self.leg_offset_x = leg_offset_x
        self.leg_offset_z = leg_offset_z

    def trot(self, leg_index, position, period, tick, command_vel=None, angular_command_vel=0.0):
        command_vel = command_vel or [0.0, 0.0]
        phase = (tick + (period / 2 if leg_index in (0, 3) else 0)) % period
        progress = phase / period
        step_length = float(np.clip(command_vel[0], -0.25, 0.25)) * 0.18
        lateral_step = float(np.clip(command_vel[1], -0.20, 0.20)) * 0.10
        yaw_step = float(np.clip(angular_command_vel, -1.0, 1.0)) * 0.04

        x = -self.base_width if leg_index % 2 == 0 else self.base_width
        y = self.base_height
        z = 0.0

        if progress < 0.5:
            swing = progress / 0.5
            z = (swing - 0.5) * step_length
            x += (swing - 0.5) * lateral_step
            y -= math.sin(math.pi * swing) * 0.045
        else:
            stance = (progress - 0.5) / 0.5
            z = (0.5 - stance) * step_length
            x += (0.5 - stance) * lateral_step

        body_pos = self.global_foot_pos(leg_index, [x, y, z])
        body_pos = self.apply_rpy(body_pos[0], body_pos[1], body_pos[2], 0.0, 0.0, yaw_step)
        return self.local_foot_pos(leg_index, body_pos)

    def crawl(self, leg_index, position, step_height, step_length, period, tick):
        phase_offsets = [0.33, 0.83, 0.16, 0.66]
        phase = ((tick / period) + phase_offsets[leg_index]) % 1.0
        x = -self.base_width if leg_index % 2 == 0 else self.base_width
        y = self.base_height
        z = 0.0

        if phase < 0.25:
            swing = phase / 0.25
            y -= math.sin(math.pi * swing) * step_height
            z = (swing - 0.5) * step_length
        else:
            stance = (phase - 0.25) / 0.75
            z = (0.5 - stance) * step_length

        return [x, y, z]

    def global_foot_pos(self, leg_index, position):
        x, y, z = position
        if leg_index == 0:
            return [x + self.leg_offset_x, y, z + self.leg_offset_z]
        if leg_index == 1:
            return [x + self.leg_offset_x, y, z - self.leg_offset_z]
        if leg_index == 2:
            return [x - self.leg_offset_x, y, z + self.leg_offset_z]
        return [x - self.leg_offset_x, y, z - self.leg_offset_z]

    def local_foot_pos(self, leg_index, position):
        x, y, z = position
        if leg_index == 0:
            return [x - self.leg_offset_x, y, z - self.leg_offset_z]
        if leg_index == 1:
            return [x - self.leg_offset_x, y, z + self.leg_offset_z]
        if leg_index == 2:
            return [x + self.leg_offset_x, y, z - self.leg_offset_z]
        return [x + self.leg_offset_x, y, z + self.leg_offset_z]

    def apply_rpy(self, x, y, z, roll, pitch, yaw):
        rotate_y = np.matrix([
            [np.cos(yaw), 0, np.sin(yaw)],
            [0, 1, 0],
            [-np.sin(yaw), 0, np.cos(yaw)],
        ])
        rotate_x = np.matrix([
            [1, 0, 0],
            [0, np.cos(pitch), -np.sin(pitch)],
            [0, np.sin(pitch), np.cos(pitch)],
        ])
        rotate_z = np.matrix([
            [np.cos(roll), -np.sin(roll), 0],
            [np.sin(roll), np.cos(roll), 0],
            [0, 0, 1],
        ])
        vector = rotate_x @ rotate_y @ rotate_z @ np.matrix([[x], [y], [z]])
        return [vector.item(0), vector.item(1), vector.item(2)]