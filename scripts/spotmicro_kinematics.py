#!/usr/bin/env python3

import math

import numpy as np


SHOULDER_OFFSET = 0.055
UPPER_LEG = 0.1075
LOWER_LEG = 0.130

SHOULDER_MIN = -0.548
SHOULDER_MAX = 0.548
LEG_MIN = -2.666
LEG_MAX = 1.548
FOOT_MIN = -2.6
FOOT_MAX = 0.1


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def forward_kinematics(theta0, theta1, theta2, is_left=True):
    side = -1.0 if is_left else 1.0
    lateral = side * SHOULDER_OFFSET
    reach = UPPER_LEG * math.sin(theta1) + LOWER_LEG * math.sin(theta1 + theta2)
    height = UPPER_LEG * math.cos(theta1) + LOWER_LEG * math.cos(theta1 + theta2)

    x = lateral * math.cos(theta0) + reach * math.sin(theta0)
    y = lateral * math.sin(theta0) - reach * math.cos(theta0)
    z = height
    return [x, y, z]


def _candidate_angles(position, is_left=True):
    x, y, z = position
    side_offset = -SHOULDER_OFFSET if is_left else SHOULDER_OFFSET
    radial_sq = x * x + y * y - side_offset * side_offset
    if radial_sq < 0.0:
        return []

    radial = math.sqrt(radial_sq)
    theta0_candidates = []
    for signed_radial in (radial, -radial):
        c0 = side_offset * x - y * signed_radial
        s0 = side_offset * y + x * signed_radial
        theta0_candidates.append(wrap_to_pi(math.atan2(s0, c0)))

    cos_knee = (radial_sq + z * z - UPPER_LEG ** 2 - LOWER_LEG ** 2) / (2.0 * UPPER_LEG * LOWER_LEG)
    if cos_knee < -1.0 or cos_knee > 1.0:
        return []

    candidates = []
    for theta2 in (math.acos(cos_knee), -math.acos(cos_knee)):
        for signed_radial in (radial, -radial):
            theta1 = math.atan2(signed_radial, z) - math.atan2(
                LOWER_LEG * math.sin(theta2),
                UPPER_LEG + LOWER_LEG * math.cos(theta2),
            )
            motor_theta2 = theta2
            for theta0 in theta0_candidates:
                candidate = [theta0, theta1, motor_theta2]
                if _within_limits(candidate):
                    candidates.append(candidate)

    return candidates


def _within_limits(angles):
    return (
        SHOULDER_MIN <= angles[0] <= SHOULDER_MAX
        and LEG_MIN <= angles[1] <= LEG_MAX
        and FOOT_MIN <= angles[2] <= FOOT_MAX
    )


def calc_joint_angles(position, is_left=True):
    return _candidate_angles(position, is_left)


def calc_correct_thetas(position, previous_thetas, is_left=True):
    possible_joint_angles = calc_joint_angles(position, is_left)
    if not possible_joint_angles:
        return list(previous_thetas)

    return min(
        possible_joint_angles,
        key=lambda angles: np.sum(np.abs(np.array(angles) - np.array(previous_thetas))),
    )


def standing_angles():
    return np.array([0.0, 0.0, 0.0] * 4)


def laying_down_angles():
    return np.array([0.0, 1.45, -2.35] * 4)