# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Joint motor model profiles for the RBY1 robot.

Each profile describes the PhysX friction/effort/armature parameters of the
underlying Dynamixel motor model. Profiles are looked up by joint base name
(stripped of ``left_``/``right_`` prefix).
"""
from __future__ import annotations

from typing import Final


# Motor model parameters keyed by motor family.
JOINT_MOTOR_MODEL_PROFILES: Final[dict[str, dict[str, float]]] = {
    "arm_shoulder": {
        "joint_friction": 0.0,
        "static_effort": 5.2,
        "dynamic_effort": 5.2,
        "viscous_nm_s_per_rad": 4.7,
        "armature": 0.29,
    },
    "arm_elbow": {
        "joint_friction": 0.0,
        "static_effort": 2.2,
        "dynamic_effort": 2.2,
        "viscous_nm_s_per_rad": 1.9,
        "armature": 0.4,
    },
    "arm_wrist": {
        "joint_friction": 0.0,
        "static_effort": 2.2,
        "dynamic_effort": 2.2,
        "viscous_nm_s_per_rad": 1.9,
        "armature": 0.2,
    },
    "arm_wrist2": {
        "joint_friction": 0.0,
        "static_effort": 1.2,
        "dynamic_effort": 1.2,
        "viscous_nm_s_per_rad": 1.0,
        "armature": 0.2,
    },
    "lower_torso": {
        "joint_friction": 0.0,
        "static_effort": 10.0,
        "dynamic_effort": 10.0,
        "viscous_nm_s_per_rad": 20.0,
        "armature": 1.7,
    },
    "upper_torso": {
        "joint_friction": 0.0,
        "static_effort": 10.0,
        "dynamic_effort": 10.0,
        "viscous_nm_s_per_rad": 10.0,
        "armature": 1.7,
    },
    "default": {
        "joint_friction": 0.0,
        "static_effort": 2.0,
        "dynamic_effort": 2.0,
        "viscous_nm_s_per_rad": 3.0,
        "armature": 0.5,
    },
}

# Map joint base name → motor profile key. Names not present here use "default".
JOINT_MODEL_TO_MOTOR_MODEL_PROFILE: Final[dict[str, str]] = {
    "arm_0": "arm_shoulder",
    "arm_1": "arm_shoulder",
    "arm_2": "arm_shoulder",
    "arm_3": "arm_elbow",
    "arm_4": "arm_wrist",
    "arm_5": "arm_wrist",
    "arm_6": "arm_wrist2",
    "torso_0": "lower_torso",
    "torso_1": "lower_torso",
    "torso_2": "lower_torso",
    "torso_3": "upper_torso",
    "torso_4": "upper_torso",
    "torso_5": "upper_torso",
}


def strip_side_prefix(joint_name: str) -> str:
    """Remove ``left_``/``right_`` prefix so symmetric joints share a profile."""
    for prefix in ("left_", "right_"):
        if joint_name.startswith(prefix):
            return joint_name.removeprefix(prefix)
    return joint_name


def get_profile_for_joint(joint_name: str) -> dict[str, float]:
    """Look up the motor profile for a fully qualified joint name."""
    base = strip_side_prefix(joint_name)
    profile_key = JOINT_MODEL_TO_MOTOR_MODEL_PROFILE.get(base, "default")
    return JOINT_MOTOR_MODEL_PROFILES[profile_key]
