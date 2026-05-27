#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Iterable


JOINT_ORDER = [
    "base_yaw_joint",
    "shoulder_pitch_joint",
    "elbow_pitch_joint",
    "wrist_pitch_joint",
    "gripper_joint",
]


@dataclass(frozen=True)
class JointLimit:
    lower: float
    upper: float

    def contains(self, value: float) -> bool:
        return self.lower <= value <= self.upper

    def clamp(self, value: float) -> float:
        return min(max(value, self.lower), self.upper)


@dataclass(frozen=True)
class ArmModel:
    base_height_m: float = 0.09
    shoulder_offset_m: float = 0.18
    upper_arm_m: float = 0.24
    forearm_m: float = 0.20
    wrist_m: float = 0.11
    tool_m: float = 0.09


@dataclass(frozen=True)
class JointAngles:
    base_yaw: float = 0.0
    shoulder_pitch: float = 0.0
    elbow_pitch: float = 0.0
    wrist_pitch: float = 0.0
    gripper: float = 0.0

    @classmethod
    def from_positions(cls, positions: dict[str, float]) -> "JointAngles":
        return cls(
            base_yaw=positions.get("base_yaw_joint", 0.0),
            shoulder_pitch=positions.get("shoulder_pitch_joint", 0.0),
            elbow_pitch=positions.get("elbow_pitch_joint", 0.0),
            wrist_pitch=positions.get("wrist_pitch_joint", 0.0),
            gripper=positions.get("gripper_joint", 0.0),
        )

    def as_list(self) -> list[float]:
        return [
            self.base_yaw,
            self.shoulder_pitch,
            self.elbow_pitch,
            self.wrist_pitch,
            self.gripper,
        ]


@dataclass(frozen=True)
class EndEffectorPose:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    pitch_rad: float
    radial_reach_m: float
    within_joint_limits: bool
    joint_limit_violations: tuple[str, ...]


@dataclass(frozen=True)
class IkSolution:
    reachable: bool
    joint_angles: JointAngles
    target_x_m: float
    target_y_m: float
    target_z_m: float
    radial_reach_m: float
    reason: str
    within_joint_limits: bool
    joint_limit_violations: tuple[str, ...]


JOINT_LIMITS = {
    "base_yaw_joint": JointLimit(-math.pi, math.pi),
    "shoulder_pitch_joint": JointLimit(-math.pi / 2.0, math.pi / 2.0),
    "elbow_pitch_joint": JointLimit(-math.pi / 2.0, math.pi / 2.0),
    "wrist_pitch_joint": JointLimit(-math.pi / 2.0, math.pi / 2.0),
    "gripper_joint": JointLimit(-math.pi / 4.0, math.pi / 4.0),
}


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def joint_positions_from_message(names: Iterable[str], positions: Iterable[float]) -> dict[str, float]:
    return {name: float(position) for name, position in zip(names, positions)}


def joint_limit_violations(angles: JointAngles) -> tuple[str, ...]:
    violations: list[str] = []
    for name, value in zip(JOINT_ORDER, angles.as_list()):
        if not JOINT_LIMITS[name].contains(value):
            violations.append(name)
    return tuple(violations)


def clamp_to_joint_limits(angles: JointAngles) -> JointAngles:
    return JointAngles(
        base_yaw=JOINT_LIMITS["base_yaw_joint"].clamp(normalize_angle(angles.base_yaw)),
        shoulder_pitch=JOINT_LIMITS["shoulder_pitch_joint"].clamp(angles.shoulder_pitch),
        elbow_pitch=JOINT_LIMITS["elbow_pitch_joint"].clamp(angles.elbow_pitch),
        wrist_pitch=JOINT_LIMITS["wrist_pitch_joint"].clamp(angles.wrist_pitch),
        gripper=JOINT_LIMITS["gripper_joint"].clamp(angles.gripper),
    )


def forward_kinematics(angles: JointAngles, model: ArmModel = ArmModel()) -> EndEffectorPose:
    shoulder = angles.shoulder_pitch
    elbow = shoulder + angles.elbow_pitch
    wrist = elbow + angles.wrist_pitch
    tool_chain_m = model.wrist_m + model.tool_m

    radius = (
        model.shoulder_offset_m
        + model.upper_arm_m * math.cos(shoulder)
        + model.forearm_m * math.cos(elbow)
        + tool_chain_m * math.cos(wrist)
    )
    z_m = (
        model.base_height_m
        + model.upper_arm_m * math.sin(shoulder)
        + model.forearm_m * math.sin(elbow)
        + tool_chain_m * math.sin(wrist)
    )
    yaw = normalize_angle(angles.base_yaw)
    x_m = radius * math.cos(yaw)
    y_m = radius * math.sin(yaw)
    violations = joint_limit_violations(angles)

    return EndEffectorPose(
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        yaw_rad=yaw,
        pitch_rad=normalize_angle(wrist),
        radial_reach_m=radius,
        within_joint_limits=not violations,
        joint_limit_violations=violations,
    )


def inverse_kinematics(
    target_x_m: float,
    target_y_m: float,
    target_z_m: float,
    model: ArmModel = ArmModel(),
    target_tool_pitch_rad: float = 0.0,
) -> IkSolution:
    yaw = normalize_angle(math.atan2(target_y_m, target_x_m))
    radius = math.hypot(target_x_m, target_y_m)
    tool_chain_m = model.wrist_m + model.tool_m
    wrist_radius = radius - model.shoulder_offset_m - tool_chain_m * math.cos(target_tool_pitch_rad)
    wrist_z = target_z_m - model.base_height_m - tool_chain_m * math.sin(target_tool_pitch_rad)

    l1 = model.upper_arm_m
    l2 = model.forearm_m
    distance_sq = wrist_radius * wrist_radius + wrist_z * wrist_z
    distance = math.sqrt(distance_sq)
    min_reach = abs(l1 - l2)
    max_reach = l1 + l2
    reachable = min_reach <= distance <= max_reach

    cos_elbow_raw = (distance_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    cos_elbow = min(max(cos_elbow_raw, -1.0), 1.0)
    elbow = math.atan2(math.sqrt(max(0.0, 1.0 - cos_elbow * cos_elbow)), cos_elbow)
    shoulder = math.atan2(wrist_z, wrist_radius) - math.atan2(
        l2 * math.sin(elbow),
        l1 + l2 * math.cos(elbow),
    )
    wrist = target_tool_pitch_rad - shoulder - elbow

    angles = JointAngles(
        base_yaw=yaw,
        shoulder_pitch=shoulder,
        elbow_pitch=elbow,
        wrist_pitch=wrist,
        gripper=0.0,
    )
    violations = joint_limit_violations(angles)
    reason = "ok"
    if not reachable:
        reason = "outside_workspace"
    elif violations:
        reason = "joint_limit_violation"

    return IkSolution(
        reachable=reachable and not violations,
        joint_angles=angles,
        target_x_m=target_x_m,
        target_y_m=target_y_m,
        target_z_m=target_z_m,
        radial_reach_m=radius,
        reason=reason,
        within_joint_limits=not violations,
        joint_limit_violations=violations,
    )
