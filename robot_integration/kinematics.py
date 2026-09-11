#!/usr/bin/env python3
"""URDF forward kinematics for the MH5 arm.

Pure geometry. Imports no transport and talks to no controller. Used to turn a
joint-space command into a Cartesian point so the gateway's workspace check can
reject targets that leave the permitted volume.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

ARM_JOINTS = ("joint_1_s", "joint_2_l", "joint_3_u",
              "joint_4_r", "joint_5_b", "joint_6_t")


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw, applied as Rz @ Ry @ Rx."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation about a unit axis."""
    norm = float(np.linalg.norm(axis))
    if norm == 0.0:
        return np.eye(3)
    k = axis / norm
    kx, ky, kz = k
    skew = np.array([[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]], dtype=float)
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = translation
    return out


@dataclass
class ChainJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray          # 4x4 fixed transform from parent to joint frame
    axis: np.ndarray            # rotation axis in the joint frame
    actuated_index: Optional[int]  # index into the arm joint vector, or None


class Chain:
    """A resolved kinematic chain between two links of a URDF."""

    def __init__(self, joints: List[ChainJoint], base_link: str, tip_link: str):
        self.joints = joints
        self.base_link = base_link
        self.tip_link = tip_link

    @property
    def actuated_names(self) -> Tuple[str, ...]:
        return tuple(j.name for j in self.joints if j.actuated_index is not None)

    def fk(self, q: Sequence[float]) -> np.ndarray:
        """Return the 4x4 transform from base_link to tip_link."""
        if len(q) != len(ARM_JOINTS):
            raise ValueError(f"expected {len(ARM_JOINTS)} joint values, got {len(q)}")
        transform = np.eye(4)
        for joint in self.joints:
            transform = transform @ joint.origin
            if joint.actuated_index is not None:
                angle = float(q[joint.actuated_index])
                if not math.isfinite(angle):
                    raise ValueError(f"non-finite joint value for {joint.name}")
                transform = transform @ homogeneous(
                    axis_angle_to_matrix(joint.axis, angle), np.zeros(3))
        return transform

    def fk_position(self, q: Sequence[float]) -> np.ndarray:
        return self.fk(q)[:3, 3]


def _parse_vector(text: Optional[str], default: Tuple[float, float, float]) -> np.ndarray:
    if not text:
        return np.array(default, dtype=float)
    parts = [float(v) for v in text.replace(",", " ").split()]
    if len(parts) != 3:
        raise ValueError(f"expected three numbers, got {text!r}")
    return np.array(parts, dtype=float)


def load_chain(urdf_path: str, base_link: str = "base_link",
               tip_link: str = "grasp_link") -> Chain:
    """Resolve the joint chain from base_link to tip_link in a URDF."""
    root = ET.parse(urdf_path).getroot()

    by_child: Dict[str, ET.Element] = {}
    for joint in root.iter("joint"):
        child = joint.find("child")
        if child is not None and child.get("link"):
            by_child[child.get("link")] = joint

    # Walk upward from the tip to the base, then reverse.
    reversed_chain: List[ChainJoint] = []
    link = tip_link
    seen = set()
    while link != base_link:
        if link in seen:
            raise ValueError(f"cycle in URDF while walking up from {tip_link}")
        seen.add(link)
        joint = by_child.get(link)
        if joint is None:
            raise ValueError(f"no joint produces link {link!r}; "
                             f"cannot reach {base_link!r} from {tip_link!r}")
        origin_el = joint.find("origin")
        xyz = _parse_vector(origin_el.get("xyz") if origin_el is not None else None,
                            (0.0, 0.0, 0.0))
        rpy = _parse_vector(origin_el.get("rpy") if origin_el is not None else None,
                            (0.0, 0.0, 0.0))
        axis_el = joint.find("axis")
        axis = _parse_vector(axis_el.get("xyz") if axis_el is not None else None,
                             (1.0, 0.0, 0.0))
        joint_type = joint.get("type", "fixed")
        name = joint.get("name", "")
        actuated = ARM_JOINTS.index(name) if name in ARM_JOINTS else None
        if joint_type in ("revolute", "continuous") and actuated is None:
            raise ValueError(f"chain contains unexpected actuated joint {name!r}")
        reversed_chain.append(ChainJoint(
            name=name, joint_type=joint_type,
            parent=joint.find("parent").get("link"), child=link,
            origin=homogeneous(rpy_to_matrix(*rpy), xyz),
            axis=axis, actuated_index=actuated))
        link = joint.find("parent").get("link")

    return Chain(list(reversed(reversed_chain)), base_link, tip_link)
