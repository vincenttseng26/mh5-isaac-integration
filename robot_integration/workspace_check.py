#!/usr/bin/env python3
"""Forward-kinematic workspace check for the motion command gateway.

Turns a joint-space command into the Cartesian position of the grasp link and
tests it against a configured volume. Pure geometry: no transport, no
controller, no network.

Attach an instance as the gateway's workspace_check callback. It is fail-closed
by construction: any non-finite input, any kinematic failure and any point
outside the volume returns False, and the gateway treats False as a rejection.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np
import yaml

from .kinematics import ARM_JOINTS, Chain, load_chain


@dataclass(frozen=True)
class WorkspaceBounds:
    x_m: Tuple[float, float]
    y_m: Tuple[float, float]
    z_m: Tuple[float, float]
    max_radius_m: float
    min_radius_m: float

    def validate(self) -> None:
        for name, (lo, hi) in (("x_m", self.x_m), ("y_m", self.y_m), ("z_m", self.z_m)):
            if not (math.isfinite(lo) and math.isfinite(hi)) or lo >= hi:
                raise ValueError(f"invalid workspace bound {name}: {(lo, hi)}")
        if not 0.0 <= self.min_radius_m < self.max_radius_m:
            raise ValueError("invalid workspace radii")


class WorkspaceChecker:
    """Callable suitable for SafetyGatewayCore(workspace_check=...)."""

    def __init__(self, chain: Chain, bounds: WorkspaceBounds):
        bounds.validate()
        self.chain = chain
        self.bounds = bounds
        self.last_position: Optional[np.ndarray] = None
        self.last_reason: Optional[str] = None

    @staticmethod
    def from_config(config_path: str) -> "WorkspaceChecker":
        with open(config_path, "r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
        section = raw.get("workspace")
        if not section:
            raise ValueError("gateway config has no workspace section")
        chain = load_chain(section["urdf"], section["base_link"], section["tip_link"])
        if chain.actuated_names != ARM_JOINTS:
            raise ValueError(f"chain actuates {chain.actuated_names}, expected {ARM_JOINTS}")
        bounds = WorkspaceBounds(
            x_m=tuple(section["x_m"]),
            y_m=tuple(section["y_m"]),
            z_m=tuple(section["z_m"]),
            max_radius_m=float(section["max_radius_m"]),
            min_radius_m=float(section["min_radius_m"]),
        )
        return WorkspaceChecker(chain, bounds)

    def position_for(self, q: Sequence[float]) -> np.ndarray:
        return self.chain.fk_position(q)

    def __call__(self, q: Sequence[float]) -> bool:
        self.last_position = None
        self.last_reason = None
        try:
            if len(q) != len(ARM_JOINTS):
                self.last_reason = "wrong joint count"
                return False
            if not all(math.isfinite(float(v)) for v in q):
                self.last_reason = "non-finite joint value"
                return False
            position = self.chain.fk_position(q)
        except (ValueError, TypeError) as exc:
            self.last_reason = f"kinematics failed: {exc}"
            return False

        self.last_position = position
        x, y, z = (float(v) for v in position)
        if not all(math.isfinite(v) for v in (x, y, z)):
            self.last_reason = "non-finite Cartesian result"
            return False

        bounds = self.bounds
        for name, value, (lo, hi) in (("x", x, bounds.x_m),
                                      ("y", y, bounds.y_m),
                                      ("z", z, bounds.z_m)):
            if value < lo or value > hi:
                self.last_reason = f"{name}={value:.4f} m outside [{lo:.4f}, {hi:.4f}]"
                return False

        radius = math.sqrt(x * x + y * y + z * z)
        if radius > bounds.max_radius_m:
            self.last_reason = f"radius {radius:.4f} m exceeds {bounds.max_radius_m:.4f}"
            return False
        if radius < bounds.min_radius_m:
            self.last_reason = f"radius {radius:.4f} m below {bounds.min_radius_m:.4f}"
            return False
        return True
