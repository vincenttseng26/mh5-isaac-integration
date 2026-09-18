#!/usr/bin/env python3
"""Fail-closed motion command gateway core.

This module is deliberately transport-free. It imports no socket, no ROS, no
subprocess and no FS100 client, and it holds no address, port or controller
identity. Importing or instantiating it cannot create a path from Isaac Sim to
the physical robot. Its only output is whatever sink the caller injects; with no
sink, an accepted intent goes nowhere.

The gateway starts disarmed, refuses to arm while any declared precondition is
false, and disarms itself on E-stop, robot disable, disconnect or watchdog
expiry. Arming never survives a process restart.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import yaml

UNIT_RADIAN = "radian"


class GatewayRejection(Exception):
    """Raised for every rejected intent. Carries a stable machine-readable code."""

    def __init__(self, code: str, detail: str):
        super().__init__(f"{code}: {detail}")
        self.code = code
        self.detail = detail


@dataclass(frozen=True)
class MotionIntent:
    """A proposed joint-space target. Positions are radians, in config order."""

    joint_names: Tuple[str, ...]
    positions_rad: Tuple[float, ...]
    unit: str
    stamp_s: float
    sequence: int


@dataclass
class GatewayConfig:
    canonical_joint_order: Tuple[str, ...]
    unit: str
    position_limits_rad: Dict[str, Tuple[float, float]]
    max_step_rad: float
    max_velocity_rad_s: float
    max_acceleration_rad_s2: float
    min_command_interval_s: float
    max_command_rate_hz: float
    freshness_timeout_s: float
    watchdog_timeout_s: float
    dead_man_timeout_s: float
    require_explicit_arm: bool
    arm_persists_across_restart: bool
    preconditions: Dict[str, bool] = field(default_factory=dict)

    @staticmethod
    def load(path: str) -> "GatewayConfig":
        with open(path, "r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
        if raw.get("schema") != 1:
            raise ValueError(f"unsupported gateway config schema: {raw.get('schema')}")
        if raw.get("unit") != UNIT_RADIAN:
            raise ValueError(f"gateway config unit must be {UNIT_RADIAN}")
        order = tuple(raw["canonical_joint_order"])
        if len(set(order)) != len(order):
            raise ValueError("canonical_joint_order contains duplicates")
        limits = {}
        for name in order:
            if name not in raw["position_limits_rad"]:
                raise ValueError(f"missing position limits for {name}")
            lower, upper = raw["position_limits_rad"][name]
            if not (math.isfinite(lower) and math.isfinite(upper)) or lower >= upper:
                raise ValueError(f"invalid position limits for {name}")
            limits[name] = (float(lower), float(upper))
        if raw.get("arm_persists_across_restart", False):
            raise ValueError("arm_persists_across_restart must be false")
        for key in ("max_step_rad", "max_velocity_rad_s", "max_acceleration_rad_s2",
                    "min_command_interval_s", "freshness_timeout_s",
                    "watchdog_timeout_s", "dead_man_timeout_s"):
            value = float(raw[key])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be finite and positive")
        return GatewayConfig(
            canonical_joint_order=order,
            unit=raw["unit"],
            position_limits_rad=limits,
            max_step_rad=float(raw["max_step_rad"]),
            max_velocity_rad_s=float(raw["max_velocity_rad_s"]),
            max_acceleration_rad_s2=float(raw["max_acceleration_rad_s2"]),
            min_command_interval_s=float(raw["min_command_interval_s"]),
            max_command_rate_hz=float(raw["max_command_rate_hz"]),
            freshness_timeout_s=float(raw["freshness_timeout_s"]),
            watchdog_timeout_s=float(raw["watchdog_timeout_s"]),
            dead_man_timeout_s=float(raw["dead_man_timeout_s"]),
            require_explicit_arm=bool(raw["require_explicit_arm"]),
            arm_persists_across_restart=False,
            preconditions=dict(raw.get("preconditions", {})),
        )


class SafetyGatewayCore:
    """Fail-closed joint-space command gateway.

    Every rejection path leaves the previously accepted target untouched. The
    gateway never replays a stale command: after a watchdog expiry it disarms,
    and a disarmed gateway rejects everything until a human arms it again.
    """

    def __init__(
        self,
        config: GatewayConfig,
        sink: Optional[Callable[[Sequence[float]], None]] = None,
        workspace_check: Optional[Callable[[Sequence[float]], bool]] = None,
        collision_check: Optional[Callable[[Sequence[float]], bool]] = None,
    ):
        self.config = config
        self._sink = sink
        self._workspace_check = workspace_check
        self._collision_check = collision_check
        self._armed = False
        self._last_positions: Optional[Tuple[float, ...]] = None
        self._last_velocity: Optional[Tuple[float, ...]] = None
        self._last_accept_time: Optional[float] = None
        self._last_dead_man: Optional[float] = None
        self._last_sequence: Optional[int] = None
        self.rejections: List[str] = []

    # -- state ---------------------------------------------------------------

    @property
    def armed(self) -> bool:
        return self._armed

    def unmet_preconditions(self) -> List[str]:
        return sorted(k for k, v in self.config.preconditions.items() if not v)

    def arm(self, now_s: float, operator_ack: bool = False) -> None:
        """Arm the gateway. Refuses while any precondition is false."""
        if not operator_ack:
            raise GatewayRejection("ARM_REQUIRES_OPERATOR_ACK",
                                   "arming requires an explicit operator acknowledgement")
        unmet = self.unmet_preconditions()
        if unmet:
            raise GatewayRejection("ARM_PRECONDITION_UNMET",
                                   "unmet preconditions: " + ", ".join(unmet))
        if self._workspace_check is None:
            raise GatewayRejection("ARM_NO_WORKSPACE_CHECK",
                                   "a workspace check must be attached before arming")
        if self._collision_check is None:
            raise GatewayRejection("ARM_NO_COLLISION_CHECK",
                                   "a collision check must be attached before arming")
        self._armed = True
        self._last_dead_man = now_s
        self._last_accept_time = None
        self._last_velocity = None

    def disarm(self, reason: str = "explicit") -> None:
        self._armed = False
        self._last_velocity = None
        self.rejections.append(f"DISARMED:{reason}")

    def heartbeat(self, now_s: float) -> None:
        """Dead-man refresh. Without this the gateway disarms on the next submit."""
        self._last_dead_man = now_s

    def notify_estop(self) -> None:
        self.disarm("e_stop")

    def notify_robot_disabled(self) -> None:
        self.disarm("robot_disabled")

    def notify_disconnect(self) -> None:
        self.disarm("disconnect")

    # -- command path --------------------------------------------------------

    def submit(self, intent: MotionIntent, now_s: float) -> Tuple[float, ...]:
        try:
            return self._submit(intent, now_s)
        except GatewayRejection as exc:
            self.rejections.append(exc.code)
            raise

    def _submit(self, intent: MotionIntent, now_s: float) -> Tuple[float, ...]:
        cfg = self.config

        if not self._armed:
            raise GatewayRejection("DISARMED", "gateway is disarmed")

        # Dead-man and watchdog are checked before anything is trusted, and
        # both disarm rather than merely rejecting, so nothing can resume
        # silently after a stall.
        if self._last_dead_man is None or now_s - self._last_dead_man > cfg.dead_man_timeout_s:
            self.disarm("dead_man_timeout")
            raise GatewayRejection("DEAD_MAN_TIMEOUT", "dead-man heartbeat expired")
        if (self._last_accept_time is not None
                and now_s - self._last_accept_time > cfg.watchdog_timeout_s):
            self.disarm("watchdog_timeout")
            raise GatewayRejection("WATCHDOG_TIMEOUT", "no accepted command within watchdog window")

        if intent.unit != cfg.unit:
            raise GatewayRejection("WRONG_UNIT", f"expected {cfg.unit}, got {intent.unit}")
        if tuple(intent.joint_names) != cfg.canonical_joint_order:
            raise GatewayRejection(
                "JOINT_NAME_OR_ORDER",
                f"expected {cfg.canonical_joint_order}, got {tuple(intent.joint_names)}")
        if len(intent.positions_rad) != len(cfg.canonical_joint_order):
            raise GatewayRejection("WRONG_LENGTH", "position count does not match joint count")
        for name, value in zip(cfg.canonical_joint_order, intent.positions_rad):
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise GatewayRejection("NON_FINITE", f"{name} is not finite")

        if self._last_sequence is not None and intent.sequence <= self._last_sequence:
            raise GatewayRejection("STALE_SEQUENCE",
                                   f"sequence {intent.sequence} not newer than {self._last_sequence}")

        age = now_s - intent.stamp_s
        if age > cfg.freshness_timeout_s:
            raise GatewayRejection("STALE_TIMESTAMP", f"intent age {age:.4f}s exceeds limit")
        if age < -1e-6:
            raise GatewayRejection("FUTURE_TIMESTAMP", f"intent stamp is {-age:.4f}s in the future")

        if self._last_accept_time is not None:
            interval = now_s - self._last_accept_time
            if interval < cfg.min_command_interval_s:
                raise GatewayRejection("RATE_LIMIT",
                                       f"interval {interval:.4f}s below minimum")

        positions = tuple(float(v) for v in intent.positions_rad)

        for name, value in zip(cfg.canonical_joint_order, positions):
            lower, upper = cfg.position_limits_rad[name]
            if value < lower or value > upper:
                raise GatewayRejection("POSITION_LIMIT",
                                       f"{name}={value:.6f} outside [{lower:.6f}, {upper:.6f}]")

        velocity = None
        if self._last_positions is not None and self._last_accept_time is not None:
            dt = now_s - self._last_accept_time
            if dt <= 0:
                raise GatewayRejection("NON_POSITIVE_DT", "non-positive time step")
            deltas = [p - q for p, q in zip(positions, self._last_positions)]
            for name, delta in zip(cfg.canonical_joint_order, deltas):
                if abs(delta) > cfg.max_step_rad:
                    raise GatewayRejection("STEP_LIMIT",
                                           f"{name} step {delta:+.6f} rad exceeds "
                                           f"{cfg.max_step_rad} rad")
            velocity = tuple(d / dt for d in deltas)
            for name, v in zip(cfg.canonical_joint_order, velocity):
                if abs(v) > cfg.max_velocity_rad_s:
                    raise GatewayRejection("VELOCITY_LIMIT",
                                           f"{name} velocity {v:+.6f} rad/s exceeds "
                                           f"{cfg.max_velocity_rad_s}")
            if self._last_velocity is not None:
                for name, v, pv in zip(cfg.canonical_joint_order, velocity, self._last_velocity):
                    accel = (v - pv) / dt
                    if abs(accel) > cfg.max_acceleration_rad_s2:
                        raise GatewayRejection("ACCELERATION_LIMIT",
                                               f"{name} acceleration {accel:+.6f} rad/s^2 "
                                               f"exceeds {cfg.max_acceleration_rad_s2}")

        # Both external checks are fail-closed: absent or falsy means reject.
        if self._workspace_check is None:
            raise GatewayRejection("NO_WORKSPACE_CHECK", "workspace check not attached")
        if not self._workspace_check(positions):
            raise GatewayRejection("WORKSPACE_REJECTED", "workspace check rejected the target")
        if self._collision_check is None:
            raise GatewayRejection("NO_COLLISION_CHECK", "collision check not attached")
        if not self._collision_check(positions):
            raise GatewayRejection("COLLISION_REJECTED", "collision check rejected the target")

        self._last_positions = positions
        self._last_velocity = velocity
        self._last_accept_time = now_s
        self._last_sequence = intent.sequence

        if self._sink is not None:
            self._sink(positions)
        return positions
