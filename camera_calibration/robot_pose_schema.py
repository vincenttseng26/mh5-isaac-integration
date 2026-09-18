"""Transport-agnostic schema + validator for robot pose samples used in
hand-eye data collection.

This module intentionally does NOT talk to any robot controller or network
transport. It defines the read-only sample format that a future FS100
adapter (or any other controller adapter) must emit, and a strict validator
that fails closed on anything not already verified against Yaskawa
documentation for this cell.

Why "fail closed" instead of "best guess":
Yaskawa FS100 controllers expose robot pose in more than one representation
(joint values as pulses vs. degrees; TCP pose as a coordinate-frame-relative
Cartesian position plus an Euler-angle orientation whose axis order and sign
convention are controller/parameter-file specific). Silently assuming a
pulse-to-degree scale factor or a rotation order would produce hand-eye
samples that *look* valid but encode a wrong transform with no error at
solve time — the AX=XB solver has no way to detect a systematic convention
error, it just returns a wrong-but-plausible extrinsic. So every field whose
correctness depends on an FS100-specific convention is either:
  1. restricted to an unambiguous representation (quaternion, rotation
     matrix, degrees, radians), or
  2. required to carry explicit confirmed conversion metadata, or
  3. rejected outright until confirmed (see the ALLOWED_* allowlists below).

See README.md "MH5 + FS100 資料收集" for the human-facing checklist and the
list of fields still pending Agy's FS100 transport/parameter confirmation.
"""
from __future__ import annotations

import dataclasses
import datetime as _dt
from typing import Any, Optional

# ---------------------------------------------------------------------------
# Allowlists. These are the ONLY representations this module will accept.
# Extending them requires a verified citation (manual section, parameter file
# dump, or vendor confirmation), not an assumption. See README for how to
# extend them once FS100 details are confirmed.
# ---------------------------------------------------------------------------

# Joint angle units we can interpret without ambiguity. "pulse" is
# deliberately NOT in this set: the pulse-per-degree scale for an FS100 is
# per-axis, stored in the controller's PULSE.PRM (or equivalent absolute
# encoder parameter) file, and differs by robot model/gear ratio. We do not
# hard-code or guess it.
ALLOWED_JOINT_UNITS = frozenset({"degree", "radian"})

# TCP orientation representations we can interpret without ambiguity.
# Euler-angle forms (e.g. FS100's Rx/Ry/Rz job-variable convention) are
# deliberately excluded: the axis order (extrinsic/intrinsic, XYZ vs ZYX...)
# and sign convention are not asserted here without a verified citation.
# If/when that convention is confirmed for this FS100 unit, add a new
# explicit literal (e.g. "euler_fs100_rz_ry_rx_confirmed_2026xxxx") rather
# than a generic "euler" bucket, so the confirmation is traceable.
ALLOWED_ORIENTATION_REPRESENTATIONS = frozenset({
    "quaternion_wxyz",
    "rotation_matrix_3x3",
})

ALLOWED_POSITION_UNITS = frozenset({"m", "mm"})

# Transport identifiers this collection layer has verified end-to-end.
# FS100 transport (e.g. the High-Speed Ethernet Server / HSES function used
# by the "Ethernet Function" option, ports 10040/10041) has NOT been
# confirmed for this cell yet, so the only value accepted right now is the
# explicit placeholder "unconfirmed", and samples carrying it are marked
# not usable for a real solve (see RobotPoseSample.is_production_ready).
ALLOWED_TRANSPORTS = frozenset({"unconfirmed"})

# A 6-axis Motoman (MH5 included) has exactly 6 joint values.
EXPECTED_JOINT_COUNT = 6

SCHEMA_VERSION = "1.0"


class PoseSampleError(ValueError):
    """Base class for all robot pose sample validation failures."""


class UnverifiedRepresentationError(PoseSampleError):
    """Raised when a sample uses a representation/unit/transport that has
    not been verified for this robot/controller — e.g. pulse joint values
    without a confirmed conversion, or an Euler-angle orientation. This is
    a hard rejection, not a fallback guess."""


class TimeSyncError(PoseSampleError):
    """Raised when a sample's own timestamp fields are inconsistent, or
    when cross-sample / cross-frame timing checks fail."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise PoseSampleError(message)


@dataclasses.dataclass
class Timestamp:
    """Dual timestamp: a monotonic clock reading for ordering/skew
    checks within one collection session, and a UTC wall-clock reading for
    correlating against external logs (e.g. the FS100 controller's own
    logging, or the camera capture pipeline's timestamps).

    Both are required — a UTC-only timestamp cannot detect clock steps
    (NTP jumps, DST, manual clock changes) between the frame and the pose;
    a monotonic-only timestamp cannot be correlated across processes/machines.
    """

    monotonic_ns: int
    utc_iso8601: str

    def utc_datetime(self) -> _dt.datetime:
        text = self.utc_iso8601
        if text.endswith("Z"):
            text = text[:-1] + "+00:00"
        dt = _dt.datetime.fromisoformat(text)
        if dt.tzinfo is None:
            raise TimeSyncError(
                f"utc_iso8601 {self.utc_iso8601!r} has no timezone offset; "
                "must be explicit UTC (e.g. suffix 'Z' or '+00:00')."
            )
        return dt.astimezone(_dt.timezone.utc)

    @classmethod
    def from_dict(cls, data: dict) -> "Timestamp":
        _require("monotonic_ns" in data, "timestamp.monotonic_ns is required")
        _require("utc_iso8601" in data, "timestamp.utc_iso8601 is required")
        monotonic_ns = data["monotonic_ns"]
        utc_iso8601 = data["utc_iso8601"]
        _require(isinstance(monotonic_ns, int), "timestamp.monotonic_ns must be an integer (ns)")
        _require(isinstance(utc_iso8601, str) and len(utc_iso8601) > 0,
                  "timestamp.utc_iso8601 must be a non-empty ISO-8601 string")
        ts = cls(monotonic_ns=monotonic_ns, utc_iso8601=utc_iso8601)
        ts.utc_datetime()  # raises TimeSyncError if unparseable/naive
        return ts


@dataclasses.dataclass
class SourceInfo:
    """Provenance of the sample: which controller/robot/adapter produced it,
    and over what transport. `transport` is checked against
    ALLOWED_TRANSPORTS — see module docstring."""

    controller: str
    robot_model: str
    adapter: str
    adapter_version: str
    transport: str

    @classmethod
    def from_dict(cls, data: dict) -> "SourceInfo":
        for field in ("controller", "robot_model", "adapter", "adapter_version", "transport"):
            _require(field in data, f"source.{field} is required")
            _require(isinstance(data[field], str) and data[field] != "",
                      f"source.{field} must be a non-empty string")
        if data["transport"] not in ALLOWED_TRANSPORTS:
            raise UnverifiedRepresentationError(
                f"source.transport={data['transport']!r} is not in the verified "
                f"allowlist {sorted(ALLOWED_TRANSPORTS)}. FS100 transport has not "
                "been confirmed yet for this cell — use 'unconfirmed' for dry-run "
                "data, or add the confirmed transport identifier to "
                "ALLOWED_TRANSPORTS in robot_pose_schema.py once Agy confirms it "
                "(see README 'MH5 + FS100 資料收集' pending-confirmation table)."
            )
        return cls(**{k: data[k] for k in
                       ("controller", "robot_model", "adapter", "adapter_version", "transport")})


@dataclasses.dataclass
class JointValues:
    unit: str
    values: list
    pulse_conversion: Optional[dict] = None

    @classmethod
    def from_dict(cls, data: dict) -> "JointValues":
        _require("unit" in data, "joints.unit is required")
        _require("values" in data, "joints.values is required")
        unit = data["unit"]
        values = data["values"]
        _require(isinstance(values, list) and len(values) == EXPECTED_JOINT_COUNT,
                  f"joints.values must be a list of exactly {EXPECTED_JOINT_COUNT} "
                  f"numbers (MH5 is a 6-axis robot), got {values!r}")
        for v in values:
            _require(isinstance(v, (int, float)), f"joints.values entries must be numeric, got {v!r}")

        if unit not in ALLOWED_JOINT_UNITS:
            raise UnverifiedRepresentationError(
                f"joints.unit={unit!r} is not verified. Only "
                f"{sorted(ALLOWED_JOINT_UNITS)} are accepted. FS100 raw joint "
                "feedback is commonly in encoder pulses; this layer refuses to "
                "guess the per-axis pulse-per-degree scale (it lives in the "
                "controller's absolute-encoder parameter file and is robot-"
                "unit-specific). Convert to degrees/radians in the FS100 "
                "adapter using a confirmed scale before emitting a sample, or "
                "attach a fully-populated+confirmed joints.pulse_conversion "
                "block if you have added explicit support for it."
            )
        return cls(unit=unit, values=list(values), pulse_conversion=data.get("pulse_conversion"))


@dataclasses.dataclass
class Position:
    unit: str
    x: float
    y: float
    z: float

    @classmethod
    def from_dict(cls, data: dict) -> "Position":
        for field in ("unit", "x", "y", "z"):
            _require(field in data, f"tcp_pose.position.{field} is required")
        unit = data["unit"]
        if unit not in ALLOWED_POSITION_UNITS:
            raise UnverifiedRepresentationError(
                f"tcp_pose.position.unit={unit!r} is not verified. Only "
                f"{sorted(ALLOWED_POSITION_UNITS)} are accepted — state the "
                "unit explicitly rather than relying on a default."
            )
        return cls(unit=unit, x=float(data["x"]), y=float(data["y"]), z=float(data["z"]))


@dataclasses.dataclass
class Orientation:
    representation: str
    values: list

    @classmethod
    def from_dict(cls, data: dict) -> "Orientation":
        _require("representation" in data, "tcp_pose.orientation.representation is required")
        _require("values" in data, "tcp_pose.orientation.values is required")
        rep = data["representation"]
        values = data["values"]
        if rep not in ALLOWED_ORIENTATION_REPRESENTATIONS:
            raise UnverifiedRepresentationError(
                f"tcp_pose.orientation.representation={rep!r} is not verified. "
                f"Only {sorted(ALLOWED_ORIENTATION_REPRESENTATIONS)} are accepted. "
                "FS100 typically reports TCP orientation as Rx/Ry/Rz Euler angles "
                "whose axis order and sign convention are not asserted by this "
                "module without a verified citation (see README pending-"
                "confirmation table). Convert to a quaternion or rotation matrix "
                "in the FS100 adapter using a confirmed convention before "
                "emitting a sample."
            )
        if rep == "quaternion_wxyz":
            _require(isinstance(values, list) and len(values) == 4,
                      "orientation.values must be [w, x, y, z] for quaternion_wxyz")
            norm = sum(v * v for v in values) ** 0.5
            _require(abs(norm - 1.0) < 1e-3,
                      f"orientation quaternion is not normalized (|q|={norm:.6f}); "
                      "check the adapter's conversion")
        elif rep == "rotation_matrix_3x3":
            _require(isinstance(values, list) and len(values) == 3
                      and all(isinstance(row, list) and len(row) == 3 for row in values),
                      "orientation.values must be a 3x3 nested list for rotation_matrix_3x3")
            flat = [v for row in values for v in row]
            _require(all(isinstance(v, (int, float)) for v in flat),
                      "rotation_matrix_3x3 entries must be numeric")
        return cls(representation=rep, values=values)


@dataclasses.dataclass
class TcpPose:
    position: Position
    orientation: Orientation

    @classmethod
    def from_dict(cls, data: dict) -> "TcpPose":
        _require("position" in data, "tcp_pose.position is required")
        _require("orientation" in data, "tcp_pose.orientation is required")
        return cls(position=Position.from_dict(data["position"]),
                    orientation=Orientation.from_dict(data["orientation"]))


@dataclasses.dataclass
class RobotPoseSample:
    """One read-only, transport-agnostic robot pose sample for hand-eye
    data collection. Construct via `RobotPoseSample.from_dict`, never by
    hand-filling fields that involve a representation choice — that keeps
    every unverified assumption funneled through the allowlists above."""

    schema_version: str
    sample_id: str
    timestamp: Timestamp
    source: SourceInfo
    frame_id: str
    tool_id: int
    user_frame_id: Optional[int]
    joints: JointValues
    tcp_pose: TcpPose
    raw: Optional[dict] = None

    @property
    def is_production_ready(self) -> bool:
        """False for any sample whose provenance transport is not yet a
        confirmed FS100 link — i.e. everything today, until Agy confirms
        the transport and it is added to ALLOWED_TRANSPORTS."""
        return self.source.transport != "unconfirmed"

    @classmethod
    def from_dict(cls, data: dict) -> "RobotPoseSample":
        _require(isinstance(data, dict), "sample must be a JSON object")
        _require("schema_version" in data, "schema_version is required")
        _require(data["schema_version"] == SCHEMA_VERSION,
                  f"schema_version {data['schema_version']!r} != supported {SCHEMA_VERSION!r}")
        _require("sample_id" in data and isinstance(data["sample_id"], str) and data["sample_id"],
                  "sample_id is required (non-empty string)")
        _require("timestamp" in data, "timestamp is required")
        _require("source" in data, "source is required")
        _require("frame_id" in data and isinstance(data["frame_id"], str) and data["frame_id"],
                  "frame_id is required (non-empty string) — the reference frame "
                  "joints/tcp_pose are expressed in")
        _require("tool_id" in data, "tool_id is required")
        _require(isinstance(data["tool_id"], int) and data["tool_id"] >= 0,
                  "tool_id must be a non-negative integer (FS100 tool number)")
        user_frame_id = data.get("user_frame_id")
        _require(user_frame_id is None or (isinstance(user_frame_id, int) and user_frame_id >= 0),
                  "user_frame_id must be null (robot/base coordinate) or a "
                  "non-negative integer (FS100 user coordinate number)")
        _require("joints" in data, "joints is required")
        _require("tcp_pose" in data, "tcp_pose is required")

        return cls(
            schema_version=data["schema_version"],
            sample_id=data["sample_id"],
            timestamp=Timestamp.from_dict(data["timestamp"]),
            source=SourceInfo.from_dict(data["source"]),
            frame_id=data["frame_id"],
            tool_id=data["tool_id"],
            user_frame_id=user_frame_id,
            joints=JointValues.from_dict(data["joints"]),
            tcp_pose=TcpPose.from_dict(data["tcp_pose"]),
            raw=data.get("raw"),
        )

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "sample_id": self.sample_id,
            "timestamp": dataclasses.asdict(self.timestamp),
            "source": dataclasses.asdict(self.source),
            "frame_id": self.frame_id,
            "tool_id": self.tool_id,
            "user_frame_id": self.user_frame_id,
            "joints": dataclasses.asdict(self.joints),
            "tcp_pose": {
                "position": dataclasses.asdict(self.tcp_pose.position),
                "orientation": dataclasses.asdict(self.tcp_pose.orientation),
            },
            "raw": self.raw,
        }


def check_time_sync(samples: list, max_skew_ms: float) -> list:
    """Cross-sample sanity checks that don't require any hardware:

    1. utc and monotonic clocks must agree on ordering and on elapsed time
       between consecutive samples (within max_skew_ms) — catches clock
       steps (NTP adjustment, manual clock change) that would otherwise
       silently mis-pair a pose with the wrong frame.
    2. sample_ids must be unique.

    Returns a list of human-readable warning/error strings (empty if all
    checks pass). Does not raise, so a caller can decide whether skew
    warnings are fatal for their use case.
    """
    problems: list = []
    seen_ids: dict = {}
    for s in samples:
        if s.sample_id in seen_ids:
            problems.append(f"duplicate sample_id: {s.sample_id!r}")
        seen_ids[s.sample_id] = s

    ordered = sorted(samples, key=lambda s: s.timestamp.monotonic_ns)
    for prev, cur in zip(ordered, ordered[1:]):
        mono_dt_ms = (cur.timestamp.monotonic_ns - prev.timestamp.monotonic_ns) / 1e6
        utc_dt_ms = (cur.timestamp.utc_datetime() - prev.timestamp.utc_datetime()).total_seconds() * 1000.0
        skew_ms = abs(mono_dt_ms - utc_dt_ms)
        if mono_dt_ms < 0:
            problems.append(
                f"non-monotonic ordering bug: {prev.sample_id} -> {cur.sample_id} "
                f"has negative monotonic delta ({mono_dt_ms:.3f} ms)"
            )
        if utc_dt_ms < 0:
            problems.append(
                f"UTC clock went backwards between {prev.sample_id} and {cur.sample_id} "
                f"({utc_dt_ms:.3f} ms) while monotonic clock advanced "
                f"({mono_dt_ms:.3f} ms) — likely a clock step during collection"
            )
        elif skew_ms > max_skew_ms:
            problems.append(
                f"clock skew between monotonic and UTC deltas exceeds "
                f"{max_skew_ms} ms between {prev.sample_id} and {cur.sample_id}: "
                f"mono_dt={mono_dt_ms:.3f}ms utc_dt={utc_dt_ms:.3f}ms skew={skew_ms:.3f}ms"
            )
    return problems
