#!/usr/bin/env python3
"""Red-object detection and top-down grasp staging for the MH5 workcell.

Pure geometry. Imports no transport, opens no socket and commands no motion.
Consumes a point cloud that is ALREADY expressed in base_link (the panel's
LiveZedOverlay applies the accepted optical-to-base transform), so detection can
share the single-client ZED stream instead of competing for it.

The grasp convention follows the URDF, verified numerically:
  * grasp_link +X is the approach direction (grasp_joint applies Ry(-90 deg)),
    so a top-down grasp needs grasp_link +X aligned with base_link -Z.
  * The RG2-FT fingers separate along grasp_link +Y (dot product 1.000000).
In top_down_matrix(yaw) the +Y axis is (-sin yaw, cos yaw, 0). Aligning the
fingers across an object's narrow side therefore means setting yaw to the angle
of the object's LONG axis in the XY plane.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml


CHANNELS = {"red": 0, "green": 1, "blue": 2}


@dataclass(frozen=True)
class GraspProfile:
    """Reviewed, per-object detection and grasp geometry."""
    name: str
    # Colour rule on the 0..255 scale: the dominant channel must clear
    # min_value, exceed ratios[other] * other for each named channel, and every
    # channel named in caps must stay below its cap.
    channel: str = "red"
    min_value: float = 130.0
    ratios: Dict[str, float] = field(default_factory=lambda: {"green": 1.45,
                                                              "blue": 1.45})
    caps: Dict[str, float] = field(default_factory=lambda: {"green": 150.0})
    # base_link crop. Defaults match config/safety/mh5_gateway_core_v0.yaml.
    x_m: Tuple[float, float] = (0.30, 0.80)
    y_m: Tuple[float, float] = (-0.35, 0.35)
    z_m: Tuple[float, float] = (0.010, 0.60)
    # Clustering.
    voxel_m: float = 0.010
    min_points: int = 40
    # Grasp staging.
    grasp_depth_below_top_m: float = 0.020
    approach_height_m: float = 0.100
    lift_height_m: float = 0.120
    min_grasp_z_m: float = 0.015
    # RG2-FT stroke. The gateway maps width register 0..1000 to 0..100 mm.
    gripper_max_width_m: float = 0.100
    gripper_clearance_m: float = 0.012

    def validate(self) -> None:
        if self.channel not in CHANNELS:
            raise ValueError(f"unknown colour channel {self.channel!r}")
        for name in list(self.ratios) + list(self.caps):
            if name not in CHANNELS:
                raise ValueError(f"unknown colour channel {name!r}")
        if self.channel in self.ratios:
            raise ValueError("a channel cannot be compared against itself")
        if not all(math.isfinite(float(v)) and v > 0.0 for v in self.ratios.values()):
            raise ValueError("colour ratios must be positive and finite")
        for label, (lo, hi) in (("x_m", self.x_m), ("y_m", self.y_m), ("z_m", self.z_m)):
            if not (math.isfinite(lo) and math.isfinite(hi)) or lo >= hi:
                raise ValueError(f"invalid crop bound {label}: {(lo, hi)}")
        if self.voxel_m <= 0.0 or self.min_points < 1:
            raise ValueError("invalid clustering parameters")
        if self.gripper_max_width_m <= 0.0 or self.gripper_clearance_m < 0.0:
            raise ValueError("invalid gripper geometry")


@dataclass
class Detection:
    """One detected object, entirely in base_link metres."""
    valid: bool
    reason: Optional[str] = None
    centroid_m: Optional[np.ndarray] = None
    extent_m: Optional[np.ndarray] = None      # long, short, height
    top_z_m: Optional[float] = None
    yaw_rad: Optional[float] = None            # angle of the long axis in XY
    width_across_fingers_m: Optional[float] = None
    point_count: int = 0
    candidate_clusters: int = 0

    def as_dict(self) -> Dict:
        def out(v):
            if v is None:
                return None
            return [float(x) for x in np.asarray(v).ravel()]
        return {"valid": bool(self.valid), "reason": self.reason,
                "centroid_base_link_m": out(self.centroid_m),
                "extent_m": out(self.extent_m),
                "top_z_m": None if self.top_z_m is None else float(self.top_z_m),
                "yaw_rad": None if self.yaw_rad is None else float(self.yaw_rad),
                "width_across_fingers_m": (None if self.width_across_fingers_m is None
                                           else float(self.width_across_fingers_m)),
                "point_count": int(self.point_count),
                "candidate_clusters": int(self.candidate_clusters)}


def load_profile(path: str) -> Tuple[GraspProfile, bool]:
    """Load an object profile. Returns the profile and its motion permission.

    The permission flag is reported, never consumed: this module emits no
    motion, and a caller that plans physical movement must check it itself.
    """
    with open(path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, dict) or not raw.get("name"):
        raise ValueError(f"{path} is not a named object profile")
    fields = {"name": str(raw["name"])}
    known = {f for f in GraspProfile.__dataclass_fields__ if f != "name"}
    for section in ("detection", "grasp"):
        for key, value in (raw.get(section) or {}).items():
            if key not in known:
                raise ValueError(f"{path}: unknown {section} key {key!r}")
            if isinstance(value, dict):
                fields[key] = {str(k): float(v) for k, v in value.items()}
            elif isinstance(value, list):
                fields[key] = tuple(value)
            elif isinstance(value, str):
                fields[key] = value
            else:
                fields[key] = float(value)
    if "min_points" in fields:
        fields["min_points"] = int(fields["min_points"])
    profile = GraspProfile(**fields)
    profile.validate()
    return profile, bool(raw.get("robot_motion_allowed", False))


def colour_mask(rgb: np.ndarray, profile: GraspProfile) -> np.ndarray:
    """Apply the profile colour rule. Accepts 0..1 floats or 0..255 values."""
    profile.validate()
    values = np.asarray(rgb, dtype=float)
    if values.ndim != 2 or values.shape[1] < 3:
        raise ValueError("rgb must be an (N,3) array")
    values = values[:, :3]
    if float(np.nanmax(values, initial=0.0)) <= 1.0 + 1e-9:
        values = values * 255.0
    primary = values[:, CHANNELS[profile.channel]]
    mask = primary > profile.min_value
    for name, ratio in profile.ratios.items():
        mask &= primary > float(ratio) * values[:, CHANNELS[name]]
    for name, cap in profile.caps.items():
        mask &= values[:, CHANNELS[name]] < float(cap)
    return mask


def _clusters(points: np.ndarray, voxel: float) -> List[np.ndarray]:
    """Voxel-grid 26-connected components, largest first.

    Returns every component, not just the biggest: two objects of the same
    colour are two components, and a caller that only ever looks at the first
    one silently ignores the rest.
    """
    keys = np.floor(points / voxel).astype(np.int64)
    buckets: Dict[Tuple[int, int, int], List[int]] = {}
    for index, key in enumerate(map(tuple, keys)):
        buckets.setdefault(key, []).append(index)
    offsets = [(dx, dy, dz)
               for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
               if (dx, dy, dz) != (0, 0, 0)]
    seen: set = set()
    found: List[List[int]] = []
    for key in buckets:
        if key in seen:
            continue
        seen.add(key)
        queue = deque([key])
        members: List[int] = []
        while queue:
            current = queue.popleft()
            members.extend(buckets[current])
            for offset in offsets:
                neighbour = (current[0] + offset[0], current[1] + offset[1],
                             current[2] + offset[2])
                if neighbour in buckets and neighbour not in seen:
                    seen.add(neighbour)
                    queue.append(neighbour)
        found.append(members)
    found.sort(key=len, reverse=True)
    return [points[np.asarray(m, dtype=int)] for m in found]


def _describe_cluster(cluster: np.ndarray, total_clusters: int) -> Detection:
    """Centroid, extent and grasp yaw for one already-isolated cluster."""
    centroid = cluster.mean(axis=0)
    top_z = float(cluster[:, 2].max())

    # Principal axis in the XY plane. The eigenvector of the larger eigenvalue
    # is the long axis, and yaw is its angle.
    planar = cluster[:, :2] - cluster[:, :2].mean(axis=0)
    covariance = planar.T @ planar / max(len(planar) - 1, 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    long_axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    short_axis = eigenvectors[:, int(np.argmin(eigenvalues))]
    yaw = math.atan2(float(long_axis[1]), float(long_axis[0]))
    # A grasp is symmetric under a half turn; keep yaw in (-pi/2, pi/2].
    if yaw > math.pi / 2:
        yaw -= math.pi
    elif yaw <= -math.pi / 2:
        yaw += math.pi

    along_long = planar @ long_axis
    along_short = planar @ short_axis
    extent = np.array([float(along_long.max() - along_long.min()),
                       float(along_short.max() - along_short.min()),
                       float(cluster[:, 2].max() - cluster[:, 2].min())])

    return Detection(True, None, centroid, extent, top_z, yaw,
                     width_across_fingers_m=float(extent[1]),
                     point_count=int(len(cluster)),
                     candidate_clusters=total_clusters)


def detect_objects(xyz: np.ndarray, rgb: np.ndarray, profile: GraspProfile,
                   max_objects: int = 8) -> Tuple[List[Detection], Optional[str]]:
    """Detect EVERY profile-coloured object, largest cluster first.

    Returns the detections and, when the list is empty, the reason. Clusters
    smaller than ``profile.min_points`` are dropped rather than reported, so
    speckle noise does not turn into a grasp target.
    """
    profile.validate()
    points = np.asarray(xyz, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        return [], "point cloud must be an (N,3) array"
    if len(points) != len(np.asarray(rgb)):
        return [], "point and colour counts differ"
    if len(points) == 0:
        return [], "empty point cloud"

    keep = np.isfinite(points).all(axis=1) & colour_mask(rgb, profile)
    keep &= (points[:, 0] >= profile.x_m[0]) & (points[:, 0] <= profile.x_m[1])
    keep &= (points[:, 1] >= profile.y_m[0]) & (points[:, 1] <= profile.y_m[1])
    keep &= (points[:, 2] >= profile.z_m[0]) & (points[:, 2] <= profile.z_m[1])
    selected = points[keep]
    if len(selected) < profile.min_points:
        return [], (f"only {len(selected)} in-workspace {profile.channel} points "
                    f"(need {profile.min_points})")

    clusters = _clusters(selected, profile.voxel_m)
    big = [c for c in clusters if len(c) >= profile.min_points]
    if not big:
        return [], (f"largest cluster has {len(clusters[0]) if clusters else 0} "
                    f"points (need {profile.min_points})")
    return [_describe_cluster(c, len(clusters)) for c in big[:max_objects]], None


def detect_object(xyz: np.ndarray, rgb: np.ndarray,
                  profile: GraspProfile) -> Detection:
    """Detect the largest profile-coloured object in a base_link point cloud."""
    found, reason = detect_objects(xyz, rgb, profile)
    if not found:
        return Detection(False, reason)
    return found[0]


@dataclass
class GraspStages:
    """Three top-down waypoints for grasp_link, in base_link metres."""
    valid: bool
    reason: Optional[str] = None
    yaw_rad: float = 0.0
    pre_grasp_m: Optional[np.ndarray] = None
    grasp_m: Optional[np.ndarray] = None
    lift_m: Optional[np.ndarray] = None
    gripper_open_m: Optional[float] = None
    notes: List[str] = field(default_factory=list)

    def as_dict(self) -> Dict:
        def out(v):
            return None if v is None else [float(x) for x in np.asarray(v).ravel()]
        return {"valid": bool(self.valid), "reason": self.reason,
                "yaw_rad": float(self.yaw_rad),
                "pre_grasp_base_link_m": out(self.pre_grasp_m),
                "grasp_base_link_m": out(self.grasp_m),
                "lift_base_link_m": out(self.lift_m),
                "gripper_open_m": (None if self.gripper_open_m is None
                                   else float(self.gripper_open_m)),
                "notes": list(self.notes)}


def grasp_stages(detection: Detection, profile: GraspProfile) -> GraspStages:
    """Turn a detection into pre-grasp, grasp and lift points. No motion."""
    profile.validate()
    if not detection.valid or detection.centroid_m is None:
        return GraspStages(False, detection.reason or "detection is invalid")

    width = float(detection.width_across_fingers_m or 0.0)
    opening = width + profile.gripper_clearance_m
    if opening > profile.gripper_max_width_m:
        return GraspStages(False, f"object is {width * 1000:.1f} mm across the finger "
                                  f"axis; the RG2-FT stroke is "
                                  f"{profile.gripper_max_width_m * 1000:.0f} mm")

    # The physical RG2 fingers open downward.  Lift the grasp waypoint above
    # the geometric object centre so the fingers clear the tabletop/object
    # before closing (validated empty-load compensation: 18 mm).
    grasp_z = float(detection.top_z_m) - profile.grasp_depth_below_top_m + 0.018
    notes: List[str] = ["downward-opening gripper Z compensation +18 mm"]
    if grasp_z < profile.min_grasp_z_m:
        notes.append(f"grasp height raised from {grasp_z:.4f} m to the "
                     f"{profile.min_grasp_z_m:.4f} m floor")
        grasp_z = profile.min_grasp_z_m

    x, y = float(detection.centroid_m[0]), float(detection.centroid_m[1])
    grasp = np.array([x, y, grasp_z])
    pre_grasp = grasp + np.array([0.0, 0.0, profile.approach_height_m])
    lift = grasp + np.array([0.0, 0.0, profile.lift_height_m])

    for label, point in (("pre-grasp", pre_grasp), ("grasp", grasp), ("lift", lift)):
        if not (profile.x_m[0] <= point[0] <= profile.x_m[1] and
                profile.y_m[0] <= point[1] <= profile.y_m[1] and
                profile.z_m[0] <= point[2] <= profile.z_m[1]):
            return GraspStages(False, f"{label} point {np.round(point, 4).tolist()} "
                                      "leaves the permitted workspace")

    return GraspStages(True, None, float(detection.yaw_rad or 0.0),
                       pre_grasp, grasp, lift, opening, notes)
