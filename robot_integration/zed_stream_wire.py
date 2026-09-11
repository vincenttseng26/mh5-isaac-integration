#!/usr/bin/env python3
"""Wire-format decoder for the read-only ZED live point-cloud stream.

One place to keep the point-record layout, so a sender-side schema bump does
not silently mis-parse in every consumer.  History:

  schema_version 2  dtype "xyz_f32_rgba_u8+rgb_u8"          16 bytes/point
  schema_version 3  dtype "xyz_f32_rgba_u8_uv_u16+rgb_u8"   20 bytes/point
                    (adds the source pixel u,v as uint16, so a point can be
                     mapped back to the RGB image)

Both are accepted.  The layout is chosen from the header's `dtype` string and
then cross-checked against point_payload_bytes / payload_points, so a future
mismatch fails loudly instead of producing plausible-looking garbage: reading
a 20-byte record as 16 bytes still yields a correctly aligned point every
fifth record (lcm(16,20)=80), which looks like a sparse but valid cloud.
"""
from __future__ import annotations

import numpy as np

SUPPORTED_SCHEMAS = (2, 3)

_LAYOUTS = {
    "xyz_f32_rgba_u8+rgb_u8": [("xyz", "<f4", 3), ("rgba", "u1", 4)],
    "xyz_f32_rgba_u8_uv_u16+rgb_u8": [("xyz", "<f4", 3), ("rgba", "u1", 4), ("uv", "<u2", 2)],
}
_BY_SCHEMA = {2: "xyz_f32_rgba_u8+rgb_u8", 3: "xyz_f32_rgba_u8_uv_u16+rgb_u8"}


def point_dtype(header: dict) -> np.dtype:
    """numpy dtype of one point record, from the header's dtype/schema."""
    name = header.get("dtype") or _BY_SCHEMA.get(header.get("schema_version"))
    if name not in _LAYOUTS:
        raise ValueError(f"unsupported point dtype {name!r} (schema {header.get('schema_version')})")
    return np.dtype(_LAYOUTS[name])


def decode_points(header: dict, payload: bytes):
    """(xyz Nx3 float32, rgba Nx4 uint8, uv Nx2 uint16 or None) from one frame.

    Raises ValueError on any inconsistency rather than returning a partial or
    misaligned cloud.
    """
    if header.get("schema_version") not in SUPPORTED_SCHEMAS:
        raise ValueError(f"unsupported schema_version {header.get('schema_version')!r}")
    dt = point_dtype(header)
    nbytes = int(header.get("point_payload_bytes", 0))
    if nbytes <= 0 or nbytes > len(payload):
        raise ValueError(f"point_payload_bytes {nbytes} vs payload {len(payload)}")
    if nbytes % dt.itemsize:
        raise ValueError(f"point_payload_bytes {nbytes} is not a multiple of the {dt.itemsize}-byte record")
    n = nbytes // dt.itemsize
    declared = header.get("payload_points")
    if declared is not None and int(declared) != n:
        raise ValueError(f"payload_points {declared} but {nbytes} bytes / {dt.itemsize} = {n}")
    rec = np.frombuffer(payload[:nbytes], dtype=dt)
    uv = rec["uv"] if "uv" in dt.names else None
    return rec["xyz"], rec["rgba"], uv


def rgb_image(header: dict, payload: bytes):
    """The trailing downsampled RGB image, or None when absent/inconsistent."""
    shape = tuple(header.get("rgb_shape", []))
    if len(shape) != 3:
        return None
    tail = payload[int(header.get("point_payload_bytes", 0)):]
    if len(tail) != int(np.prod(shape)):
        return None
    return np.frombuffer(tail, dtype=np.uint8).reshape(shape)
