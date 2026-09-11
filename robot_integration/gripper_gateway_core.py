"""Transport-free safety state machine for the physical RG2-FT gateway.

The network/Modbus process owns one instance.  Motion is possible only after
COMMAND -> arm -> one matching nonce.  Every accepted motion consumes the arm
and returns to MIRROR, including failed Modbus operations.
"""

from __future__ import annotations

from dataclasses import dataclass
import secrets
from typing import Any, Callable, Dict


MIN_WIDTH_TENTH_MM = 0
MAX_WIDTH_TENTH_MM = 1000
MIN_FORCE_TENTH_N = 30
MAX_FORCE_TENTH_N = 120


class GripperRejection(ValueError):
    """A fail-closed request rejection suitable for returning to a client."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code
        self.detail = detail


@dataclass(frozen=True)
class GripperStatus:
    width_tenth_mm: int
    busy: bool
    grip_detected: bool

    def as_dict(self) -> Dict[str, Any]:
        return {
            "width_tenth_mm": self.width_tenth_mm,
            "width_mm": self.width_tenth_mm / 10.0,
            "busy": self.busy,
            "grip_detected": self.grip_detected,
        }


class GripperGatewayCore:
    def __init__(self, token: str, status_reader: Callable[[], GripperStatus],
                 motion_writer: Callable[[int, int], None],
                 stop_writer: Callable[[], None]):
        if not token:
            raise ValueError("a non-empty gateway token is required")
        self._token = token
        self._status_reader = status_reader
        self._motion_writer = motion_writer
        self._stop_writer = stop_writer
        self.mode = "MIRROR"
        self.armed = False
        self.nonce = None

    def _authorize(self, request: Dict[str, Any]) -> None:
        if request.get("token") != self._token:
            raise GripperRejection("BAD_TOKEN", "invalid command token")

    def _disarm(self) -> None:
        self.mode = "MIRROR"
        self.armed = False
        self.nonce = None

    def disconnect(self) -> None:
        """Revoke authority when the client connection that armed it closes."""
        self._disarm()

    def handle(self, request: Dict[str, Any]) -> Dict[str, Any]:
        try:
            return self._handle(request)
        except GripperRejection as exc:
            return {"ok": False, "code": exc.code, "error": exc.detail,
                    "mode": self.mode, "armed": self.armed}

    def _handle(self, request: Dict[str, Any]) -> Dict[str, Any]:
        command = request.get("cmd")
        if command == "status":
            return {"ok": True, "mode": self.mode, "armed": self.armed,
                    "gripper": self._status_reader().as_dict()}
        if command == "stop":
            # Emergency stop remains deliberately unauthenticated and available
            # in every state.  It also revokes any pending command authority.
            try:
                self._stop_writer()
            finally:
                self._disarm()
            return {"ok": True, "cmd": "stop", "mode": self.mode,
                    "armed": self.armed}
        if command == "set_mode":
            self._authorize(request)
            mode = request.get("mode")
            if mode not in ("MIRROR", "COMMAND"):
                raise GripperRejection("BAD_MODE", "mode must be MIRROR or COMMAND")
            self._disarm()
            self.mode = mode
            return {"ok": True, "mode": self.mode, "armed": False}
        if command == "arm":
            self._authorize(request)
            if self.mode != "COMMAND":
                raise GripperRejection("WRONG_MODE", "select COMMAND before arming")
            status = self._status_reader()
            if status.busy:
                raise GripperRejection("GRIPPER_BUSY", "gripper is already moving")
            self.nonce = secrets.token_urlsafe(18)
            self.armed = True
            return {"ok": True, "mode": self.mode, "armed": True,
                    "nonce": self.nonce, "gripper": status.as_dict()}
        if command == "move":
            self._authorize(request)
            if self.mode != "COMMAND" or not self.armed:
                raise GripperRejection("NOT_ARMED", "gateway is not armed")
            if request.get("nonce") != self.nonce:
                raise GripperRejection("BAD_NONCE", "missing or stale one-shot nonce")
            width = request.get("width_tenth_mm")
            force = request.get("force_tenth_n")
            if isinstance(width, bool) or not isinstance(width, int):
                raise GripperRejection("BAD_WIDTH", "width must be an integer in 0.1 mm")
            if isinstance(force, bool) or not isinstance(force, int):
                raise GripperRejection("BAD_FORCE", "force must be an integer in 0.1 N")
            if not MIN_WIDTH_TENTH_MM <= width <= MAX_WIDTH_TENTH_MM:
                raise GripperRejection("BAD_WIDTH", "width must be in 0..1000 (0.1 mm)")
            if not MIN_FORCE_TENTH_N <= force <= MAX_FORCE_TENTH_N:
                raise GripperRejection("BAD_FORCE", "force must be in 30..120 (0.1 N)")
            before = self._status_reader()
            if before.busy:
                raise GripperRejection("GRIPPER_BUSY", "gripper is already moving")
            try:
                self._motion_writer(width, force)
            finally:
                self._disarm()
            return {"ok": True, "cmd": "move", "accepted_width_tenth_mm": width,
                    "accepted_force_tenth_n": force, "before": before.as_dict(),
                    "mode": self.mode, "armed": self.armed}
        raise GripperRejection("BAD_COMMAND", "unknown command")
