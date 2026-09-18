"""Board configuration: the single source of truth for board geometry.

Everything else (board_generator, detector, tests) imports BoardConfig
instead of hard-coding squares_x/square_length/etc.
"""
from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any, Dict

DEFAULT_CONFIG_PATH = Path(__file__).parent / "configs" / "board_config.yaml"


@dataclasses.dataclass
class BoardConfig:
    name: str = "charuco_5x7_30mm"
    squares_x: int = 5
    squares_y: int = 7
    square_length_m: float = 0.03
    marker_length_m: float = 0.0225
    dictionary: str = "DICT_6X6_250"
    # See configs/board_config.yaml for why this matters.
    legacy_pattern: bool = True

    def validate(self) -> None:
        if self.squares_x < 2 or self.squares_y < 2:
            raise ValueError("squares_x/squares_y must each be >= 2")
        if not (0 < self.marker_length_m < self.square_length_m):
            raise ValueError(
                "marker_length_m must be positive and smaller than square_length_m "
                f"(got marker={self.marker_length_m}, square={self.square_length_m})"
            )
        if self.square_length_m <= 0:
            raise ValueError("square_length_m must be positive")

    @property
    def num_internal_corners(self) -> int:
        """Max number of ChArUco corners detectable on this board."""
        return (self.squares_x - 1) * (self.squares_y - 1)

    @property
    def board_width_m(self) -> float:
        return self.squares_x * self.square_length_m

    @property
    def board_height_m(self) -> float:
        return self.squares_y * self.square_length_m

    def to_dict(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "BoardConfig":
        known = {f.name for f in dataclasses.fields(cls)}
        filtered = {k: v for k, v in data.items() if k in known}
        cfg = cls(**filtered)
        cfg.validate()
        return cfg

    @classmethod
    def from_yaml(cls, path: Path | str = DEFAULT_CONFIG_PATH) -> "BoardConfig":
        import yaml  # local import: keep pyyaml optional for pure-detection use

        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls.from_dict(data)

    def to_yaml(self, path: Path | str) -> None:
        import yaml

        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, sort_keys=False)


def load_config(path: Path | str | None = None) -> BoardConfig:
    """Load BoardConfig from YAML, falling back to hard-coded defaults
    (which already match the required spec) if PyYAML is unavailable."""
    try:
        return BoardConfig.from_yaml(path or DEFAULT_CONFIG_PATH)
    except ModuleNotFoundError:
        cfg = BoardConfig()
        cfg.validate()
        return cfg
