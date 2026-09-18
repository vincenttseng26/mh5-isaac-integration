from pathlib import Path

from camera_calibration.config import BoardConfig, DEFAULT_CONFIG_PATH, load_config

A2_CONFIG_PATH = Path(__file__).resolve().parents[1] / "configs" / "board_config_a2.yaml"


def test_dataclass_defaults_match_required_spec():
    cfg = BoardConfig()
    assert cfg.squares_x == 5
    assert cfg.squares_y == 7
    assert cfg.square_length_m == 0.03
    assert cfg.marker_length_m == 0.0225
    assert cfg.dictionary == "DICT_6X6_250"
    cfg.validate()  # must not raise


def test_yaml_config_matches_required_spec():
    pytest_yaml = None
    try:
        import yaml  # noqa: F401
    except ImportError:
        import pytest

        pytest.skip("pyyaml not installed")

    cfg = load_config(DEFAULT_CONFIG_PATH)
    assert cfg.squares_x == 5
    assert cfg.squares_y == 7
    assert cfg.square_length_m == 0.03
    assert cfg.marker_length_m == 0.0225
    assert cfg.dictionary == "DICT_6X6_250"


def test_invalid_marker_length_rejected():
    import pytest

    cfg = BoardConfig(marker_length_m=0.05, square_length_m=0.03)  # marker > square
    with pytest.raises(ValueError):
        cfg.validate()


def test_num_internal_corners():
    cfg = BoardConfig(squares_x=5, squares_y=7)
    assert cfg.num_internal_corners == 4 * 6  # (5-1) * (7-1)


def test_a2_yaml_config_matches_manifest_spec():
    """configs/board_config_a2.yaml must match
    configs/a2_board_source/manifest.json exactly -- see provenance.json for
    the sha256 verification and how legacy_pattern was determined."""
    try:
        import yaml  # noqa: F401
    except ImportError:
        import pytest

        pytest.skip("pyyaml not installed")

    cfg = load_config(A2_CONFIG_PATH)
    assert cfg.squares_x == 10
    assert cfg.squares_y == 8
    assert cfg.square_length_m == 0.045
    assert cfg.marker_length_m == 0.032
    assert cfg.dictionary == "DICT_5X5_100"
    assert cfg.legacy_pattern is False
    assert cfg.num_internal_corners == 63
    cfg.validate()  # must not raise

    # independent 5x7 config must remain untouched by adding the A2 one
    default_cfg = load_config(DEFAULT_CONFIG_PATH)
    assert default_cfg.squares_x == 5
    assert default_cfg.squares_y == 7
    assert default_cfg.dictionary == "DICT_6X6_250"
