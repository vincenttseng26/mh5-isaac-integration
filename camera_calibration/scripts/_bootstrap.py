"""Adds the repo root to sys.path so `import camera_calibration...` works
when these scripts are run directly (python scripts/foo.py) instead of via
`python -m camera_calibration.scripts.foo`."""
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
