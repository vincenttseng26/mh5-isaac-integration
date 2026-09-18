import importlib
import sys
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent
PACKAGE_ROOT = TESTS_DIR.parent  # .../camera_calibration or .../mh5_zed_calibration
REPO_ROOT = PACKAGE_ROOT.parent

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# The package directory is not named "camera_calibration" in every checkout
# (e.g. it's "mh5_zed_calibration" on cyc-server). Modules inside the
# package use relative imports, so that's fine on its own, but the tests
# (and some scripts) import it by the literal name "camera_calibration".
# Import it under its real directory name and alias that into sys.modules --
# the same trick scripts/probe_zed_stage2.py already uses -- so
# `from camera_calibration.x import y` resolves regardless of the actual
# checkout folder name.
if "camera_calibration" not in sys.modules:
    pkg = importlib.import_module(PACKAGE_ROOT.name)
    sys.modules.setdefault("camera_calibration", pkg)
