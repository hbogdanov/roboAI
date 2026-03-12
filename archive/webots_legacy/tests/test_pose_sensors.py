import pathlib
import sys
import types


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
CTRL_DIR = REPO_ROOT / "webots_project" / "controllers" / "roboai_controller"
sys.path.insert(0, str(CTRL_DIR))


if "controller" not in sys.modules:
    sys.modules["controller"] = types.SimpleNamespace(
        Robot=object,
        Lidar=object,
        Camera=object,
        GPS=object,
        Compass=object,
    )

from sensors import PoseSensorWrapper  # noqa: E402


def test_heading_from_compass_zero_heading():
    heading = PoseSensorWrapper.heading_from_compass((0.0, 1.0, 0.0))
    assert abs(heading - 0.0) < 1e-6


def test_heading_from_compass_left_turn():
    heading = PoseSensorWrapper.heading_from_compass((1.0, 0.0, 0.0))
    assert abs(heading - 1.5707963267948966) < 1e-6
