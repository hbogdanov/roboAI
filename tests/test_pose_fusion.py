import pathlib
import sys


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
CTRL_DIR = REPO_ROOT / "webots_project" / "controllers" / "roboai_controller"
sys.path.insert(0, str(CTRL_DIR))

from pose_fusion import PoseFusion  # noqa: E402


class DummyState:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y


def test_confidence_update_bounds():
    f = PoseFusion()
    c = f.update_confidence(dt=0.1, encoders_available=True, lidar_ranges_count=120, wheel_speed_mag=1.0)
    assert 0.0 <= c <= 1.0


def test_landmark_correction_applies_near_goal():
    f = PoseFusion()
    s = DummyState(x=1.0, y=1.0)
    out = f.maybe_correct_with_landmark(s, landmark_xy=(1.1, 1.1), max_dist_m=0.5, alpha=0.5)
    assert out["applied"] is True
    assert s.x != 1.0 or s.y != 1.0


def test_landmark_correction_skips_far_goal():
    f = PoseFusion()
    s = DummyState(x=0.0, y=0.0)
    out = f.maybe_correct_with_landmark(s, landmark_xy=(10.0, 10.0), max_dist_m=0.5, alpha=0.5)
    assert out["applied"] is False
