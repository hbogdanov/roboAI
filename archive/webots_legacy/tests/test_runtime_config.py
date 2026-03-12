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

import roboai_controller  # noqa: E402


def test_resolve_initial_pose_defaults_for_office(monkeypatch):
    monkeypatch.delenv("ROBOAI_INITIAL_POSE", raising=False)
    x, y, theta = roboai_controller.resolve_initial_pose("world_office")
    assert (x, y, theta) == (-0.45, 0.0, 0.0)


def test_resolve_initial_pose_env_override(monkeypatch):
    monkeypatch.setenv("ROBOAI_INITIAL_POSE", "1.25, -0.75, 0.5")
    x, y, theta = roboai_controller.resolve_initial_pose("world_office")
    assert (x, y, theta) == (1.25, -0.75, 0.5)
