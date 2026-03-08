import pathlib
import sys


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
CTRL_DIR = REPO_ROOT / "webots_project" / "controllers" / "roboai_controller"
sys.path.insert(0, str(CTRL_DIR))

from perception import detect_color_marker_bgra  # noqa: E402


def _make_bgra(width: int, height: int, b: int, g: int, r: int):
    px = bytes([b, g, r, 255])
    return px * (width * height)


def test_detect_red_marker():
    img = _make_bgra(16, 16, b=20, g=20, r=240)
    out = detect_color_marker_bgra(img, 16, 16)
    assert out["detected"] is True
    assert out["label"] == "red"


def test_no_image_returns_none():
    out = detect_color_marker_bgra(None, 0, 0)
    assert out["detected"] is False
    assert out["label"] == "none"
