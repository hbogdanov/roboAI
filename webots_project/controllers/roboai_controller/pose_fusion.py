from __future__ import annotations

from typing import Optional, Tuple, Dict, Any
import math


class PoseFusion:
    """
    Lightweight pose confidence + optional landmark correction.
    This is intentionally simple and does not claim EKF/SLAM behavior.
    """

    def __init__(self):
        self.confidence = 0.65

    def _clamp01(self, v: float) -> float:
        return max(0.0, min(1.0, v))

    def update_confidence(
        self,
        dt: float,
        encoders_available: bool,
        lidar_ranges_count: int,
        wheel_speed_mag: float,
    ) -> float:
        # Time decay
        c = self.confidence - 0.01 * max(0.0, dt)

        # Odometry availability
        if encoders_available:
            c += 0.01
        else:
            c -= 0.05

        # Large wheel speeds without correction reduce trust slightly
        if wheel_speed_mag > 6.0:
            c -= 0.02

        # Lidar scan availability stabilizes confidence
        if lidar_ranges_count > 50:
            c += 0.01

        self.confidence = self._clamp01(c)
        return self.confidence

    def maybe_correct_with_landmark(
        self,
        state,
        landmark_xy: Optional[Tuple[float, float]],
        max_dist_m: float = 0.35,
        alpha: float = 0.45,
    ) -> Dict[str, Any]:
        """
        If current pose is close to a known landmark, apply partial pose correction.
        Returns metadata with applied flag and pre/post error.
        """
        if landmark_xy is None:
            return {"applied": False}

        lx, ly = float(landmark_xy[0]), float(landmark_xy[1])
        dx = lx - float(state.x)
        dy = ly - float(state.y)
        dist = math.hypot(dx, dy)
        if dist > max_dist_m:
            return {"applied": False, "error_before_m": dist}

        # Blend pose toward landmark (simple correction, not a filter update).
        state.x = (1.0 - alpha) * float(state.x) + alpha * lx
        state.y = (1.0 - alpha) * float(state.y) + alpha * ly
        err_after = math.hypot(lx - float(state.x), ly - float(state.y))

        self.confidence = self._clamp01(self.confidence + 0.15)
        return {
            "applied": True,
            "landmark_x": lx,
            "landmark_y": ly,
            "error_before_m": dist,
            "error_after_m": err_after,
            "alpha": alpha,
            "confidence": self.confidence,
        }
