from __future__ import annotations

import numpy as np


def emergency_stop(scan_ranges: np.ndarray, stop_distance: float, scan_angles: np.ndarray | None = None, front_angle: float = 0.5) -> bool:
    if scan_angles is None:
        return bool(np.any(scan_ranges < stop_distance))
    mask = np.abs(scan_angles) <= float(front_angle)
    if not np.any(mask):
        return False
    return bool(np.any(scan_ranges[mask] < stop_distance))
