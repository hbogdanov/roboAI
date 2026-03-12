from __future__ import annotations

from typing import Dict, Any


def detect_color_marker_bgra(image_bytes, width: int, height: int) -> Dict[str, Any]:
    """
    Very simple color marker detector on BGRA camera image.
    Reports dominant channel and confidence-like score.
    """
    if image_bytes is None or width <= 0 or height <= 0:
        return {"detected": False, "label": "none", "score": 0.0}

    # Sample every Nth pixel to reduce cost.
    step_px = 8
    b_sum = 0
    g_sum = 0
    r_sum = 0
    red_px = 0
    green_px = 0
    blue_px = 0
    n = 0
    stride = 4
    row_stride = width * stride
    for y in range(0, height, step_px):
        row = y * row_stride
        for x in range(0, width, step_px):
            i = row + x * stride
            b_sum += image_bytes[i + 0]
            g_sum += image_bytes[i + 1]
            r_sum += image_bytes[i + 2]
            b = image_bytes[i + 0]
            g = image_bytes[i + 1]
            r = image_bytes[i + 2]
            if r > 130 and r > g * 1.25 and r > b * 1.25:
                red_px += 1
            if g > 130 and g > r * 1.25 and g > b * 1.25:
                green_px += 1
            if b > 130 and b > r * 1.25 and b > g * 1.25:
                blue_px += 1
            n += 1

    if n <= 0:
        return {"detected": False, "label": "none", "score": 0.0}

    b_avg = b_sum / float(n)
    g_avg = g_sum / float(n)
    r_avg = r_sum / float(n)
    color_counts = {"red": red_px, "green": green_px, "blue": blue_px}
    label = max(color_counts, key=color_counts.get)
    score = color_counts[label] / float(max(1, n))
    detected = score >= 0.06
    return {
        "detected": bool(detected),
        "label": label if detected else "none",
        "score": round(float(score), 3),
        "rgb_avg": {"r": round(r_avg, 1), "g": round(g_avg, 1), "b": round(b_avg, 1)},
    }
