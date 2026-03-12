from __future__ import annotations

import numpy as np


def make_empty(width: int = 60, height: int = 60) -> np.ndarray:
    grid = np.zeros((height, width), dtype=bool)
    grid[[0, -1], :] = True
    grid[:, [0, -1]] = True
    return grid


def make_office(width: int = 80, height: int = 60) -> np.ndarray:
    grid = make_empty(width, height)
    grid[8:52, 22:24] = True
    grid[8:52, 50:52] = True
    grid[29:31, 22:52] = True
    grid[14:24, 22:24] = False
    grid[38:48, 22:24] = False
    grid[12:22, 50:52] = False
    grid[36:46, 50:52] = False
    grid[29:31, 34:40] = False
    grid[12:18, 32:40] = True
    grid[40:46, 32:40] = True
    return grid


def make_cluttered(width: int = 70, height: int = 70) -> np.ndarray:
    grid = make_empty(width, height)
    for x0, y0, x1, y1 in [
        (10, 10, 18, 18),
        (24, 14, 32, 26),
        (38, 8, 46, 18),
        (18, 38, 28, 48),
        (42, 36, 56, 50),
        (12, 54, 22, 62),
    ]:
        grid[y0:y1, x0:x1] = True
    return grid


def make_narrow(width: int = 80, height: int = 60) -> np.ndarray:
    grid = make_empty(width, height)
    grid[10:50, 30:32] = True
    grid[10:50, 48:50] = True
    grid[28:32, 32:48] = True
    grid[18:24, 30:32] = False
    grid[36:42, 48:50] = False
    grid[28:32, 38:42] = False
    grid[20:24, 14:24] = True
    grid[36:40, 56:66] = True
    return grid


def make_maze(width: int = 80, height: int = 80) -> np.ndarray:
    grid = make_empty(width, height)
    for x in (18, 34, 50, 66):
        grid[8:72, x:x + 2] = True
    for y in (18, 34, 50, 66):
        grid[y:y + 2, 8:72] = True

    openings = [
        (18, 12, 18),
        (18, 28, 34),
        (18, 44, 50),
        (18, 60, 66),
        (34, 20, 26),
        (34, 52, 58),
        (50, 12, 18),
        (50, 36, 42),
        (50, 60, 66),
        (66, 28, 34),
        (66, 52, 58),
    ]
    for wall_y, x0, x1 in openings:
        grid[wall_y:wall_y + 2, x0:x1] = False

    vertical_openings = [
        (18, 12, 18),
        (34, 28, 34),
        (34, 60, 66),
        (50, 20, 26),
        (50, 44, 50),
        (66, 12, 18),
        (66, 52, 58),
    ]
    for wall_x, y0, y1 in vertical_openings:
        grid[y0:y1, wall_x:wall_x + 2] = False
    return grid


def built_in_map(name: str) -> np.ndarray:
    normalized = name.strip().lower()
    if normalized == "empty":
        return make_empty()
    if normalized == "office":
        return make_office()
    if normalized == "cluttered":
        return make_cluttered()
    if normalized == "narrow":
        return make_narrow()
    if normalized == "maze":
        return make_maze()
    raise ValueError(f"Unknown map: {name}")
