from __future__ import annotations

import numpy as np

SEMANTIC_NONE = 0
SEMANTIC_DOOR = 1
SEMANTIC_DESK = 2
SEMANTIC_EXIT = 3
SEMANTIC_PERSON = 4
SEMANTIC_BEACON = 5


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


def built_in_semantic_grid(name: str) -> np.ndarray:
    obstacle_grid = built_in_map(name)
    semantics = np.zeros_like(obstacle_grid, dtype=np.int8)
    normalized = name.strip().lower()
    if normalized == "empty":
        _paint_rect(semantics, 8, 8, 12, 12, SEMANTIC_BEACON)
        _paint_rect(semantics, semantics.shape[1] - 12, semantics.shape[0] - 12, semantics.shape[1] - 8, semantics.shape[0] - 8, SEMANTIC_EXIT)
    elif normalized == "office":
        _paint_rect(semantics, 22, 14, 24, 24, SEMANTIC_DOOR)
        _paint_rect(semantics, 50, 36, 52, 46, SEMANTIC_DOOR)
        _paint_rect(semantics, 30, 12, 42, 18, SEMANTIC_DESK)
        _paint_rect(semantics, 30, 40, 42, 46, SEMANTIC_DESK)
        _paint_rect(semantics, 8, 8, 12, 12, SEMANTIC_BEACON)
        _paint_rect(semantics, 68, 48, 74, 54, SEMANTIC_EXIT)
    elif normalized == "cluttered":
        _paint_rect(semantics, 9, 9, 18, 18, SEMANTIC_DESK)
        _paint_rect(semantics, 24, 14, 32, 26, SEMANTIC_PERSON)
        _paint_rect(semantics, 54, 54, 60, 60, SEMANTIC_BEACON)
        _paint_rect(semantics, 60, 8, 66, 14, SEMANTIC_EXIT)
    elif normalized == "narrow":
        _paint_rect(semantics, 30, 18, 32, 24, SEMANTIC_DOOR)
        _paint_rect(semantics, 48, 36, 50, 42, SEMANTIC_DOOR)
        _paint_rect(semantics, 14, 20, 24, 24, SEMANTIC_DESK)
        _paint_rect(semantics, 56, 36, 66, 40, SEMANTIC_PERSON)
        _paint_rect(semantics, 8, 8, 12, 12, SEMANTIC_BEACON)
        _paint_rect(semantics, 70, 48, 76, 54, SEMANTIC_EXIT)
    elif normalized == "maze":
        _paint_rect(semantics, 10, 10, 14, 14, SEMANTIC_BEACON)
        _paint_rect(semantics, 62, 62, 68, 68, SEMANTIC_EXIT)
        _paint_rect(semantics, 28, 20, 32, 24, SEMANTIC_PERSON)
        _paint_rect(semantics, 52, 44, 56, 48, SEMANTIC_DESK)
    semantics[obstacle_grid] = SEMANTIC_NONE
    return semantics


def built_in_beacon_points(name: str) -> list[tuple[int, int]]:
    grid = built_in_semantic_grid(name)
    ys, xs = np.nonzero(grid == SEMANTIC_BEACON)
    return [(int(x), int(y)) for y, x in zip(ys, xs)]


def _paint_rect(grid: np.ndarray, x0: int, y0: int, x1: int, y1: int, value: int) -> None:
    grid[y0:y1, x0:x1] = np.int8(value)
