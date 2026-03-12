from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover - optional dependency behavior
    cv2 = None


OCCUPANCY_CMAP = ListedColormap([
    "#101820",  # unknown
    "#e8ecef",  # free
    "#ffd166",  # occupied
])
OCCUPANCY_NORM = BoundaryNorm([-1.5, -0.5, 0.5, 1.5], OCCUPANCY_CMAP.N)


def save_run_artifacts(
    demo_path,
    video_path,
    final_map_path,
    coverage_path,
    obstacle_grid,
    resolution,
    frames,
):
    _save_final_map(final_map_path, obstacle_grid, resolution, frames[-1])
    _save_coverage_curve(coverage_path, [frame.coverage for frame in frames])
    _save_demo_gif(demo_path, obstacle_grid, resolution, frames)
    _save_demo_mp4(video_path, obstacle_grid, resolution, frames)


def _save_final_map(path, obstacle_grid, resolution, frame):
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(obstacle_grid, origin="lower", cmap="gray_r", alpha=0.25)
    ax.imshow(frame.occupancy, origin="lower", cmap=OCCUPANCY_CMAP, norm=OCCUPANCY_NORM, alpha=0.95)
    if frame.trajectory:
        xs = [x / resolution for x, _ in frame.trajectory]
        ys = [y / resolution for _, y in frame.trajectory]
        ax.plot(xs, ys, color="tab:orange", linewidth=2.0)
    if frame.planner_path:
        px = [x / resolution for x, _ in frame.planner_path]
        py = [y / resolution for _, y in frame.planner_path]
        ax.plot(px, py, color="tab:cyan", linewidth=1.5, linestyle="--")
    if frame.frontier_points:
        fx = [x / resolution for x, _ in frame.frontier_points]
        fy = [y / resolution for _, y in frame.frontier_points]
        ax.scatter(fx, fy, s=8, c="red")
    ax.scatter(
        [frame.robot_pose.x / resolution],
        [frame.robot_pose.y / resolution],
        c="white",
        edgecolors="black",
        s=70,
        zorder=5,
    )
    ax.set_title("Final explored map")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def _save_coverage_curve(path, coverage_history):
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(range(len(coverage_history)), coverage_history, color="tab:green")
    ax.set_xlabel("step")
    ax.set_ylabel("coverage")
    ax.set_ylim(0.0, 1.0)
    ax.set_title("Coverage curve")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def _save_demo_gif(path, obstacle_grid, resolution, frames):
    fig, ax = plt.subplots(figsize=(6, 6))

    def update(frame_index):
        ax.clear()
        ax.imshow(obstacle_grid, origin="lower", cmap="gray_r", alpha=0.25)
        frame = frames[frame_index]
        ax.imshow(frame.occupancy, origin="lower", cmap=OCCUPANCY_CMAP, norm=OCCUPANCY_NORM, alpha=0.95)
        xs = [x / resolution for x, _ in frame.trajectory]
        ys = [y / resolution for _, y in frame.trajectory]
        ax.plot(xs, ys, color="tab:orange", linewidth=2.0)
        if frame.planner_path:
            px = [x / resolution for x, _ in frame.planner_path]
            py = [y / resolution for _, y in frame.planner_path]
            ax.plot(px, py, color="tab:cyan", linewidth=1.5, linestyle="--")
        if frame.frontier_points:
            fx = [x / resolution for x, _ in frame.frontier_points]
            fy = [y / resolution for _, y in frame.frontier_points]
            ax.scatter(fx, fy, s=8, c="red")
        ax.scatter(
            [frame.robot_pose.x / resolution],
            [frame.robot_pose.y / resolution],
            c="white",
            edgecolors="black",
            s=70,
            zorder=5,
        )
        ax.set_title(f"Coverage {frame.coverage:.2f}")
        return []

    frame_count = max(1, len(frames))
    anim = FuncAnimation(fig, update, frames=frame_count, interval=120, blit=False)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    anim.save(path, writer=PillowWriter(fps=8))
    plt.close(fig)


def _save_demo_mp4(path, obstacle_grid, resolution, frames):
    if cv2 is None:
        return
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    images = [_render_frame_array(obstacle_grid, frame, resolution) for frame in frames]
    if not images:
        return
    height, width = images[0].shape[:2]
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        8.0,
        (width, height),
    )
    for image in images:
        writer.write(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    writer.release()


def _render_frame_array(obstacle_grid, frame, resolution):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(obstacle_grid, origin="lower", cmap="gray_r", alpha=0.25)
    ax.imshow(frame.occupancy, origin="lower", cmap=OCCUPANCY_CMAP, norm=OCCUPANCY_NORM, alpha=0.95)
    if frame.trajectory:
        xs = [x / resolution for x, _ in frame.trajectory]
        ys = [y / resolution for _, y in frame.trajectory]
        ax.plot(xs, ys, color="tab:orange", linewidth=2.0)
    if frame.planner_path:
        px = [x / resolution for x, _ in frame.planner_path]
        py = [y / resolution for _, y in frame.planner_path]
        ax.plot(px, py, color="tab:cyan", linewidth=1.5, linestyle="--")
    if frame.frontier_points:
        fx = [x / resolution for x, _ in frame.frontier_points]
        fy = [y / resolution for _, y in frame.frontier_points]
        ax.scatter(fx, fy, s=8, c="red")
    ax.scatter([frame.robot_pose.x / resolution], [frame.robot_pose.y / resolution], c="white", edgecolors="black", s=70)
    ax.set_title(f"Coverage {frame.coverage:.2f}")
    fig.tight_layout()
    fig.canvas.draw()
    image = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    image = image.reshape(fig.canvas.get_width_height()[::-1] + (4,))[:, :, :3]
    plt.close(fig)
    return image
