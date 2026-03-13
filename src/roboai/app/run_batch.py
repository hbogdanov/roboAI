from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from roboai.app.run_demo import run_demo
from roboai.core.metrics import write_metrics_csv, write_metrics_json


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--maps", nargs="+", required=True)
    parser.add_argument("--planners", nargs="+", required=True)
    parser.add_argument("--frontier-policies", nargs="+", default=["naive"])
    parser.add_argument("--seeds", nargs="+", type=int, required=True)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--coverage-goal", type=float, default=None)
    parser.add_argument("--disturbance", choices=["none", "moving_obstacle", "temporary_block"], default="none")
    parser.add_argument("--range-noise-std", type=float, default=0.0)
    parser.add_argument("--dropout-prob", type=float, default=0.0)
    parser.add_argument("--pose-noise-std", type=float, default=0.0)
    parser.add_argument("--write-run-artifacts", action="store_true")
    args = parser.parse_args()

    Path("reports").mkdir(exist_ok=True)
    metrics_list = []
    for map_name in args.maps:
        for planner_name in args.planners:
            for frontier_policy in args.frontier_policies:
                for seed in args.seeds:
                    metrics = run_demo(
                        map_name,
                        planner_name,
                        seed,
                        max_steps=args.max_steps,
                        coverage_goal=args.coverage_goal,
                        write_artifacts=args.write_run_artifacts,
                        frontier_policy=frontier_policy,
                        disturbance_name=args.disturbance,
                        range_noise_std=args.range_noise_std,
                        dropout_prob=args.dropout_prob,
                        pose_noise_std=args.pose_noise_std,
                    )
                    metrics_list.append(metrics)
                    write_metrics_json(
                        Path("reports") / f"metrics_{map_name}_{planner_name}_{frontier_policy}_seed{seed}.json",
                        metrics,
                    )

    write_metrics_csv(Path("reports") / "batch_metrics.csv", metrics_list)
    _write_summary_plots(Path("reports"), metrics_list)
    _write_summary_json(Path("reports") / "batch_summary.json", metrics_list)
    _write_summary_markdown(Path("reports") / "batch_summary.md", metrics_list)


def _write_summary_plots(report_dir: Path, metrics_list) -> None:
    by_planner: dict[str, list] = {}
    for item in metrics_list:
        by_planner.setdefault(_series_label(item), []).append(item)

    planners = list(by_planner)
    _bar_plot(
        report_dir / "success_rate_by_planner.png",
        planners,
        [sum(1.0 for item in by_planner[p] if item.success) / len(by_planner[p]) for p in planners],
        "success rate",
        "Success Rate by Planner",
    )
    _bar_plot(
        report_dir / "path_length_by_planner.png",
        planners,
        [_mean([item.path_length for item in by_planner[p]]) for p in planners],
        "path length",
        "Path Length by Planner",
    )
    _bar_plot(
        report_dir / "runtime_by_planner.png",
        planners,
        [_mean([item.runtime_seconds for item in by_planner[p]]) for p in planners],
        "runtime (s)",
        "Runtime by Planner",
    )
    _coverage_vs_time_plot(report_dir / "coverage_vs_time.png", planners, by_planner)
    _map_coverage_plot(report_dir / "batch_summary_coverage.png", metrics_list)


def _map_coverage_plot(path: Path, metrics_list) -> None:
    grouped: dict[str, list[float]] = {}
    for item in metrics_list:
        grouped.setdefault(f"{item.map_name}:{item.planner_name}", []).append(item.coverage)
    labels = list(grouped)
    values = [_mean(grouped[label]) for label in labels]
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.bar(labels, values, color="tab:blue")
    ax.set_ylabel("mean coverage")
    ax.set_ylim(0.0, 1.0)
    ax.set_title("Batch coverage summary")
    ax.tick_params(axis="x", rotation=30)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def _bar_plot(path: Path, labels: list[str], values: list[float], ylabel: str, title: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.bar(labels, values, color="tab:blue")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.tick_params(axis="x", rotation=20)
    if ylabel == "success rate":
        ax.set_ylim(0.0, 1.0)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def _coverage_vs_time_plot(path: Path, planners: list[str], by_planner: dict[str, list]) -> None:
    fig, ax = plt.subplots(figsize=(8, 4))
    for planner in planners:
        curves = [np.asarray(item.coverage_history, dtype=float) for item in by_planner[planner]]
        max_len = max(len(curve) for curve in curves)
        padded = []
        for curve in curves:
            if len(curve) < max_len:
                curve = np.pad(curve, (0, max_len - len(curve)), constant_values=curve[-1])
            padded.append(curve)
        mean_curve = np.mean(np.vstack(padded), axis=0)
        ax.plot(range(len(mean_curve)), mean_curve, label=planner)
    ax.set_xlabel("step")
    ax.set_ylabel("coverage")
    ax.set_ylim(0.0, 1.0)
    ax.set_title("Coverage vs Time")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def _write_summary_json(path: Path, metrics_list) -> None:
    by_planner: dict[str, list] = {}
    for item in metrics_list:
        by_planner.setdefault(_series_label(item), []).append(item)
    payload = {
        planner: {
            "success_rate": sum(1.0 for item in items if item.success) / len(items),
            "mean_coverage": _mean([item.coverage for item in items]),
            "mean_path_length": _mean([item.path_length for item in items]),
            "mean_runtime_seconds": _mean([item.runtime_seconds for item in items]),
            "mean_collisions": _mean([float(item.collisions) for item in items]),
            "mean_replans": _mean([float(item.replans) for item in items]),
            "mean_replan_triggers": _mean([float(item.replan_triggers) for item in items]),
            "mean_recovery_events": _mean([float(item.recovery_events) for item in items]),
        }
        for planner, items in by_planner.items()
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _write_summary_markdown(path: Path, metrics_list) -> None:
    by_planner: dict[str, list] = {}
    for item in metrics_list:
        by_planner.setdefault(_series_label(item), []).append(item)

    lines = [
        "# Official Benchmark Summary",
        "",
        "| planner/policy | success rate | mean coverage | mean path length | mean runtime (s) | mean collisions | mean replans | mean recovery events |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for planner, items in by_planner.items():
        lines.append(
            "| "
            f"{planner} | "
            f"{sum(1.0 for item in items if item.success) / len(items):.3f} | "
            f"{_mean([item.coverage for item in items]):.3f} | "
            f"{_mean([item.path_length for item in items]):.2f} | "
            f"{_mean([item.runtime_seconds for item in items]):.2f} | "
            f"{_mean([float(item.collisions) for item in items]):.2f} | "
            f"{_mean([float(item.replans) for item in items]):.2f} | "
            f"{_mean([float(item.recovery_events) for item in items]):.2f} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _mean(values: list[float]) -> float:
    if not values:
        return 0.0
    return float(sum(values) / len(values))


def _series_label(item) -> str:
    label = f"{item.planner_name}/{item.frontier_policy}"
    if item.disturbance_name != "none":
        label += f"/{item.disturbance_name}"
    if item.range_noise_std > 0.0 or item.dropout_prob > 0.0 or item.pose_noise_std > 0.0:
        label += "/noisy"
    return label


if __name__ == "__main__":
    main()
