from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from roboai.app.run_batch import _mean
from roboai.app.run_demo import run_demo
from roboai.app.run_multi_demo import run_multi_demo
from roboai.core.frontier_model import DEFAULT_WEIGHTS, save_frontier_model
from roboai.core.metrics import write_metrics_csv


def main() -> None:
    Path("reports").mkdir(exist_ok=True)
    model_path = Path("reports") / "frontier_model_weights.json"
    if not model_path.exists():
        save_frontier_model(model_path, DEFAULT_WEIGHTS)
    records = []

    frontier_cases = [
        ("naive", None),
        ("information_gain", None),
        ("semantic_information_gain", None),
        ("learned_linear", str(model_path)),
    ]
    for policy, model_path in frontier_cases:
        for seed in [1, 7]:
            records.append({
                "study": "frontier_policy",
                "variant": policy,
                "metrics": run_demo(
                    map_name="office",
                    planner_name="astar",
                    seed=seed,
                    frontier_policy=policy,
                    semantic_mode="enabled",
                    frontier_model_path=model_path,
                    write_artifacts=False,
                ),
            })

    for planner_name in ["astar", "hybrid"]:
        for seed in [1, 7]:
            records.append({
                "study": "disturbance_planner",
                "variant": planner_name,
                "metrics": run_demo(
                    map_name="office",
                    planner_name=planner_name,
                    seed=seed,
                    frontier_policy="semantic_information_gain",
                    disturbance_name="temporary_block",
                    write_artifacts=False,
                ),
            })

    for semantic_mode in ["disabled", "enabled"]:
        for seed in [1, 7]:
            records.append({
                "study": "noise_semantics",
                "variant": semantic_mode,
                "metrics": run_demo(
                    map_name="cluttered",
                    planner_name="hybrid",
                    seed=seed,
                    frontier_policy="semantic_information_gain",
                    semantic_mode=semantic_mode,
                    range_noise_std=0.015,
                    dropout_prob=0.03,
                    pose_noise_std=0.01,
                    write_artifacts=False,
                ),
            })

    for robot_mode in ["single", "multi"]:
        for seed in [1, 7]:
            metrics = (
                run_demo(
                    map_name="office",
                    planner_name="hybrid",
                    seed=seed,
                    frontier_policy="semantic_information_gain",
                    write_artifacts=False,
                )
                if robot_mode == "single"
                else run_multi_demo(
                    map_name="office",
                    planner_name="hybrid",
                    seed=seed,
                    write_artifacts=False,
                )
            )
            records.append({"study": "robot_count", "variant": robot_mode, "metrics": metrics})

    metrics_list = [record["metrics"] for record in records]
    write_metrics_csv(Path("reports") / "ablation_metrics.csv", metrics_list)
    _write_ablation_summary(Path("reports") / "ablation_summary.md", records)
    _write_policy_plot(Path("reports") / "ablation_policy_coverage.png", records)
    _write_robot_plot(Path("reports") / "ablation_robot_time_to_coverage.png", records)
    _write_json(Path("reports") / "ablation_summary.json", records)


def _write_ablation_summary(path: Path, records: list[dict]) -> None:
    lines = [
        "# Ablation Summary",
        "",
        "## Questions",
        "",
        "- Does smarter frontier scoring improve coverage and utility?",
        "- Does hybrid planning improve resilience under disturbances?",
        "- Do semantics help when sensing and localization are noisy?",
        "- Does two-robot exploration reduce time-to-coverage?",
        "",
    ]
    for study in ["frontier_policy", "disturbance_planner", "noise_semantics", "robot_count"]:
        lines.append(f"## {study.replace('_', ' ').title()}")
        lines.append("")
        lines.append("| variant | success rate | mean coverage | mean runtime (s) | mean replans | mean recovery events | time-to-coverage | overlap | near-conflicts |")
        lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
        for variant in sorted({record["variant"] for record in records if record["study"] == study}):
            items = [record["metrics"] for record in records if record["study"] == study and record["variant"] == variant]
            lines.append(
                f"| {variant} | "
                f"{_mean([1.0 if item.success else 0.0 for item in items]):.3f} | "
                f"{_mean([item.coverage for item in items]):.3f} | "
                f"{_mean([item.runtime_seconds for item in items]):.2f} | "
                f"{_mean([float(item.replans) for item in items]):.2f} | "
                f"{_mean([float(item.recovery_events) for item in items]):.2f} | "
                f"{_mean([float(item.time_to_coverage_step) for item in items]):.1f} | "
                f"{_mean([float(item.explored_overlap_ratio) for item in items]):.2f} | "
                f"{_mean([float(item.near_conflicts) for item in items]):.2f} |"
            )
        lines.append("")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_policy_plot(path: Path, records: list[dict]) -> None:
    relevant = [record for record in records if record["study"] in {"frontier_policy", "disturbance_planner", "noise_semantics"}]
    grouped: dict[str, list[float]] = {}
    for record in relevant:
        grouped.setdefault(f"{record['study']}:{record['variant']}", []).append(record["metrics"].coverage)
    labels = list(grouped)
    values = [_mean(grouped[label]) for label in labels]
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.bar(labels, values, color="#2a9d8f")
    ax.set_ylabel("mean coverage")
    ax.set_ylim(0.0, 1.0)
    ax.set_title("Ablation Coverage by Policy Variant")
    ax.tick_params(axis="x", rotation=30)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def _write_robot_plot(path: Path, records: list[dict]) -> None:
    grouped: dict[str, list[float]] = {}
    for record in records:
        if record["study"] != "robot_count":
            continue
        grouped.setdefault(record["variant"], []).append(float(record["metrics"].time_to_coverage_step))
    labels = list(grouped)
    values = [_mean(grouped[label]) for label in labels]
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.bar(labels, values, color=["#457b9d", "#e76f51"])
    ax.set_ylabel("steps to coverage goal")
    ax.set_title("Single vs Two-Robot Time to Coverage")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def _write_json(path: Path, records: list[dict]) -> None:
    payload = [
        {
            "study": record["study"],
            "variant": record["variant"],
            "metrics": asdict(record["metrics"]),
        }
        for record in records
    ]
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
