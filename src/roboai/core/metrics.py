from __future__ import annotations

import csv
import json
from dataclasses import asdict
from pathlib import Path

from roboai.core.types import RunMetrics


def write_metrics_json(path: str | Path, metrics: RunMetrics) -> None:
    Path(path).write_text(json.dumps(asdict(metrics), indent=2), encoding="utf-8")


def write_metrics_csv(path: str | Path, metrics_list: list[RunMetrics]) -> None:
    rows = [asdict(item) for item in metrics_list]
    if not rows:
        return
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
