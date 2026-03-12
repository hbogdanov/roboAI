import argparse
import glob
import os
import shutil
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
LOG_ROOT = REPO_ROOT / "data" / "logs"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--batch-dir", required=True)
    args = parser.parse_args()

    batch_dir = Path(args.batch_dir).resolve()
    batch_dir.mkdir(parents=True, exist_ok=True)

    all_logs = sorted(glob.glob(str(LOG_ROOT / "run_*.json")), key=os.path.getmtime)
    if not all_logs:
        raise SystemExit("No run_*.json files found in data/logs")

    latest = Path(all_logs[-1])
    existing_names = {p.name for p in batch_dir.glob("run_*.json")}
    if latest.name in existing_names:
        print(f"Latest log already archived: {latest.name}")
        return

    trial_idx = len(existing_names) + 1
    dst = batch_dir / f"run_{trial_idx:02d}_{latest.name}"
    shutil.copy2(latest, dst)
    print(f"Archived {latest.name} -> {dst.name}")


if __name__ == "__main__":
    main()
