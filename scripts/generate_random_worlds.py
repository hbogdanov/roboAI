import argparse
import os
import random
import re
from pathlib import Path


TARGET_DEFS = ["OBS1", "OBS2", "DESK_A", "DESK_B", "WALL_SEG", "MARKER_RED", "MARKER_BLUE"]


def _replace_translation(block: str, x: float, y: float) -> str:
    return re.sub(
        r"translation\s+[-0-9\.]+\s+[-0-9\.]+\s+([-0-9\.]+)",
        lambda m: f"translation {x:.3f} {y:.3f} {m.group(1)}",
        block,
        count=1,
    )


def randomize_world_text(text: str, rng: random.Random) -> str:
    for def_name in TARGET_DEFS:
        pattern = re.compile(rf"(DEF\s+{def_name}\s+Solid\s*\{{.*?\n\}})", re.S)
        m = pattern.search(text)
        if not m:
            continue
        block = m.group(1)
        x = rng.uniform(-0.75, 0.75)
        y = rng.uniform(-0.75, 0.75)
        text = text.replace(block, _replace_translation(block, x=x, y=y), 1)
    return text


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-world", default="webots_project/worlds/world_obstacles.wbt")
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--out-dir", default="webots_project/worlds/generated")
    args = parser.parse_args()

    base = Path(args.base_world)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    text = base.read_text(encoding="utf-8")
    rng = random.Random(args.seed)

    for i in range(args.count):
        t = randomize_world_text(text, rng=rng)
        out = out_dir / f"world_rand_{i+1:03d}.wbt"
        out.write_text(t, encoding="utf-8")
        print(f"Wrote {out}")


if __name__ == "__main__":
    main()
