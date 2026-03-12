from __future__ import annotations

import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from roboai.sim.grid2d.maps import built_in_map


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, choices=["empty", "office", "cluttered", "narrow", "maze"])
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    grid = built_in_map(args.map)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(grid, origin="lower", cmap="gray_r")
    ax.set_title(args.map)
    fig.tight_layout()
    fig.savefig(args.out, dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    main()
