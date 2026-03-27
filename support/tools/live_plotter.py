#!/usr/bin/env python3
# Copyright (c) 2024 Henrik Söderlund
from __future__ import annotations

"""Live matplotlib visualizer for the line follower simulation.

Reads a scenario JSON file once (for the static track geometry) and then
continuously polls an events log file that the running simulator writes to,
updating a matplotlib window in real time.

Intended to be launched as a subprocess by `./run.py simulate --visualize`.
Can also be run stand-alone:

    python3 support/tools/live_plotter.py \\
        --scenario $PWD/line_follower/scenarios/simple/trainer_scenario.json \\
        --events   /path/to/sim_events.json
"""

import argparse
import json
import math
import sys
from pathlib import Path

from support import check_packages

check_packages("matplotlib")
import matplotlib.animation as animation  # noqa: E402
import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

# ── constants ────────────────────────────────────────────────────────────────
_POLL_INTERVAL_MS = 80  # animation frame interval (≈12 fps; light on CPU)
_ROBOT_ARROW_LEN = 0.08  # metres
_ROBOT_ARROW_WID = 0.05  # metres
_IR_BAR_HALF_WIDTH = 0.030  # 15 sensors × 4 mm / 2

# ── helpers ──────────────────────────────────────────────────────────────────


def _yaw_from_quat(q: dict) -> float:
    """Extract Z-axis yaw from a quaternion dict {w, x, y, z}."""
    w, z = q.get("w", 1.0), q.get("z", 0.0)
    return 2.0 * math.atan2(z, w)


def _pose_pos(pose: dict):
    p = pose["position"]
    return p["x"], p["y"]


# ── track drawing (static) ───────────────────────────────────────────────────


def draw_track(ax, scenario: dict) -> None:
    """Draw all track line segments onto *ax* once."""
    for segment in scenario.get("track_segments", []):
        seg_pose = segment.get("pose", {})
        seg_yaw = _yaw_from_quat(seg_pose.get("rotation", {"w": 1.0, "z": 0.0}))
        seg_ox = seg_pose.get("position", {}).get("x", 0.0)
        seg_oy = seg_pose.get("position", {}).get("y", 0.0)
        cos_y, sin_y = math.cos(seg_yaw), math.sin(seg_yaw)

        for line in segment.get("track_lines", []):
            if not line.get("visible", True):
                continue

            s, e = line["start"], line["end"]

            # Rotate + translate line endpoints from segment-local to world
            sx = cos_y * s["x"] - sin_y * s["y"] + seg_ox
            sy = sin_y * s["x"] + cos_y * s["y"] + seg_oy
            ex = cos_y * e["x"] - sin_y * e["y"] + seg_ox
            ey = sin_y * e["x"] + cos_y * e["y"] + seg_oy

            # whiteness: 0.0 = black track, 1.0 = white background
            whiteness = line.get("whiteness", 0.0)
            color = str(whiteness) if whiteness > 0 else "black"
            width_pts = max(1.0, line.get("width", 10.0) / 8.0)
            ax.plot([sx, ex], [sy, ey], color=color, lw=width_pts, solid_capstyle="round", zorder=1)


# ── live reader ───────────────────────────────────────────────────────────────


class _LiveReader:
    """Incrementally reads new lines from a growing line-delimited JSON file."""

    def __init__(self, path: Path) -> None:
        self._path = path
        self._fh = None
        # accumulated robot path
        self.robot_xs: list[float] = []
        self.robot_ys: list[float] = []
        # latest poses
        self.robot_pose: dict | None = None
        self.ir_pose: dict | None = None

    def ensure_open(self) -> bool:
        """Try to open the file; return True once open."""
        if self._fh is not None:
            return True
        if self._path.exists():
            self._fh = open(self._path, "r")
            return True
        return False

    def poll(self) -> bool:
        """Read all new lines. Returns True if any data was consumed."""
        if not self.ensure_open():
            return False
        consumed = False
        while True:
            line = self._fh.readline()
            if not line:
                break
            line = line.strip()
            if not line:
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            robot_pose = event.get("global_pose")
            ir_pose = event.get("ir_pose")
            if robot_pose:
                self.robot_pose = robot_pose
                self.robot_xs.append(robot_pose["position"]["x"])
                self.robot_ys.append(robot_pose["position"]["y"])
            if ir_pose:
                self.ir_pose = ir_pose
            consumed = True
        return consumed


# ── public entry point ────────────────────────────────────────────────────────


def run(scenario_path: str, events_path: str) -> None:
    """Show the live visualization window.

    Must be called from the **main thread** (macOS GUI requirement).
    Blocks until the window is closed.
    """
    spath = Path(scenario_path)
    if not spath.exists():
        sys.exit(f"[live_plotter] Scenario file not found: {spath}")

    with open(spath) as f:
        scenario = json.load(f)

    reader = _LiveReader(Path(events_path))

    # ── figure setup ─────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")
    ax.set_facecolor("#f5f2eb")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Line Follower — Live Simulation")
    ax.grid(True, alpha=0.3, zorder=0)

    draw_track(ax, scenario)

    (path_line,) = ax.plot(
        [], [], color="#e03030", lw=1.0, alpha=0.55, label="Robot path", zorder=2
    )

    robot_arrow = ax.annotate(
        "",
        xy=(0.0, 0.0),
        xytext=(0.0, 0.0),
        arrowprops={"arrowstyle": "-|>", "color": "#3366cc", "lw": 2, "mutation_scale": 20},
        zorder=4,
    )

    (robot_dot,) = ax.plot([], [], "o", color="#3366cc", ms=8, zorder=5)

    (ir_bar,) = ax.plot(
        [], [], "-", color="#e07010", lw=4, alpha=0.85, solid_capstyle="round", zorder=3
    )

    ax.legend(
        handles=[
            mpatches.Patch(color="#3366cc", label="Robot"),
            mpatches.Patch(color="#e07010", label="IR sensor array"),
            mpatches.Patch(color="#e03030", alpha=0.55, label="Robot path"),
        ],
        loc="upper right",
        fontsize=8,
    )

    waiting_text = ax.text(
        0.5,
        0.5,
        "Waiting for simulation to start\u2026",
        transform=ax.transAxes,
        ha="center",
        va="center",
        fontsize=12,
        color="gray",
        zorder=6,
    )

    def update(_frame):
        reader.poll()

        if not reader.robot_xs:
            return path_line, robot_dot, ir_bar

        waiting_text.set_visible(False)
        path_line.set_data(reader.robot_xs, reader.robot_ys)

        if reader.robot_pose:
            rx, ry = _pose_pos(reader.robot_pose)
            yaw = _yaw_from_quat(reader.robot_pose["rotation"])
            robot_dot.set_data([rx], [ry])
            robot_arrow.set_position((rx, ry))
            robot_arrow.xy = (
                rx + math.cos(yaw) * _ROBOT_ARROW_LEN,
                ry + math.sin(yaw) * _ROBOT_ARROW_LEN,
            )
            robot_arrow.xytext = (rx, ry)

        if reader.ir_pose:
            ix, iy = _pose_pos(reader.ir_pose)
            ir_yaw = _yaw_from_quat(reader.ir_pose["rotation"])
            px = math.cos(ir_yaw)
            py = math.sin(ir_yaw)
            ir_bar.set_data(
                [ix - px * _IR_BAR_HALF_WIDTH, ix + px * _IR_BAR_HALF_WIDTH],
                [iy - py * _IR_BAR_HALF_WIDTH, iy + py * _IR_BAR_HALF_WIDTH],
            )

        ax.relim()
        ax.autoscale_view()
        return path_line, robot_dot, ir_bar

    ani = animation.FuncAnimation(  # noqa: F841
        fig, update, interval=_POLL_INTERVAL_MS, blit=False, cache_frame_data=False
    )
    plt.tight_layout()
    plt.show()


# ── CLI entry point ───────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Live 2-D visualizer for the line follower simulation."
    )
    parser.add_argument(
        "--scenario", required=True, help="Path to the scenario JSON file (for the track)."
    )
    parser.add_argument(
        "--events", required=True, help="Path to the running simulator's events log JSON file."
    )
    args = parser.parse_args()
    run(args.scenario, args.events)


if __name__ == "__main__":
    main()
