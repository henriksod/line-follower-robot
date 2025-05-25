#!/usr/bin/env python3

import json
import matplotlib.pyplot as plt
import argparse
from pathlib import Path


def plot_map_and_robot(map_data, robot_log_data):
    fig, ax = plt.subplots()

    # Plot track segments
    for segment in map_data["track_segments"]:
        for line in segment["track_lines"]:
            start = line["start"]
            end = line["end"]
            width = line["width"]

            # Draw line on the plot
            ax.plot([start["x"], end["x"]], [start["y"], end["y"]], "k-", lw=width / 10)

    # Plot robot's path
    x_positions = [entry["global_pose"]["position"]["x"] for entry in robot_log_data]
    y_positions = [entry["global_pose"]["position"]["y"] for entry in robot_log_data]

    ax.plot(x_positions, y_positions, "ro-", markersize=5, label="Robot Path")

    # Plot ir's path
    x_positions = [entry["ir_pose"]["position"]["x"] for entry in robot_log_data]
    y_positions = [entry["ir_pose"]["position"]["y"] for entry in robot_log_data]

    ax.plot(x_positions, y_positions, "bo-", markersize=5, label="IR Path")

    # Set plot limits and labels
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_aspect("equal")
    ax.set_xlabel("X position")
    ax.set_ylabel("Y position")
    ax.set_title("Robot Path on Map")
    ax.legend()

    plt.grid(True)
    plt.show()


def main(scenario_file, robot_log_file):
    # Load the map data
    with open(scenario_file, "r") as f:
        map_data = json.load(f)

    # Load the robot log data
    with open(robot_log_file, "r") as f:
        robot_log_data = [json.loads(line) for line in f]

    # Plot the map and robot path
    plot_map_and_robot(map_data, robot_log_data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot robot path on the map.")
    parser.add_argument("scenario_file", type=str, help="Path to the JSON file with scenario data.")
    parser.add_argument(
        "robot_log_file", type=str, help="Path to the JSON file with robot state data."
    )

    args = parser.parse_args()

    main(str(Path(args.scenario_file).resolve()), str(Path(args.robot_log_file).resolve()))
