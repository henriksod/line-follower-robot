#!/usr/bin/env python3

import json
import subprocess
import time
from uuid import uuid4
from pathlib import Path
from statistics import mean
from concurrent.futures import ProcessPoolExecutor, as_completed
import cma

# Constants
SIMULATOR_PATH = "./bazel-bin/line_follower/deployment/simulator/ph0_line_follower/example"
POPULATION_SIZE = 10
TIMEOUT_SECONDS = 1200  # 20 minutes

# Global scenario path (for multiprocessing access)
SCENARIO_PATH = "./line_follower/scenarios/simple/trainer_scenario.json"

# Parameter bounds
PARAM_RANGES = {
    "pid_speed_parameters": {
        "proportional_gain": (1.0, 3.0),
        "integral_gain": (0.0, 0.1),
        "derivative_gain": (0.0, 0.1),
        "max_value": (80.0, 120.0),
        "min_value": (0.0, 1.0),
    },
    "pid_steer_parameters": {
        "proportional_gain": (1.0, 3.0),
        "integral_gain": (0.0, 0.1),
        "derivative_gain": (0.0, 0.1),
        "max_value": (1.9, 2.1),
        "min_value": (-2.1, -1.9),
    },
    "max_forward_velocity": (0.05, 2.0),
    "turning_speed_ratio": (0.0, 1.0),
    "sharp_turn_forward_velocity": (0.05, 2.0),
    "sharp_turn_angular_velocity": (1.0, 3.0),
}

CALIB_ORDER = [
    ("pid_speed_parameters", "proportional_gain"),
    ("pid_speed_parameters", "integral_gain"),
    ("pid_speed_parameters", "derivative_gain"),
    ("pid_speed_parameters", "max_value"),
    ("pid_speed_parameters", "min_value"),
    ("pid_steer_parameters", "proportional_gain"),
    ("pid_steer_parameters", "integral_gain"),
    ("pid_steer_parameters", "derivative_gain"),
    ("pid_steer_parameters", "max_value"),
    ("pid_steer_parameters", "min_value"),
    ("max_forward_velocity",),
    ("turning_speed_ratio",),
    ("sharp_turn_forward_velocity",),
    ("sharp_turn_angular_velocity",),
]


def flatten_calibration(calib):
    values = []
    for key in CALIB_ORDER:
        section = calib
        for part in key:
            section = section[part]
        values.append(section)
    return values


def unflatten_calibration(values):
    calib = {
        "pid_speed_parameters": {},
        "pid_steer_parameters": {},
    }
    for val, key in zip(values, CALIB_ORDER, strict=False):
        if len(key) == 2:
            calib[key[0]][key[1]] = val
        else:
            calib[key[0]] = val
    return calib


def run_simulation(scenario_path, calibration):
    temp_dir = Path("temp_run")
    temp_dir.mkdir(exist_ok=True)

    calib_path = temp_dir / f"calib_{uuid4().hex}.json"
    output_path = temp_dir / f"log_{uuid4().hex}.json"

    with open(calib_path, "w") as f:
        json.dump(calibration, f)

    try:
        subprocess.run(
            [
                SIMULATOR_PATH,
                f"--scenario={scenario_path}",
                f"--calibration={calib_path}",
                f"--events_log_json={output_path}",
                "--verbosity=error",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=60,
        )

        with open(output_path) as f:
            stats = [
                json.loads(line)["line_following_statistics"]
                for line in f
                if "line_following_statistics" in line
            ]

            if not stats:
                return 0

            avg_error = mean([x["tracking_error"] for x in stats])
            avg_speed = mean([x["average_speed"] for x in stats])
            time_on_line = stats[-1]["time_spent_on_line"]

            reward = avg_speed * time_on_line - 10.0 * avg_error
            return reward

    except subprocess.TimeoutExpired as e:
        print(f"Simulation failed: {e}")
        return 0


def fitness(x):
    calib = unflatten_calibration(x)
    reward = run_simulation(SCENARIO_PATH, calib)
    return -reward  # CMA-ES minimizes


def train_cma_es():

    # Fix: Properly extract bounds
    bounds = []
    for key in CALIB_ORDER:
        if len(key) == 2:
            section, param = key
            bounds.append(PARAM_RANGES[section][param])
        else:
            bounds.append(PARAM_RANGES[key[0]])

    lower_bounds, upper_bounds = zip(*bounds, strict=False)
    x0 = [(low + high) / 2 for low, high in bounds]

    es = cma.CMAEvolutionStrategy(
        x0,
        0.5,
        {
            "bounds": [lower_bounds, upper_bounds],
            "popsize": POPULATION_SIZE,
            "maxfevals": 1000,
        },
    )

    start_time = time.time()

    while not es.stop():
        if time.time() - start_time > TIMEOUT_SECONDS:
            print("Training stopped due to timeout.")
            break

        solutions = es.ask()

        # Evaluate solutions in parallel
        with ProcessPoolExecutor() as executor:
            futures = {executor.submit(fitness, x): i for i, x in enumerate(solutions)}
            fitnesses = [None] * len(solutions)
            for future in as_completed(futures):
                idx = futures[future]
                fitnesses[idx] = future.result()

        es.tell(solutions, fitnesses)
        es.disp()

    best_params = es.result.xbest
    best_calib = unflatten_calibration(best_params)

    with open("best_calibration.json", "w") as f:
        json.dump(best_calib, f, indent=2)
    print("Best calibration saved to best_calibration.json")


if __name__ == "__main__":
    train_cma_es()
