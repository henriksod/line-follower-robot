#!/usr/bin/env python3

import json
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from statistics import mean
from uuid import uuid4

# Ensure the repo root is on sys.path so `support` is importable when this
# script is run directly (e.g. `python3 support/tools/trainer.py`).
_repo_root = Path(__file__).resolve().parent.parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

import cma  # noqa: E402

# Constants
# Path to the simulator binary, relative to the repo root inside the dazel container.
SIMULATOR_PATH = "./bazel-bin/line_follower/deployment/simulator/ph0_line_follower/example"
BAZEL_TARGET = "//line_follower/deployment/simulator/ph0_line_follower:example"

DEFAULT_SCENARIO_PATH = str(_repo_root / "line_follower/scenarios/simple/trainer_scenario.json")
DEFAULT_BASELINE_PATH = str(
    _repo_root / "line_follower/scenarios/calibration/example_calibration.json"
)
DEFAULT_OUTPUT_PATH = str(_repo_root / "best_calibration.json")
DEFAULT_POPULATION_SIZE = 10
DEFAULT_TIMEOUT_SECONDS = 1200  # 20 minutes
DEFAULT_MAX_EVALS = 1000

# Hard parameter bounds — wide enough for meaningful exploration around the baseline.
PARAM_RANGES = {
    "pid_speed_parameters": {
        "proportional_gain": (0.1, 5.0),
        "integral_gain": (0.0, 1.0),
        "derivative_gain": (0.0, 1.0),
        "max_value": (50.0, 200.0),
        "min_value": (0.0, 20.0),
    },
    "pid_steer_parameters": {
        "proportional_gain": (0.1, 5.0),
        "integral_gain": (0.0, 1.0),
        "derivative_gain": (0.0, 1.0),
        "max_value": (0.5, 5.0),
        "min_value": (-5.0, -0.5),
    },
    "max_forward_velocity": (0.01, 3.0),
    "turning_speed_ratio": (0.0, 1.0),
    "sharp_turn_forward_velocity": (0.01, 3.0),
    "sharp_turn_angular_velocity": (0.5, 5.0),
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


def _dazel_exec_prefix():
    """Return (DockerInstance, base docker-exec args) for running commands in the dazel container."""
    from support.dazel.dazel import DockerInstance

    di = DockerInstance.from_config()
    cmd = [di.docker_command, "exec", "-i", "-w", di.remote_directory]
    if di.docker_run_privileged:
        cmd.append("--privileged")
    if di.user:
        # .dazelrc sets DAZEL_USER with shell expressions like "$(id -u):$(id -g)".
        # dazel.py passes this through os.system() where the shell evaluates it,
        # but subprocess.run() lists bypass the shell so we must resolve it ourselves.
        user = re.sub(r"\$\(id -u\)", str(os.getuid()), di.user)
        user = re.sub(r"\$\(id -g\)", str(os.getgid()), user)
        cmd.append(f"--user={user}")
    cmd.append(di.instance_name)
    return di, cmd


def _ensure_container_running(di):
    """Start the dazel container if it is not already running."""
    if (
        not os.path.exists(di.dazel_run_file)
        or not di.is_running()
        or (
            os.path.exists(di.dockerfile)
            and os.path.getctime(di.dockerfile) > os.path.getctime(di.dazel_run_file)
        )
    ):
        rc = di.start()
        if rc:
            raise RuntimeError(f"Failed to start dazel container (exit code {rc})")


def _build_simulator():
    """Start the dazel container if needed, then build the simulator binary."""
    di, _ = _dazel_exec_prefix()
    _ensure_container_running(di)
    rc = di.send_command(["build", BAZEL_TARGET])
    if rc != 0:
        raise RuntimeError(f"Failed to build simulator: {BAZEL_TARGET}")


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
    for val, key in zip(values, CALIB_ORDER):
        if len(key) == 2:
            calib[key[0]][key[1]] = val
        else:
            calib[key[0]] = val
    return calib


def run_simulation(scenario_path, calibration):
    # Use absolute paths so spawned worker processes (macOS spawn) find the
    # temp directory regardless of their working directory.
    temp_dir = _repo_root / "temp_run"
    temp_dir.mkdir(exist_ok=True)

    calib_path = temp_dir / f"calib_{uuid4().hex}.json"
    output_path = temp_dir / f"log_{uuid4().hex}.json"

    with open(calib_path, "w") as f:
        json.dump(calibration, f)

    try:
        _, prefix = _dazel_exec_prefix()
        subprocess.run(
            prefix
            + [
                SIMULATOR_PATH,
                f"--scenario={scenario_path}",
                f"--calibration={calib_path}",
                f"--events_log_json={output_path}",
                "--verbosity=error",
                # Advance virtual time by kUpdateRateMicros+1 per outer loop
                # iteration so the simulation runs at full CPU speed instead of
                # being gated by the wall clock (10 ms real time per step).
                "--time_step_us=10001",
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
            return 0.0

        avg_error = mean([x["tracking_error"] for x in stats])
        avg_speed = mean([x["average_speed"] for x in stats])
        time_on_line = stats[-1]["time_spent_on_line"]

        # Reward: distance covered on-line at speed, penalised by tracking error.
        # Dividing by (1 + error) prevents high-error solutions from scoring well
        # even when they stay on the line by accident.
        return time_on_line * avg_speed / (1.0 + 5.0 * avg_error)

    except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as e:
        print(f"Simulation failed: {e}")
        return 0.0
    finally:
        calib_path.unlink(missing_ok=True)
        output_path.unlink(missing_ok=True)


def fitness(x, scenario_path):
    """Top-level picklable fitness function for ProcessPoolExecutor workers."""
    calib = unflatten_calibration(x)
    reward = run_simulation(scenario_path, calib)
    return -reward  # CMA-ES minimizes


def train(
    scenario_path=DEFAULT_SCENARIO_PATH,
    baseline_path=DEFAULT_BASELINE_PATH,
    output_path=DEFAULT_OUTPUT_PATH,
    population_size=DEFAULT_POPULATION_SIZE,
    timeout_seconds=DEFAULT_TIMEOUT_SECONDS,
    max_evals=DEFAULT_MAX_EVALS,
):
    """Run CMA-ES calibration optimisation, warm-started from baseline_path."""
    with open(baseline_path) as f:
        baseline = json.load(f)
    x0 = flatten_calibration(baseline)

    bounds = []
    for key in CALIB_ORDER:
        if len(key) == 2:
            section, param = key
            bounds.append(PARAM_RANGES[section][param])
        else:
            bounds.append(PARAM_RANGES[key[0]])

    lower_bounds, upper_bounds = zip(*bounds)

    # Clip x0 so it lies strictly within bounds (CMA-ES requirement).
    x0 = [max(lo, min(hi, v)) for v, (lo, hi) in zip(x0, bounds)]

    # Initial sigma: 10 % of average bound width.
    # Small because x0 is already a known-good calibration.
    sigma0 = 0.1 * mean([hi - lo for lo, hi in bounds])

    es = cma.CMAEvolutionStrategy(
        x0,
        sigma0,
        {
            "bounds": [list(lower_bounds), list(upper_bounds)],
            "popsize": population_size,
            "maxfevals": max_evals,
            "verbose": 3,
        },
    )

    print(f"Building simulator target: {BAZEL_TARGET}")
    _build_simulator()
    print("Build complete. Starting CMA-ES optimisation.")
    print(f"  Baseline:      {baseline_path}")
    print(f"  Scenario:      {scenario_path}")
    print(f"  Output:        {output_path}")
    print(f"  Population:    {population_size}")
    print(f"  Max evals:     {max_evals}")
    print(f"  Timeout:       {timeout_seconds}s")
    print(f"  Initial sigma: {sigma0:.4f}")

    start_time = time.time()

    # Score the baseline so we only save genuinely better results.
    best_reward = -fitness(x0, scenario_path)
    print(f"  Baseline reward: {best_reward:.4f}")

    while not es.stop():
        if time.time() - start_time > timeout_seconds:
            print("Training stopped: timeout reached.")
            break

        solutions = es.ask()

        # Run all population members in parallel — each is a separate docker exec
        # call so they don't contend with each other inside the container.
        with ProcessPoolExecutor(max_workers=population_size) as executor:
            futures = {
                executor.submit(fitness, x, scenario_path): i for i, x in enumerate(solutions)
            }
            fitnesses = [None] * len(solutions)
            for future in as_completed(futures):
                idx = futures[future]
                fitnesses[idx] = future.result()

        es.tell(solutions, fitnesses)
        es.disp()

        # Save immediately whenever we find a new best.
        current_best = -min(fitnesses)
        if current_best > best_reward:
            best_reward = current_best
            best_calib = unflatten_calibration(es.result.xbest)
            with open(output_path, "w") as f:
                json.dump(best_calib, f, indent=2)
            print(f"  New best reward: {best_reward:.4f} — saved to {output_path}")

    best_calib = unflatten_calibration(es.result.xbest)
    with open(output_path, "w") as f:
        json.dump(best_calib, f, indent=2)
    print(f"\nTraining complete. Best calibration saved to {output_path}")


if __name__ == "__main__":
    train()
