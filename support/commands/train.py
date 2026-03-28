# Copyright (c) 2024 Henrik Söderlund

"""./run.py train — CMA-ES calibration optimiser for the ph0 line follower."""

import os
from pathlib import Path

import click


def _repo_root() -> Path:
    entrypoint = os.environ.get("LINE_FOLLOWER_ROBOT_REPO_ENTRYPOINT")
    if entrypoint:
        return Path(entrypoint).parent
    return Path(os.getcwd())


@click.command()
@click.option(
    "--scenario",
    default="",
    help=(
        "Absolute path to the scenario JSON file "
        "(default: line_follower/scenarios/simple/trainer_scenario.json)."
    ),
)
@click.option(
    "--baseline",
    default="",
    help=(
        "Absolute path to the baseline calibration JSON to warm-start CMA-ES from "
        "(default: line_follower/scenarios/calibration/example_calibration.json)."
    ),
)
@click.option(
    "--output",
    default="",
    help="Path to write the best calibration JSON (default: <repo>/best_calibration.json).",
)
@click.option(
    "--population-size",
    default=10,
    show_default=True,
    type=int,
    help="CMA-ES population size (number of parallel simulations per generation).",
)
@click.option(
    "--timeout",
    default=1200,
    show_default=True,
    type=int,
    help="Maximum wall-clock seconds for the training run.",
)
@click.option(
    "--max-evals",
    default=1000,
    show_default=True,
    type=int,
    help="Maximum total number of simulator evaluations.",
)
def cli(scenario, baseline, output, population_size, timeout, max_evals):
    """Optimise line-follower calibration using CMA-ES via dazel.

    Warm-starts from a baseline calibration and runs --population-size
    simulations in parallel per generation. Results are saved to --output
    whenever a new best is found, so you can Ctrl-C safely at any time.
    """
    repo = _repo_root()

    if not scenario:
        scenario = str(repo / "line_follower/scenarios/simple/trainer_scenario.json")
    if not baseline:
        baseline = str(repo / "line_follower/scenarios/calibration/example_calibration.json")
    if not output:
        output = str(repo / "best_calibration.json")

    from support.tools.trainer import train

    train(
        scenario_path=scenario,
        baseline_path=baseline,
        output_path=output,
        population_size=population_size,
        timeout_seconds=timeout,
        max_evals=max_evals,
    )
