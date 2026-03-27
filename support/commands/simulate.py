# Copyright (c) 2024 Henrik Söderlund

"""./run.py simulate — convenience wrapper around the ph0 line follower simulation."""

import os
import signal
import sys
import threading
from pathlib import Path

import click

from support import check_packages
from support.dazel.dazel import DockerInstance


def _repo_root() -> Path:
    entrypoint = os.environ.get("LINE_FOLLOWER_ROBOT_REPO_ENTRYPOINT")
    if entrypoint:
        return Path(entrypoint).parent
    return Path(os.getcwd())


@click.command()
@click.option("--scenario", required=True, help="Absolute path to the scenario JSON file.")
@click.option(
    "--calibration", default="", help="Absolute path to the calibration JSON file (optional)."
)
@click.option(
    "--verbosity",
    default="info",
    show_default=True,
    type=click.Choice(["debug", "info", "warn", "error"], case_sensitive=False),
    help="Simulator logging verbosity.",
)
@click.option(
    "--events-log", default="", help="Path for the events log (default: <repo>/sim_events.json)."
)
@click.option(
    "--visualize",
    is_flag=True,
    default=False,
    help="Open a live matplotlib window alongside the simulation.",
)
def cli(scenario, calibration, verbosity, events_log, visualize):
    """Run the ph0 line follower simulation via dazel."""

    repo = _repo_root()

    if not events_log:
        events_log = str(repo / "sim_events.json")

    # Remove stale events from a previous run.
    try:
        Path(events_log).unlink()
    except FileNotFoundError:
        pass

    bazel_args = [
        "run",
        "//line_follower/deployment/simulator/ph0_line_follower:example",
        "--",
        "--scenario",
        scenario,
        "--verbosity",
        verbosity,
        "--events_log_json",
        events_log,
    ]
    if calibration:
        bazel_args += ["--calibration", calibration]

    # ── start / reuse the dazel container ────────────────────────────────────
    di = DockerInstance.from_config()

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
            sys.exit(rc)

    if not visualize:
        sys.exit(di.send_command(bazel_args))

    # ── visualize: run simulation in background thread, plotter on main thread ─
    # macOS requires GUI (matplotlib/Tk/Cocoa) operations on the main thread.
    # Running the dazel command in a thread and keeping plt.show() on the main
    # thread is the only reliable way to get a window on macOS.

    check_packages("matplotlib")

    sim_rc = [0]
    sim_done = threading.Event()

    def _run_sim():
        sim_rc[0] = di.send_command(bazel_args)
        sim_done.set()

    sim_thread = threading.Thread(target=_run_sim, daemon=True)

    def _handle_signal(signum, frame):
        sim_done.set()  # unblock if waiting

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    sim_thread.start()

    # Import and run the plotter on the main thread.
    # sys.path already includes the repo root because run.py is there.
    from support.tools.live_plotter import run as plotter_run  # noqa: E402

    plotter_run(scenario, events_log)

    # Window closed — wait for sim to finish (it may already be done).
    sim_done.wait()
    sys.exit(sim_rc[0])
