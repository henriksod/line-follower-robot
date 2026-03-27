# line-follower-robot

## Requirements

This repository requires [Docker](https://www.docker.com/) and [Python 3](https://www.python.org/) to function.

```
sudo apt install docker-ce python3 python3-pip
```

Bazel is run via a [dazel](https://github.com/nadirizr/dazel) container.

Ensure arduino is installed for the compiler and upload tools:

```
sudo apt install arduino clang-format
```

## Install pre-commit hooks (important)

1. `pip install pre-commit`
2. `pre-commit install`

## Entrypoint

```
./run.py --help
```

## Running the simulator

```
./run.py simulate \
  --scenario $PWD/line_follower/scenarios/simple/trainer_scenario.json \
  --calibration $PWD/line_follower/scenarios/calibration/example_calibration.json
```

> **Note:** Use `$PWD`-prefixed absolute paths so they resolve correctly both on
> the macOS host and inside the Docker container.

### Live visualization

Add `--visualize` to open a real-time matplotlib window alongside the simulation:

```
./run.py simulate \
  --scenario $PWD/line_follower/scenarios/simple/trainer_scenario.json \
  --calibration $PWD/line_follower/scenarios/calibration/example_calibration.json \
  --visualize
```

The simulator runs inside Docker via dazel and writes events to `sim_events.json`
in the repo root (volume-mounted, so accessible on the host). The matplotlib
window opens natively on macOS and polls that file live. Close the window or
press **Ctrl+C** to stop everything.

See `./run.py simulate --help` for all options.