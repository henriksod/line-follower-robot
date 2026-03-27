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
./run.py bazel run //line_follower/deployment/simulator/ph0_line_follower:example -- \
  --scenario $PWD/line_follower/scenarios/simple/trainer_scenario.json \
  --calibration $PWD/line_follower/scenarios/calibration/example_calibration.json
```

> **Note:** Use `$PWD`-prefixed absolute paths for `--scenario` and `--calibration`. `bazel run` sets the working directory to the runfiles directory, so relative paths will not resolve correctly.