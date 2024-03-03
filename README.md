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