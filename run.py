#!/usr/bin/env python3

import os
import sys
import json
from support import check_packages

check_packages("pyyaml")
import yaml  # noqa

with open(f"{os.getcwd()}/config/config.yaml") as f:
    os.environ["LINE_FOLLOWER_ROBOT_CONFIG"] = json.dumps(yaml.safe_load(f))

# Set environment variable to point to this file
os.environ["LINE_FOLLOWER_ROBOT_REPO_ENTRYPOINT"] = __file__

# global logger
check_packages("loguru")
from loguru import logger  # noqa

from support.commands import cli  # noqa

if __name__ == "__main__":
    logger.remove()
    log_level = "INFO"
    if "LOGURU_LEVEL" in os.environ:
        log_level = os.environ["LOGURU_LEVEL"]
    logger.add(sys.stderr, colorize=True, level=log_level)
    cli()  # Main entrypoint
