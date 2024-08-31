# Copyright (c) 2023 Henrik Söderlund

"""
A module that builds the arduino-cli tool
"""

filegroup(
    name = "arduino_cli",
    srcs = [
        "arduino-cli",
    ],
    visibility = ["//visibility:public"],
)
