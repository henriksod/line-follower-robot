# Copyright (c) 2023 Henrik Söderlund

"""
A module that builds the arduino-cli tool
"""

load("@rules_pkg//pkg:mappings.bzl", "pkg_attributes", "pkg_files")
load("@rules_pkg//pkg:pkg.bzl", "pkg_zip")

pkg_files(
    name = "library",
    srcs = glob(["**"]),
    attributes = pkg_attributes(mode = "0755"),
    prefix = "/qtr-sensors-arduino/",
)

pkg_zip(
    name = "qtr_sensors_arduino",
    srcs = [":library"],
    visibility = ["//visibility:public"],
)
