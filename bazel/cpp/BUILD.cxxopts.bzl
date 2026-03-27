# Copyright (c) 2023 Henrik Söderlund

"""
A module that builds the cxxopts library
"""

load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "cxxopts",
    hdrs = glob(["include/cxxopts.hpp"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)
