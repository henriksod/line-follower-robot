"""
A module that builds the uncrustify formatting tool
"""

load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
)

cmake(
    name = "uncrustify_build",
    cache_entries = {
        "CMAKE_BUILD_TYPE": "Release",
    },
    lib_source = ":all_srcs",
    out_binaries = ["uncrustify"],
)

sh_library(
    name = "uncrustify_build_sh",
    data = [":uncrustify_build"],
)

genrule(
    name = "uncrustify_gen",
    srcs = [":uncrustify_build_sh"],
    cmd = """
#! /usr/bin/env bash
tool_path=$$(echo \"$(locations :uncrustify_build_sh)/uncrustify_build/bin/uncrustify\" | sed 's/ .*//')
mv $$tool_path \"$@\"
""",
    outs = ["uncrustify_tool"],
)

sh_binary(
    name = "uncrustify",
    srcs = ["uncrustify_tool"],
    visibility = ["//visibility:public"],
)
