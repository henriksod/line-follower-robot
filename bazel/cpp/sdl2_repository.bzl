# Copyright (c) 2024 Henrik Söderlund

"""Repository rule that locates a Homebrew-installed SDL2 on macOS.

Usage in WORKSPACE:
    load("//bazel/cpp:sdl2_repository.bzl", "sdl2_repository")
    sdl2_repository(name = "sdl2")

On macOS the rule runs `brew --prefix sdl2` to find the installation
prefix, then creates symlinks and a BUILD file exposing a `cc_library`
named `@sdl2//:sdl2`.

On any other host (or when SDL2 is not installed) the rule emits an
empty cc_library stub so that platform-guarded `select()` expressions
in BUILD files compile without error on Linux / Docker builds.
"""

def _sdl2_repository_impl(ctx):
    # Try to locate SDL2 via Homebrew (macOS only).
    result = ctx.execute(["brew", "--prefix", "sdl2"])
    if result.return_code != 0:
        # SDL2 not found or not on macOS — emit an empty stub so that
        # `select({"@platforms//os:osx": ["@sdl2//:sdl2"], ...})` in
        # BUILD files does not break Linux / Docker builds.
        ctx.file("BUILD.bazel", 'cc_library(name = "sdl2", visibility = ["//visibility:public"])\n')
        return

    prefix = result.stdout.strip()

    # Create the directory layout expected by the generated BUILD file.
    ctx.execute(["mkdir", "-p", "include", "lib"])
    ctx.symlink(prefix + "/include/SDL2", "include/SDL2")
    ctx.symlink(prefix + "/lib/libSDL2.dylib", "lib/libSDL2.dylib")

    ctx.file("BUILD.bazel", """
cc_import(
    name = "sdl2_dylib",
    shared_library = "lib/libSDL2.dylib",
)

cc_library(
    name = "sdl2",
    hdrs = glob(["include/SDL2/*.h"]),
    includes = ["include"],
    deps = [":sdl2_dylib"],
    linkopts = [
        "-framework AudioToolbox",
        "-framework Cocoa",
        "-framework CoreAudio",
        "-framework CoreFoundation",
        "-framework CoreServices",
        "-framework CoreVideo",
        "-framework ForceFeedback",
        "-framework IOKit",
        "-framework Metal",
    ],
    visibility = ["//visibility:public"],
)
""")

sdl2_repository = repository_rule(
    implementation = _sdl2_repository_impl,
    local = True,
)
