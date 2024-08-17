workspace(name = "line_follower_robot")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "rules_foreign_cc",
    sha256 = "476303bd0f1b04cc311fc258f1708a5f6ef82d3091e53fd1977fa20383425a6a",
    strip_prefix = "rules_foreign_cc-0.10.1",
    url = "https://github.com/bazelbuild/rules_foreign_cc/releases/download/0.10.1/rules_foreign_cc-0.10.1.tar.gz",
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

# This sets up some common toolchains for building targets. For more details, please see
# https://bazelbuild.github.io/rules_foreign_cc/0.10.1/flatten.html#rules_foreign_cc_dependencies
rules_foreign_cc_dependencies()

http_archive(
    name = "com_google_googletest",
    strip_prefix = "googletest-5ab508a01f9eb089207ee87fd547d290da39d015",
    urls = ["https://github.com/google/googletest/archive/5ab508a01f9eb089207ee87fd547d290da39d015.zip"],
)

http_archive(
    name = "rules_yaml",
    sha256 = "8541337f1208f18616cd03dcc7bf032e3e0eebc1455fd49b5a3360d404fb9080",
    url = "https://github.com/calebfroese/rules_yaml/releases/download/v1.0.0/rules_yaml-v1.0.0.tar.gz",
)

load("@rules_yaml//:defs.bzl", "yaml_repositories")

# This sets up the dependencies for rules_yaml
yaml_repositories()

http_archive(
    name = "rules_pkg",
    sha256 = "d250924a2ecc5176808fc4c25d5cf5e9e79e6346d79d5ab1c493e289e722d1d0",
    urls = [
        "https://github.com/bazelbuild/rules_pkg/releases/download/0.10.1/rules_pkg-0.10.1.tar.gz",
    ],
)

load("@rules_pkg//:deps.bzl", "rules_pkg_dependencies")

rules_pkg_dependencies()

# arduino-cli
http_archive(
    name = "com_github_arduino_arduino_cli",
    build_file = "//bazel/arduino:arduino_cli.bzl",
    sha256 = "8f8284793e6594284b0322d1e0cad75cd346749a8e6924e85bf12b1a93d81a32",
    url = "https://downloads.arduino.cc/arduino-cli/nightly/arduino-cli_nightly-latest_Linux_64bit.tar.gz",
)

# Buildifier
# buildifier is written in Go and hence needs rules_go to be built.
# See https://github.com/bazelbuild/rules_go for the up to date setup instructions.
http_archive(
    name = "io_bazel_rules_go",
    sha256 = "6dc2da7ab4cf5d7bfc7c949776b1b7c733f05e56edc4bcd9022bb249d2e2a996",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_go/releases/download/v0.39.1/rules_go-v0.39.1.zip",
        "https://github.com/bazelbuild/rules_go/releases/download/v0.39.1/rules_go-v0.39.1.zip",
    ],
)

load("@io_bazel_rules_go//go:deps.bzl", "go_rules_dependencies")

go_rules_dependencies()

load("@io_bazel_rules_go//go:deps.bzl", "go_register_toolchains")

go_register_toolchains(version = "1.20.3")

http_archive(
    name = "bazel_gazelle",
    sha256 = "727f3e4edd96ea20c29e8c2ca9e8d2af724d8c7778e7923a854b2c80952bc405",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/bazel-gazelle/releases/download/v0.30.0/bazel-gazelle-v0.30.0.tar.gz",
        "https://github.com/bazelbuild/bazel-gazelle/releases/download/v0.30.0/bazel-gazelle-v0.30.0.tar.gz",
    ],
)

load("@bazel_gazelle//:deps.bzl", "gazelle_dependencies")

# If you use WORKSPACE.bazel, use the following line instead of the bare gazelle_dependencies():
# gazelle_dependencies(go_repository_default_config = "@//:WORKSPACE.bazel")
gazelle_dependencies()

http_archive(
    name = "com_google_protobuf",
    sha256 = "3bd7828aa5af4b13b99c191e8b1e884ebfa9ad371b0ce264605d347f135d2568",
    strip_prefix = "protobuf-3.19.4",
    urls = [
        "https://github.com/protocolbuffers/protobuf/archive/v3.19.4.tar.gz",
    ],
)

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")

protobuf_deps()

http_archive(
    name = "com_github_bazelbuild_buildtools",
    sha256 = "ae34c344514e08c23e90da0e2d6cb700fcd28e80c02e23e4d5715dddcb42f7b3",
    strip_prefix = "buildtools-4.2.2",
    urls = [
        "https://github.com/bazelbuild/buildtools/archive/refs/tags/4.2.2.tar.gz",
    ],
)

http_archive(
    name = "bazel_embedded",
    patch_args = ["-p1"],
    patches = [
        "//bazel/patches:bazel_embedded.patch",
    ],
    sha256 = "ee26c7dd5fcbe620eae8b0de9a34ecb9264cbe81cf8359d3f14ba99badfde70e",
    strip_prefix = "bazel-embedded-d3cbe4eff9a63d3dee63067d61096d681daca33b",
    url = "https://github.com/bazelembedded/bazel-embedded/archive/d3cbe4eff9a63d3dee63067d61096d681daca33b.tar.gz",
)

load("@bazel_embedded//:bazel_embedded_deps.bzl", "bazel_embedded_deps")

bazel_embedded_deps()

load("@bazel_embedded//platforms:execution_platforms.bzl", "register_platforms")

register_platforms()

load(
    "@bazel_embedded//toolchains/compilers/gcc_arm_none_eabi:gcc_arm_none_repository.bzl",
    "gcc_arm_none_compiler",
)

gcc_arm_none_compiler()

load("@bazel_embedded//toolchains/gcc_arm_none_eabi:gcc_arm_none_toolchain.bzl", "register_gcc_arm_none_toolchain")

register_gcc_arm_none_toolchain()

load("@bazel_embedded//tools/openocd:openocd_repository.bzl", "openocd_deps")

openocd_deps()

# Arduino libraries
http_archive(
    name = "com_github_qtr_sensors_arduino",
    build_file = "//bazel/arduino:qtr_sensors_arduino.bzl",
    sha256 = "95468f7e5abb244fae8f3906c82334ab1828452ec8c3b8548a68f76efbe64cc7",
    url = "https://github.com/pololu/qtr-sensors-arduino/archive/refs/tags/4.0.0.tar.gz",
)
