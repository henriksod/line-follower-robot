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
    build_file = "//bazel/arduino:BUILD.arduino_cli.bzl",
    sha256 = "683cf2a6b8953e3d632e7e4512c36667839d2073349c4b6d312e4c67592359bd",
    url = "https://github.com/arduino/arduino-cli/releases/download/v1.4.1/arduino-cli_1.4.1_Linux_64bit.tar.gz",
)

# Explicitly pin rules_cc so Bazel 7 can find cc_library.bzl and cc_test.bzl.
# bazel_embedded_deps fetches an old version via git_repository which lacks these files.
http_archive(
    name = "bazel_features",
    sha256 = "ccf85bbf0613d12bf6df2c8470ecec544a6fe8ceab684e970e8ed4dde4cb24ec",
    strip_prefix = "bazel_features-1.44.0",
    url = "https://github.com/bazel-contrib/bazel_features/releases/download/v1.44.0/bazel_features-v1.44.0.tar.gz",
)

load("@bazel_features//:deps.bzl", "bazel_features_deps")

bazel_features_deps()

http_archive(
    name = "rules_cc",
    sha256 = "283fa1cdaaf172337898749cf4b9b1ef5ea269da59540954e51fba0e7b8f277a",
    strip_prefix = "rules_cc-0.2.17",
    url = "https://github.com/bazelbuild/rules_cc/releases/download/0.2.17/rules_cc-0.2.17.tar.gz",
)

load("@rules_cc//cc:extensions.bzl", "compatibility_proxy_repo")

compatibility_proxy_repo()

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
    build_file = "//bazel/arduino:BUILD.qtr_sensors_arduino.bzl",
    sha256 = "95468f7e5abb244fae8f3906c82334ab1828452ec8c3b8548a68f76efbe64cc7",
    url = "https://github.com/pololu/qtr-sensors-arduino/archive/refs/tags/4.0.0.tar.gz",
)

# C++ libraries
http_archive(
    name = "com_github_nlohmann_json",
    sha256 = "a22461d13119ac5c78f205d3df1db13403e58ce1bb1794edc9313677313f4a9d",
    url = "https://github.com/nlohmann/json/releases/download/v3.11.3/include.zip",
)

http_archive(
    name = "com_github_jarro2783_cxxopts",
    build_file = "//bazel/cpp:BUILD.cxxopts.bzl",
    sha256 = "9f43fa972532e5df6c5fd5ad0f5bac606cdec541ccaf1732463d8070bbb7f03b",
    strip_prefix = "cxxopts-3.2.0",
    url = "https://github.com/jarro2783/cxxopts/archive/refs/tags/v3.2.0.tar.gz",
)
