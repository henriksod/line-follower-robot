workspace(name = "line_follower_robot")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
  name = "com_google_googletest",
  urls = ["https://github.com/google/googletest/archive/5ab508a01f9eb089207ee87fd547d290da39d015.zip"],
  strip_prefix = "googletest-5ab508a01f9eb089207ee87fd547d290da39d015",
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
    sha256 = "8f9ee2dc10c1ae514ee599a8b42ed99fa262b757058f65ad3c384289ff70c4b8",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_pkg/releases/download/0.9.1/rules_pkg-0.9.1.tar.gz",
        "https://github.com/bazelbuild/rules_pkg/releases/download/0.9.1/rules_pkg-0.9.1.tar.gz",
    ],
)

load("@rules_pkg//:deps.bzl", "rules_pkg_dependencies")

rules_pkg_dependencies()

# Uncrustify, C/C++ code formatter
http_archive(
    name = "com_github_uncrustify_uncrustify",
    build_file = "//bazel/cpp/uncrustify:uncrustify.bzl",
    sha256 = "414bbc9f7860eb18a53074f9af14ed04638a633b2216a73f2629291300d37c1b",
    strip_prefix = "uncrustify-uncrustify-0.77.1",
    url = "https://github.com/uncrustify/uncrustify/archive/refs/tags/uncrustify-0.77.1.tar.gz",
)

pip_parse(
    name = "line_follower_pip_deps",
    python_interpreter_target = python_interpreter_target,
    requirements_lock = "//:requirements_lock.txt",
)

load(
    "@line_follower_pip_deps//:requirements.bzl",
    install_line_follower_pip_deps = "install_deps",
)

install_line_follower_pip_deps()

# arduino-cli
http_archive(
    name = "com_github_arduino_arduino_cli",
    build_file = "//bazel/arduino:arduino_cli.bzl",
    sha256 = "c6b23c4a028d27c31eeb7e1126e22b9a87e5e5c5daada8b990194dabfae6a6c5",
    strip_prefix = "arduino-cli-a7f0de42fa04243b948c14741ee06f30098475ff",
    url = "https://github.com/arduino/arduino-cli/archive/a7f0de42fa04243b948c14741ee06f30098475ff.tar.gz",
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

load("@io_bazel_rules_go//go:deps.bzl", "go_register_toolchains", "go_rules_dependencies")

go_rules_dependencies()

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
