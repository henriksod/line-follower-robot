import os
import re
import shutil
import stat
import subprocess
import sys
import urllib.request
import click
from loguru import logger
from support import check_packages
from support.utils.exec_subprocess import exec_subprocess
from support.utils.staged_files import get_staged_files

BUILDIFIER_VERSION = "v8.5.1"
_BUILDIFIER_PLATFORM_MAP = {
    ("linux", "x86_64"): "buildifier-linux-amd64",
    ("linux", "aarch64"): "buildifier-linux-arm64",
    ("darwin", "x86_64"): "buildifier-darwin-amd64",
    ("darwin", "arm64"): "buildifier-darwin-arm64",
}


def _ensure_buildifier():
    """Download buildifier binary if not available on PATH."""
    if shutil.which("buildifier"):
        return "buildifier"

    import platform

    system = platform.system().lower()
    machine = platform.machine().lower()
    asset = _BUILDIFIER_PLATFORM_MAP.get((system, machine))
    if not asset:
        raise RuntimeError(f"No buildifier binary available for {system}/{machine}")

    dest = os.path.join(os.path.expanduser("~"), ".local", "bin", "buildifier")
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    url = f"https://github.com/bazelbuild/buildtools/releases/download/{BUILDIFIER_VERSION}/{asset}"
    logger.info(f"Downloading buildifier from {url} ...")
    urllib.request.urlretrieve(url, dest)
    os.chmod(dest, os.stat(dest).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    if system == "darwin":
        # Clear quarantine flag (set by browser downloads, not urllib, but harmless).
        subprocess.run(
            ["xattr", "-d", "com.apple.quarantine", dest], check=False, stderr=subprocess.DEVNULL
        )
        # Ad-hoc sign the binary so Gatekeeper allows execution on macOS 14+/Apple Silicon.
        subprocess.run(["codesign", "--sign", "-", "--force", dest], check=True)
    logger.success(f"buildifier installed to {dest}")
    return dest


def is_python_file(file):
    return re.search(r"\.py", file.lower())


def is_cxx_file(file):
    return re.search(r"\.(c|cc|cpp|h|hpp)", file.lower())


@click.group()
def cli():
    """Runs formatting checks and auto formatting"""
    pass


@cli.command()
@click.option(
    "--all",
    "all_files",
    is_flag=True,
    show_default=True,
    default=False,
    help="Run formatting checks on all files.",
)
def check(all_files):
    """Check for formatting issues using Flake8 and Buildifier"""
    check_packages(
        (
            "flake8",
            "flake8-blind-except",
            "flake8-docstrings",
            "flake8-comprehensions",
            "flake8-docstrings",
            "flake8-implicit-str-concat",
            "pydocstyle>=5.0.0",
            "codespell",
            "cpplint",
        )
    )
    if not all_files:
        files = get_staged_files()
        python_files = [f for f in files if is_python_file(f)]
        cxx_files = [f for f in files if is_cxx_file(f)]
        logger.info("Staged python files:\n%s" % ("\n".join(python_files)))
        cmd_flake8 = (
            f"{sys.executable} -m flake8 {' '.join(python_files)}" if python_files else "true"
        )
        logger.info("Staged cxx files:\n%s" % ("\n".join(cxx_files)))
        cmd_cpplint = (
            (
                f"{sys.executable} -m cpplint --filter=-build/c++11,-runtime/references,-whitespace/indent_namespace --linelength=100"
                f" {' '.join(cxx_files)}"
            )
            if cxx_files
            else "true"
        )
    else:
        cmd_flake8 = f"{sys.executable} -m flake8 {os.getcwd()}"
        cmd_cpplint = (
            f"find {os.path.join(os.getcwd(), 'line_follower')}"
            " -E -regex '.*\\.(cc|cpp|h|hpp)'"
            f" | xargs {sys.executable} -m cpplint"
            " --filter=-build/c++11,-runtime/references,-whitespace/indent_namespace --linelength=100"
        )

    buildifier = _ensure_buildifier()
    cmd_buildifier = f"{buildifier} --lint=warn --warnings=-constant-glob,-module-docstring,-external-path -r ."
    cmd_codespell = "codespell --count"
    exec_subprocess(
        "%s && %s && %s && %s"
        % (
            cmd_flake8,
            cmd_cpplint,
            cmd_buildifier,
            cmd_codespell,
        ),
        msg_on_error=(
            "\nRun the following command to fix most issues with formatting:\n\t"
            "./run.py format fix --all"
        ),
        msg_on_success="Formatting checks passed!",
        exit_on_failure=True,
    )


@cli.command()
@click.option(
    "--all",
    "all_files",
    is_flag=True,
    show_default=True,
    default=False,
    help="Run formatting fix on all files.",
)
def fix(all_files):
    """Fix formatting issues using Black and Buildifier"""
    check_packages("black")
    if not all_files:
        files = get_staged_files()
        python_files = [f for f in files if is_python_file(f)]
        cxx_files = [f for f in files if is_cxx_file(f)]
        logger.info("Staged python files:\n%s" % ("\n".join(python_files)))
        cmd_black = (
            (f"{sys.executable} -m black --line-length=100" f" {' '.join(python_files)}")
            if python_files
            else "true"
        )
        cmd_clangformat = (f"clang-format --files={' '.join(cxx_files)}") if cxx_files else "true"
    else:
        cmd_black = f"{sys.executable} -m black --line-length=100 {os.getcwd()}"
        cmd_clangformat = (
            f"find -E {os.getcwd()} -regex '.*\\.(cc|cpp|h|hpp)'" f" | xargs clang-format -i"
        )

    buildifier = _ensure_buildifier()
    cmd_buildifier = f"{buildifier} --lint=fix --warnings=-constant-glob,-module-docstring,-external-path -r ."
    exec_subprocess(
        "%s && %s && %s"
        % (
            cmd_black,
            cmd_buildifier,
            cmd_clangformat,
        ),
        msg_on_error="Auto formatting failed!",
        msg_on_success="Auto formatting complete!",
        exit_on_failure=True,
    )
