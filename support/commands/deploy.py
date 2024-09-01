import os
import re
import click
from loguru import logger
from support.utils.exec_subprocess import exec_subprocess


def get_elf_path(log):
    match = re.search(r"[\w|\.|\/]+\.ino.elf", log)
    if match:
        return match.group(0)
    return ""


@click.command()
def cli():
    """Deploys the project into a hex file compiled for teensy4.1

    Output is stored under build/output/.
    """
    run_entrypoint = os.environ["LINE_FOLLOWER_ROBOT_REPO_ENTRYPOINT"]
    deploy_target = "//line_follower/deployment/arduino/ph0_line_follower:line_follower_delivery"
    target_bin_path = (
        "bazel-bin/line_follower/deployment/arduino/ph0_line_follower/line_follower_delivery.tar"
    )
    build_path = os.path.join(os.path.dirname(run_entrypoint), "build")
    build_libs_path = os.path.join(build_path, "libs")
    build_output_path = os.path.join(build_path, "output")
    arduino_lib_path = os.path.join(
        build_path, "line_follower/deployment/arduino/ph0_line_follower"
    )
    arduino_sketch_path = os.path.join(arduino_lib_path, "ph0_line_follower.ino")
    arduino_lib_zip_path = os.path.join(arduino_lib_path, "line_follower_library_package.zip")
    unpacked_libs_path = os.path.join(
        build_path, "line_follower_library_package/src/imxrt1062/fpv5-d16-hard"
    )
    teensy_libs_path = os.path.join(
        os.path.expanduser("~"),
        ".arduino15/packages/teensy/tools/teensy-compile/11.3.1/arm/arm-none-eabi/lib",
    )
    teensy_core_path = os.path.join(
        os.path.expanduser("~"), ".arduino15/packages/teensy/hardware/avr/1.59.0"
    )
    qtr_sensors_arduino_lib_zip_path = os.path.join(
        build_path, "external/com_github_qtr_sensors_arduino/qtr_sensors_arduino.zip"
    )
    arduino_cli_config_default_path = os.path.join(
        os.path.expanduser("~"), ".arduino15/arduino-cli.yaml"
    )

    if not os.path.exists(build_path):
        os.makedirs(build_path)

    if not os.path.exists(build_libs_path):
        os.makedirs(build_libs_path)

    if not os.path.exists(build_output_path):
        os.makedirs(build_output_path)

    logger.info("Building the bazel target using hardware configuration...")
    exec_subprocess(
        f"{run_entrypoint} bazel build {deploy_target} --config=hardware",
        msg_on_error="Could not build the target to deploy.",
        exit_on_failure=True,
    )

    logger.info("Unpacking the built tar archive...")
    exec_subprocess(
        f"tar -xvf {target_bin_path} -C {build_path}",
        msg_on_error="Could not unpack the target to deploy.",
        exit_on_failure=True,
    )

    logger.info("Installing teensy4.1 platform...")
    exec_subprocess(
        "ARDUINO_BOARD_MANAGER_ADDITIONAL_URLS=https://www.pjrc.com/teensy/package_teensy_index.json"
        f" {build_path}/external/com_github_arduino_arduino_cli/arduino-cli core install teensy:avr",
        msg_on_error="Could not install teensy4.1 platform tools.",
        exit_on_failure=True,
    )

    logger.info("Making changes to arduino config to allow unsafe library installations...")
    exec_subprocess(
        f"{build_path}/external/com_github_arduino_arduino_cli/arduino-cli config init --overwrite",
        msg_on_error="Could not create arduino config file.",
        exit_on_failure=True,
    )

    exec_subprocess(
        'echo "library:\n  enable_unsafe_install: true\n"' f" >> {arduino_cli_config_default_path}",
        msg_on_error="Could not modify arduino config file.",
        exit_on_failure=True,
    )

    logger.info("Moving precompiled libraries into teensy platform...")
    exec_subprocess(
        f"unzip -o {arduino_lib_zip_path} -d {build_path}",
        msg_on_error="Could not unpack the target to deploy.",
        exit_on_failure=True,
    )

    exec_subprocess(
        f"cp {unpacked_libs_path}/*.a {teensy_libs_path}/",
        msg_on_error="Could not copy .a files to teensy lib directory.",
        exit_on_failure=True,
    )

    logger.info("Patching teensy platform...")
    exec_subprocess(
        f"cp {arduino_lib_path}/boards.txt {teensy_core_path}/",
        msg_on_error="Could not copy boards.txt file to teensy core directory.",
        exit_on_failure=True,
    )

    exec_subprocess(
        f"cp {arduino_lib_path}/platform.txt {teensy_core_path}/",
        msg_on_error="Could not copy platform.txt file to teensy core directory.",
        exit_on_failure=True,
    )

    logger.info("Installing qtr-sensors-arduino library...")
    exec_subprocess(
        f"{build_path}/external/com_github_arduino_arduino_cli/arduino-cli"
        f" lib install --zip-path --config-file {arduino_cli_config_default_path}"
        f" {qtr_sensors_arduino_lib_zip_path}",
        msg_on_error="Could not install qtr-sensors-arduino library.",
        exit_on_failure=True,
    )

    logger.info("Installing line follower library...")
    exec_subprocess(
        f"{build_path}/external/com_github_arduino_arduino_cli/arduino-cli"
        f" lib install --zip-path --config-file {arduino_cli_config_default_path}"
        f" {arduino_lib_zip_path}",
        msg_on_error="Could not install line follower library.",
        exit_on_failure=True,
    )

    logger.info("Compiling sketch...")
    result = exec_subprocess(
        f"{build_path}/external/com_github_arduino_arduino_cli/arduino-cli"
        f" compile {arduino_sketch_path} -v -b teensy:avr:teensy41",
        msg_on_error="Could not compile sketch.",
        exit_on_failure=True,
    )

    elf_path = get_elf_path(result)
    if not elf_path:
        raise Exception("Could not get path to compiled .elf file.")

    exec_subprocess(
        f"cp {os.path.dirname(elf_path)}/*.hex {build_output_path}/",
        msg_on_error="Could not copy compiled files to output directory.",
        exit_on_failure=True,
    )

    exec_subprocess(
        f"find {build_output_path} -name *.hex",
        msg_on_error="Could not deploy compiled binary.",
        msg_on_success="Successfully deployed compiled binary!",
        exit_on_failure=True,
    )
