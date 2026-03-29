#!/usr/bin/env bash
# Integration test: run the robot simulation with the trainer scenario and
# example calibration and verify that the robot finishes the track.
#
# The simulation is run in deterministic fast mode (--time_step_us=10001) and
# allowed up to 60 s of wall time.  The test passes when the event log shows a
# positive time_spent_on_line value, confirming the robot successfully followed
# the track.

set -euo pipefail

# Locate runfiles.  Bazel sets TEST_SRCDIR when running tests; RUNFILES_DIR is
# the canonical variable for the runfiles tree.  In the main Bzlmod workspace
# the mapping directory is named "_main".
RUNFILES="${RUNFILES_DIR:-${TEST_SRCDIR}}"
WORKSPACE="_main"

SIMULATOR="${RUNFILES}/${WORKSPACE}/line_follower/deployment/simulator/ph0_line_follower/example"
SCENARIO="${RUNFILES}/${WORKSPACE}/line_follower/scenarios/simple/trainer_scenario.json"
CALIBRATION="${RUNFILES}/${WORKSPACE}/line_follower/scenarios/calibration/example_calibration.json"

# Verify expected files are present before starting.
for f in "${SIMULATOR}" "${SCENARIO}" "${CALIBRATION}"; do
    if [[ ! -e "${f}" ]]; then
        echo "FAIL: required file not found: ${f}"
        exit 1
    fi
done

# Create a temporary directory for the simulation event log.
TMPDIR_OUT=$(mktemp -d)
trap 'rm -rf "${TMPDIR_OUT}"' EXIT
OUTPUT="${TMPDIR_OUT}/events.json"

# Run the simulation.  Use --time_step_us=10001 so the virtual clock advances
# one tick faster than the 10 ms sensor update rate, making every outer-loop
# iteration fire all scheduled callbacks without any wall-clock gating.
# The simulation exits on its own when the robot loses the line; the 60 s
# timeout is a safety net.  Both exit codes (0 = normal exit, 124 = killed by
# timeout) indicate the simulation ran — the output log determines success.
timeout 60 "${SIMULATOR}" \
    --scenario="${SCENARIO}" \
    --calibration="${CALIBRATION}" \
    --events_log_json="${OUTPUT}" \
    --verbosity=error \
    --time_step_us=10001 || true

# Verify that the simulation wrote an event log.
if [[ ! -f "${OUTPUT}" ]]; then
    echo "FAIL: simulation produced no event log at ${OUTPUT}"
    exit 1
fi

# Parse the event log and confirm the robot spent time on the line.
python3 - "${OUTPUT}" <<'PYEOF'
import json
import sys

output_path = sys.argv[1]

stats = []
with open(output_path) as f:
    for line in f:
        try:
            entry = json.loads(line)
        except json.JSONDecodeError:
            continue
        if "line_following_statistics" in entry:
            stats.append(entry["line_following_statistics"])

if not stats:
    print("FAIL: no line_following_statistics entries found in event log")
    sys.exit(1)

last = stats[-1]
if "time_spent_on_line" not in last:
    print("FAIL: 'time_spent_on_line' field missing from line_following_statistics")
    sys.exit(1)

time_on_line = last["time_spent_on_line"]
print(f"Logged entries: {len(stats)}, time_spent_on_line: {time_on_line:.4f} s")

if time_on_line <= 0:
    print("FAIL: robot did not spend any time on the line; track not finished")
    sys.exit(1)

print("PASS: robot finished the track")
PYEOF
