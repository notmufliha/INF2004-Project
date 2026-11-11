#!/usr/bin/env python3
"""Run Buddy 5 behavioral and integration tests without robot hardware.

Usage: python testing/subsystem5/run_complete_tests.py
Requires a C11 host compiler (CC, cc, or gcc). Artifacts use a temporary folder.
"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CC = os.environ.get("CC") or shutil.which("cc") or shutil.which("gcc")
if not CC:
    raise SystemExit("A C11 compiler (cc/gcc) is required")


def run(command):
    result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
    if result.returncode:
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
        raise SystemExit(result.returncode)
    return result.stdout.strip()


groups = {
    "mission_safety": ["testing/subsystem5/test_mission_safety.c"],
    "measurement": ["testing/subsystem5/measure_test.c"],
    "controller": ["app_program/subsystem5/s5_logic.c",
                   "app_program/subsystem5/s5_sim.c",
                   "testing/subsystem5/host_sim_main.c"],
    "maneuver": ["app_program/subsystem5/s5_maneuver.c",
                 "testing/subsystem5/test_maneuver.c"],
    "navigation": ["app_program/subsystem5/s5_logic.c",
                   "app_program/subsystem5/s5_maneuver.c",
                   "testing/subsystem5/test_navigation.c"],
    "scan": ["app_program/subsystem5/s5_logic.c",
             "testing/subsystem5/test_scan.c"],
    "line_input": ["testing/subsystem5/test_line_input.c", "app_program/subsystem3/b3_code39.c",
                   "app_program/subsystem3/b3_line_cal.c"],
    "bridge": ["testing/subsystem5/test_bridge.c"],
    "packed": ["app_program/subsystem5/s5_packed.c",
               "testing/subsystem5/test_packed.c"],
}
with tempfile.TemporaryDirectory(prefix="b5_complete_") as folder:
    for name, sources in groups.items():
        binary = Path(folder) / (name + (".exe" if os.name == "nt" else ""))
        run([CC, *(["-mno-ms-bitfields"] if os.name == "nt" else []), "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
             "-Iapp_program/subsystem5",
             *sources, "-o", str(binary)])
        output = run([str(binary)])
        print(output.splitlines()[-1])

    # Default boot is autonomous; VC retains normal DRIVE ownership.
    config = Path(folder) / "config.c"
    config.write_text('#include "app_program/robot_config.h"\n'
                      '#ifndef EXPECT_BYPASS\n#define EXPECT_BYPASS 1\n#endif\n'
                      '_Static_assert(ROBOT_ENABLE_BYPASS == EXPECT_BYPASS, "startup profile");\n'
                      '_Static_assert(ROBOT_FORWARD_STOP_MODE == 0, "motor owner");\n')
    for flags, expected_ok in (([], True),
                              (["-DROBOT_ENABLE_BYPASS=0", "-DEXPECT_BYPASS=0"], True),
                              (["-DROBOT_MOTION_TEST=1"], False)):
        args = [CC, "-std=c11", "-fsyntax-only", "-I", str(ROOT), *flags, str(config)]
        check = subprocess.run(args, cwd=ROOT, capture_output=True, text=True)
        assert (check.returncode == 0) == expected_ok, check.stderr
    print("PASS autonomous default, bypass config override, exclusive motor ownership")

for script in ("test_b1_contract.py", "test_b5_error_contract.py",
               "test_wire_decode.py", "test_guarded_drive.py",
               "test_guard_heartbeat.py", "test_ultrasonic_driver.py"):
    output = run([sys.executable, str(ROOT / "testing/subsystem5" / script)])
    print(output.splitlines()[-1])
print("PASS all 16 Buddy 5 test groups")
