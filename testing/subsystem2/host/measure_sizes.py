"""Compare native B2 storage using the target compiler; no robot required."""
import argparse
import pathlib
import shutil
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--cc", default="arm-none-eabi-gcc")
parser.add_argument("--nm", default="arm-none-eabi-nm")
parser.add_argument("--before", default="HEAD")
args = parser.parse_args()
root = pathlib.Path(__file__).resolve().parents[3]
probe = r'''
#include "app_program/subsystem2/motion_command.c"
#include "app_program/subsystem2/pid_control.c"
#define MEASURE(name, object) UB measured_##name[sizeof(object)]
MEASURE(input, motion_command_t);
MEASURE(pending, g_pending_command);
MEASURE(snapshot, motion_status_snapshot_t);
MEASURE(history, g_results);
MEASURE(pid, pid_state_t);
MEASURE(sample, subsystem2_sample_msg_t);
MEASURE(sample_pool, subsystem2_sample_msg_t[S2_POOL_COUNT]);
'''
with tempfile.TemporaryDirectory(prefix="b2-sizes-") as directory:
    work = pathlib.Path(directory)
    for label in ("before", "after"):
        tree = work / label
        shutil.copytree(root / "app_program", tree / "app_program")
        if label == "before":
            paths = subprocess.check_output(
                ["git", "ls-tree", "-r", "--name-only", args.before,
                 "app_program/subsystem2"], cwd=root, text=True).splitlines()
            for path in paths:
                (tree / path).write_bytes(subprocess.check_output(
                    ["git", "show", args.before + ":" + path], cwd=root))
        source = tree / "probe.c"
        source.write_text(probe)
        obj = tree / "probe.o"
        subprocess.run([args.cc, "-mcpu=cortex-m0plus", "-mthumb",
                        "-std=gnu11", "-O2", "-D_PICO_RP2040_",
                        "-DAPP_TEST=0", "-DTM_CONSOLE_UART=0",
                        "-DBOARD_UART0_FREE=1", "-DCNF_SMP=0",
                        "-I" + str(root / "include"),
                        "-I" + str(root / "config"),
                        "-I" + str(root / "kernel/knlinc"),
                        "-c", str(source), "-o", str(obj)], check=True)
        symbols = subprocess.check_output([args.nm, "-S", str(obj)], text=True)
        print(label + " (bytes, Cortex-M0+ ABI):")
        for line in symbols.splitlines():
            parts = line.split()
            if len(parts) == 4 and parts[3].startswith("measured_"):
                print("  " + parts[3][9:] + "=" + str(int(parts[1], 16)))
