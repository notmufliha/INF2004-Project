#!/bin/sh
# run_timing.sh - Change Requirement 1 timing evidence, no robot needed.
#
# Builds each subsystem's hot path before and after its minimum-bit change
# with arm-none-eabi-gcc (Cortex-M0+, -O2) and counts executed instructions
# and cycles in a Cortex-M0+ emulator (m0_cycles.py, Unicorn + Capstone).
#
# Run from the repository root:
#   python -m venv .venv-timing && .venv-timing/Scripts/pip install unicorn capstone pyelftools
#   PY=.venv-timing/Scripts/python sh testing/timing/run_timing.sh
# (on Linux/WSL the venv python is .venv-timing/bin/python)
#
# "Before" trees are checked out as temporary git worktrees:
#   B1 3e34f57 (JSON telemetry)   B2 7bad6fb -> dc2cc5a
#   B4 34f8528 -> d4c6499          B3, B5: HEAD with the old form in the harness
set -eu
cd "$(dirname "$0")/../.."
root=$(pwd)
t=$root/testing/timing
PY=${PY:-python}
out=$(mktemp -d)
trap 'for w in "$out"/wt-*; do [ -d "$w" ] && git worktree remove --force "$w"; done; rm -rf "$out"' EXIT
TMP=$out; TEMP=$out; export TMP TEMP   # Windows arm-none-eabi-gcc temporaries

wt() { [ -d "$out/wt-$1" ] || git worktree add -q "$out/wt-$1" "$1"; echo "$out/wt-$1"; }
build() { sh "$t/build_elf.sh" "$@"; }
count() { "$PY" "$t/m0_cycles.py" "$@"; }

echo "Cortex-M0+ cycles (RP2040 timings, zero wait state), $(date +%Y-%m-%d)"
echo "arm-none-eabi-gcc $(arm-none-eabi-gcc -dumpversion), -O2"

b1=$(wt 3e34f57)
(cd "$b1" && arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -std=gnu11 -O2 \
	-ffreestanding -ffunction-sections -fdata-sections -w \
	-D_PICO_RP2040_ -DPICO_RP2040=1 -Iinclude -Iconfig -Ikernel/knlinc \
	-Iapp_program "$t/s1_before.c" app_program/subsystem1/comm_json.c \
	app_program/subsystem1/comm_names.c "$t/stubs.c" \
	-Wl,--undefined=s1_before_telemetry -nostartfiles -specs=nano.specs \
	-Wl,--gc-sections -Wl,-e,0 -Wl,-Ttext=0x10000000 \
	-Wl,-Tdata=0x20000000 -lgcc -o "$out/s1_before.elf")
build . "$out/s1_after.elf" s1_after_init,s1_after_telemetry "" \
	"$t/s1_after.c" "$t/stubs.c" app_program/subsystem1/comm_pack.c
echo; echo "== B1 before (3e34f57, JSON); result = bytes"
count "$out/s1_before.elf" s1_before_telemetry @result
echo "== B1 after (HEAD, binary frame); result = bytes"
count "$out/s1_after.elf" s1_after_init s1_after_telemetry @result

build "$(wt 7bad6fb)" "$out/b2_before.elf" b2_isr_4edges,b2_cmd_store_load \
	-DB2_BEFORE "$t/b2_timing.c" "$t/stubs.c"
build "$(wt dc2cc5a)" "$out/b2_after.elf" b2_isr_4edges,b2_cmd_store_load "" \
	"$t/b2_timing.c" "$t/stubs.c"
echo; echo "== B2 before (7bad6fb)"
count "$out/b2_before.elf" b2_isr_4edges b2_cmd_store_load @result
echo "== B2 after (dc2cc5a)"
count "$out/b2_after.elf" b2_isr_4edges b2_cmd_store_load @result

build . "$out/b3_before.elf" b3_write_line,b3_read_all -DB3_BEFORE \
	"$t/b3_timing.c" "$t/stubs.c"
build . "$out/b3_after.elf" b3_write_line,b3_read_all "" \
	"$t/b3_timing.c" "$t/stubs.c"
echo; echo "== B3 before (plain 32-bit fields)"
count "$out/b3_before.elf" b3_write_line b3_read_all @result
echo "== B3 after (packed B3_STATE)"
count "$out/b3_after.elf" b3_write_line b3_read_all @result

for c in 34f8528 d4c6499; do
	w=$(wt $c)
	build "$w" "$out/b4_$c.elf" b4_init,b4_20_samples "" "$t/b4_timing.c" \
		"$t/stubs.c" app_program/subsystem4/imu_processing.c \
		app_program/subsystem4/imu_analysis.c \
		app_program/subsystem4/imu_numeric.c \
		app_program/subsystem4/imu_motion.c -lm
done
echo; echo "== B4 before (34f8528); result = samples accepted"
count "$out/b4_34f8528.elf" b4_init b4_20_samples @result
echo "== B4 after (d4c6499)"
count "$out/b4_d4c6499.elf" b4_init b4_20_samples @result

build . "$out/b5.elf" b5_init,b5_before,b5_after "" "$t/b5_timing.c" \
	"$t/stubs.c" app_program/subsystem5/s5_packed.c -lm
echo; echo "== B5 before (S5Profile copy) / after (pack + unpack)"
count "$out/b5.elf" b5_init b5_before b5_after @result
