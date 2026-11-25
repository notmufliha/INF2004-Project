#!/bin/sh
# measure_minbits.sh - minimum-bit evidence on the Cortex-M0+ target.
#
# Run from the repository root with arm-none-eabi-gcc on PATH:
#   sh testing/measure_minbits.sh
#
# 1. Sizes of the B3 and B5 minimum-bit objects (testing/minbits_probe.c).
# 2. If the firmware has been built (build_make), static RAM (.data + .bss)
#    per subsystem from its object files.
# B2 and B4 have their own object-level tools:
#   python testing/subsystem2/host/measure_sizes.py --before <commit>
#   testing/subsystem4/measure_sizes.c (see app_program/subsystem4/DATA_REPRESENTATION.md)
set -eu
cd "$(dirname "$0")/.."

out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
# The Windows build of arm-none-eabi-gcc writes its temporaries to TMP/TEMP.
TMP=$out
TEMP=$out
export TMP TEMP

arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -std=gnu11 -ffreestanding \
	-D_PICO_RP2040_ -DPICO_RP2040=1 \
	-Iinclude -Iconfig -Ikernel/knlinc -Iapp_program \
	-c testing/minbits_probe.c -o "$out/probe.o"
echo "Object sizes (bytes, Cortex-M0+ ABI):"
arm-none-eabi-nm -S -t d "$out/probe.o" |
	awk '{ sub(/^measured_/, "", $4); printf "  %-22s %4d\n", $4, $2 }'

obj=build_make/mtkernel_3/app_program
if [ -d "$obj" ]; then
	echo
	echo "Static RAM per subsystem (bytes, from build_make objects):"
	printf '  %-12s %6s %6s %6s\n' module data bss total
	for d in subsystem1 subsystem2 subsystem3 subsystem4 subsystem5; do
		files=$(find "$obj/$d" -name '*.o' | sort)
		[ -n "$files" ] || continue
		# shellcheck disable=SC2086
		arm-none-eabi-size -A $files | awk -v n="$d" '
			$1 ~ /^\.data/ { data += $2 }
			$1 ~ /^\.bss/ || $1 ~ /^COMMON/ { bss += $2 }
			END { printf "  %-12s %6d %6d %6d\n", n, data, bss, data + bss }'
	done
fi
