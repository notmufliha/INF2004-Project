#!/bin/sh
# build_elf.sh <tree> <out.elf> <entry,entry,...> <extra cflags> <sources...>
# Builds a bare Cortex-M0+ ELF (no startup code) for m0_cycles.py.
# Sources are relative to <tree>; the harness itself is given absolute.
set -eu
tree=$1; out=$2; entries=$3; extra=$4; shift 4
keep=""
for e in $(echo "$entries" | tr ',' ' '); do keep="$keep -Wl,--undefined=$e"; done
cd "$tree"
# shellcheck disable=SC2086
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -std=gnu11 -O2 -ffreestanding \
	-ffunction-sections -fdata-sections -w \
	-D_PICO_RP2040_ -DPICO_RP2040=1 -DAPP_TEST=0 -DTM_CONSOLE_UART=0 \
	-DBOARD_UART0_FREE=1 -DCNF_SMP=0 $extra \
	-Iinclude -Iconfig -Ikernel/knlinc -Iapp_program \
	"$@" -nostartfiles -specs=nano.specs -Wl,--gc-sections $keep \
	-Wl,--unresolved-symbols=ignore-all -Wl,-e,0 \
	-Wl,-Ttext=0x10000000 -Wl,-Tdata=0x20000000 -lgcc -o "$out"
