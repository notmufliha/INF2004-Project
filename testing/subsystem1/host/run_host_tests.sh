#!/bin/sh
# Host tests for subsystem 1's binary wire format (no Pico needed).
# Run from anywhere:  sh testing/subsystem1/host/run_host_tests.sh
# Needs a native gcc and python3.  Save the output to
# testing/subsystem1/logs/ as evidence.
set -e
cd "$(dirname "$0")"
ROOT=../../..
gcc -std=gnu11 -Wall -Wextra -Wno-unused-parameter -Wno-int-to-pointer-cast \
    -Wno-pointer-to-int-cast -D_PICO_RP2040_ -DPROHIBIT_DEF_SIZE_T -include stddef.h -DCNF_SMP=1 \
    -I$ROOT/include -I$ROOT/config -I$ROOT/kernel/knlinc \
    -o test_comm_pack test_comm_pack.c $ROOT/app_program/subsystem1/comm_pack.c
python3 -I test_wire.py
