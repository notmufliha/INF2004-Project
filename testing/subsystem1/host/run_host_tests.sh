#!/bin/sh
# Host tests for subsystem 1 (no Pico needed):
#   S1-H01  binary wire format, C packer vs laptop decoder (test_wire.py)
#   S1-H02..H05  reconnect policy boundaries (testlink_main.c)
#   S1-H06  dashboard JavaScript decoder vs the same vectors (needs node)
# Run from anywhere:  sh testing/subsystem1/host/run_host_tests.sh
# Needs a native gcc and python3.  Save the output under
# testing/subsystem1/logs/ as evidence.
set -e
cd "$(dirname "$0")"
ROOT=../../..
STRICT="-std=c99 -Wall -Wextra -Wconversion -Wsign-conversion -Werror"

gcc $STRICT -o test_s1_link testlink_main.c $ROOT/app_program/subsystem1/s1_link.c
./test_s1_link

# comm_pack.c is built with the real kernel headers, like on the Pico.
# Those headers cast 32-bit addresses to pointers, which only warns on a
# 64-bit laptop; everything else stays strict.
gcc $STRICT -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast \
    -D_PICO_RP2040_ -DPROHIBIT_DEF_SIZE_T -include stddef.h -DCNF_SMP=1 \
    -I$ROOT/include -I$ROOT/config -I$ROOT/kernel/knlinc \
    -o test_comm_pack testpack_main.c $ROOT/app_program/subsystem1/comm_pack.c
python3 -I test_wire.py

if command -v node >/dev/null 2>&1; then
    ./test_comm_pack vectors > vectors.txt
#    node test_dashboard_decoder.js vectors.txt
    rm -f vectors.txt
else
    echo "SKIP S1-H06: node not installed"
fi
