#!/bin/sh
# Host test for the Buddy 5 ultrasonic consensus (s5_measure.c).
# Built with the real kernel headers, like the S1 packer test.
# Pass -DROBOT_NO_ECHO_MEANS_CLEAR=0 as $1 to test the fail-closed setting.
set -e
cd "$(dirname "$0")/../.."
OUT=${TMPDIR:-/tmp}/s5_measure_test
gcc -std=c11 -Wall -Wextra -Werror \
    -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast \
    -D_PICO_RP2040_ -DPROHIBIT_DEF_SIZE_T -include stddef.h -DCNF_SMP=1 $1 \
    -Iinclude -Iconfig -Ikernel/knlinc \
    -Iapp_program -Iapp_program/subsystem5 \
    testing/subsystem5/measure_test.c app_program/subsystem5/s5_measure.c \
    -o "$OUT"
"$OUT"
