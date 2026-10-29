#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
cc -std=c11 -Wall -Wextra -Werror -pedantic \
   -fsanitize=address,undefined -fno-omit-frame-pointer \
   -Iapp_program/subsystem5 \
   app_program/subsystem5/s5_packed.c \
   testing/subsystem5/test_packed.c \
   -o testing/subsystem5/s5_host_packed
./testing/subsystem5/s5_host_packed
