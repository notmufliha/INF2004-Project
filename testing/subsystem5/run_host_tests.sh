#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
cc -std=c11 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -fno-omit-frame-pointer \
    -Iapp_program/subsystem5 \
    app_program/subsystem5/s5_logic.c \
    app_program/subsystem5/s5_sim.c \
    testing/subsystem5/host_sim_main.c \
    -o testing/subsystem5/s5_host_sim
./testing/subsystem5/s5_host_sim
