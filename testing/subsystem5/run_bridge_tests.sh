#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
cc -std=c11 -Wall -Wextra -Werror -pedantic -fsanitize=undefined \
  -fno-omit-frame-pointer testing/subsystem5/test_bridge.c \
  -o /tmp/b5_bridge_contract_test
/tmp/b5_bridge_contract_test
