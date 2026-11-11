#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
mkdir -p .codex_tmp
OUT=$(mktemp -d "$PWD/.codex_tmp/b5-measure.XXXXXX")
trap 'rm -f "$OUT/measure" "$OUT/measure.exe"; rmdir "$OUT"' EXIT
gcc -std=c11 -Wall -Wextra -Werror ${1:-} testing/subsystem5/measure_test.c -o "$OUT/measure"
"$OUT/measure"
