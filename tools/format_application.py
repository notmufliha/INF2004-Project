"""Format project-owned C sources; verify all non-brace C tokens are retained.

Run with Python 3 and clang-format 21 or later. Vendor/kernel code is excluded.
The token check permits only brace insertion and splitting string literals;
it is a guard against accidental edits, not a substitute for compiling tests.
"""
import argparse
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "testing/subsystem4"))
from format_b4 import align_breaks
from barr_rules import LEXER


def signature(source):
    source = re.sub(r"\\\r?\n", "", source)
    tokens = []
    for match in LEXER.finditer(source):
        token = match[0]
        if token.startswith(("/*", "//")) or token in ("{", "}"):
            continue
        if token.startswith('"') and tokens and tokens[-1].startswith('"'):
            tokens[-1] = tokens[-1][:-1] + token[1:]
        else:
            tokens.append(token)
    return tokens


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clang-format", default="clang-format")
    args = parser.parse_args()
    roots = [ROOT / "app_program", ROOT / "testing/subsystem1/host",
             ROOT / "testing/subsystem4/host"]
    paths = sorted(p for root in roots for p in root.rglob("*")
                   if p.suffix in (".c", ".h"))
    style = ROOT / "app_program/.clang-format"
    changed = 0
    for path in paths:
        before = path.read_text(encoding="utf-8")
        # clang-format preserves tabs in some legacy banner comments.
        input_source = LEXER.sub(
            lambda m: m[0].expandtabs(4)
            if m[0].startswith(("/*", "//")) else m[0], before)
        result = subprocess.run(
            [args.clang_format, f"--style=file:{style}",
             f"--assume-filename={path}"], input=input_source,
            text=True, capture_output=True, check=True)
        after = align_breaks(result.stdout)
        if not after.rstrip().endswith('/*** end of file ***/'):
            after = after.rstrip() + '\n\n/*** end of file ***/\n\n'
        if signature(before) != signature(after):
            raise RuntimeError(f"Unexpected token change in {path}")
        if path.read_bytes() != after.encode("utf-8"):
            path.write_text(after, encoding="utf-8", newline="\n")
            changed += 1
    print(f"Formatted {changed}/{len(paths)} files; non-brace tokens preserved")


if __name__ == "__main__":
    main()
