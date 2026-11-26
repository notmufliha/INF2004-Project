"""Check the common BARR-C layout subset across all application C files.

This checks line widths, whitespace, EOF markers and control-body braces.
It does not certify naming, module design, concurrency or semantic rules.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "testing/subsystem4"))
from barr_rules import matching_pairs, tokenize


def brace_findings(source):
    # Erase complete directives, preserving offsets and line numbers. A macro
    # continuation is not a standalone C control statement.
    source = re.sub(r"(?m)^[ \t]*#(?:[^\n]*\\\n)*[^\n]*",
                    lambda m: re.sub(r"[^\n]", " ", m[0]), source)
    tokens = tokenize(source)
    pairs = matching_pairs(tokens)
    values = [token[0] for token in tokens]
    errors = []
    for i, token in enumerate(tokens):
        value = token[0]
        body = None
        if value in ("if", "for", "while", "switch"):
            if i + 1 not in pairs or values[i + 1] != "(":
                continue
            if value == "while" and i and values[i - 1] == "}":
                opening = pairs.get(i - 1, 0)
                if opening and values[opening - 1] == "do":
                    continue
            body = pairs[i + 1] + 1
        elif value == "do":
            body = i + 1
        elif value == "else":
            body = i + 1
            if body < len(values) and values[body] == "if":
                continue
        if body is not None and (body >= len(values) or values[body] != "{"):
            line = source.count("\n", 0, token.start()) + 1
            errors.append((line, f"{value} requires a braced body"))
    return errors


def main():
    errors = []
    paths = sorted(p for p in (ROOT / "app_program").rglob("*")
                   if p.suffix in (".c", ".h"))
    for path in paths:
        raw = path.read_bytes()
        source = raw.decode("utf-8")
        relative = path.relative_to(ROOT)
        for n, line in enumerate(source.splitlines(), 1):
            for condition, message in (
                    (len(line) > 80, "line exceeds 80 columns"),
                    ("\t" in line, "tab character"),
                    (line.rstrip() != line, "trailing whitespace")):
                if condition:
                    errors.append(f"{relative}:{n}: {message}")
        if b"\r" in raw:
            errors.append(f"{relative}: use LF line endings")
        if not source.endswith("/*** end of file ***/\n\n"):
            errors.append(f"{relative}: missing EOF marker and final blank line")
        errors.extend(f"{relative}:{n}: {message}"
                      for n, message in brace_findings(source))
    if errors:
        print("\n".join(errors))
        return 1
    print(f"PASS: common BARR-C layout subset across {len(paths)} C/header files")
    print("Naming, interfaces and semantic rules still require separate review.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
