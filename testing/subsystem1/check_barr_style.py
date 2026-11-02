"""Check selected BARR-C:2018 rules for Subsystem 1 (B1).

Reuses B4's token-based rule set (testing/subsystem4/barr_rules.py) so the
whole team is measured the same way, and adds the same line checks as
B4's check_barr_style.py (80 columns, no tabs, no trailing spaces, control
braces on their own line, ...).  This is not a full compliance analyser:
CODING_STANDARD.md records the manual rule-by-rule review and the agreed
deviations, which are listed in ALLOWED below so they stay visible.

Run from anywhere:  python testing/subsystem1/check_barr_style.py
"""
from pathlib import Path
import re
import sys

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT / "testing/subsystem4"))
from barr_rules import check_more  # noqa: E402  (B4's shared rule set)

SCOPE = sorted(
    list((PROJECT / "app_program/subsystem1").glob("*.[ch]")) +
    [PROJECT / "app_program/fault.c", PROJECT / "app_program/fault.h",
     PROJECT / "app_program/fault_flags.h",
     PROJECT / "lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c",
     PROJECT / "lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h"] +
    list((PROJECT / "testing/subsystem1/host").glob("*.[ch]")))

# Agreed deviations (CODING_STANDARD.md, section "Deviations").  Each entry
# is (file name, regular expression matched against the message).
ALLOWED = (
    # DEV-1: team-facing names kept so B2/B3/B4/B5/VC compile unchanged.
    ("subsystem1.h", r"5\.1\.c: public type needs module prefix"),
    ("subsystem1.h", r"7\.1: invalid variable name: [A-Z_]+$"),
    # DEV-2: fault_flags.h is the bit table; its functions are in fault.h.
    ("fault_flags.h", r"5\.1\.c"),
)

TYPES_NEEDING_P = (r"\b(?:void|char|uint8_t|uint16_t|uint32_t|bool|"
                   r"comm_\w+_t|s1_\w+_t|ip_addr_t|mqtt_client_t)"
                   r"\s*\*\s*(?:const\s+)?(?!const\b)(\w+)\b(?!\s*\[)")


def allowed(path, message):
    return any(path.name == name and re.search(pattern, message)
               for name, pattern in ALLOWED)


def line_checks(path, source):
    errors = []
    for number, line in enumerate(source.splitlines(), 1):
        where = f"{path.relative_to(PROJECT)}:{number}"
        if len(line) > 80:
            errors.append(f"{where}: BARR-C 1.2.a: line exceeds 80 characters")
        if "\t" in line:
            errors.append(f"{where}: BARR-C 3.5.a: tab instead of spaces")
        if line.rstrip() != line:
            errors.append(f"{where}: BARR-C 3.4: trailing whitespace")
    code = re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"', "", source,
                  flags=re.S)
    if re.search(r"\b(?:short|long)\b|\bunsigned\s+int\b", code):
        errors.append(f"{path}: BARR-C 5.2.a: use fixed-width integer types")
    if re.search(r"\b(?:if|while|switch|for)\s*\([^\n]*\)[ \t]*\{", code):
        errors.append(f"{path}: BARR-C 1.3.b: control brace on its own line")
    if re.search(r"\bwhile\s*\(\s*(?:true|1)\s*\)", code):
        errors.append(f"{path}: BARR-C 8.4.c: use for (;;) for endless loops")
    for match in re.finditer(r"\bbool\s+(\w+)\s*(?=[;=,)])", code):
        if not match[1].startswith(("b_", "gb_")):
            errors.append(f"{path}: BARR-C 7.1.m: Boolean {match[1]} needs b_")
    for match in re.finditer(TYPES_NEEDING_P, code):
        if re.match(r"\s*\(", code[match.end():]):
            continue  # a function returning a pointer, not a variable
        if not match[1].startswith(("p_", "pp_", "gp_", "pb_")):
            errors.append(f"{path}: BARR-C 7.1.l: pointer {match[1]} needs p_")
    if path.suffix == ".c":
        if not path.with_suffix(".h").exists():
            errors.append(f"{path}: BARR-C 4.2.a: missing paired header")
        elif f'#include "{path.stem}.h"' not in source:
            errors.append(f"{path}: BARR-C 4.3.a: must include its header")
        if re.search(r"\bmain\s*\(", code) and "main" not in path.stem:
            errors.append(f"{path}: main() only in a *main.c file")
    return errors


def main():
    errors = []
    prefixes = {}
    for path in SCOPE:
        source = path.read_text(encoding="utf-8")
        found = check_more(path, source) + line_checks(path, source)
        errors.extend(e for e in found if not allowed(path, e))
        stem8 = path.stem[:8]
        if prefixes.get(stem8, path.stem) != path.stem:
            errors.append(f"{path}: BARR-C 4.1.b: first 8 characters "
                          f"collide with {prefixes[stem8]}")
        prefixes[stem8] = path.stem
    print(f"checked {len(SCOPE)} B1 files")
    if errors:
        print("\n".join(errors))
        return 1
    print("PASS: B1 selected BARR-C style checks "
          f"({len(ALLOWED)} documented deviations allowed)")
    print("See testing/subsystem1/CODING_STANDARD.md for the manual review.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
