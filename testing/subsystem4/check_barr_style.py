"""Check selected BARR-C rules for B4; not a full compliance analyser."""
from pathlib import Path
import re
import sys
from barr_rules import check_more

PROJECT = Path(__file__).resolve().parents[2]
ROOTS = (PROJECT / "app_program/subsystem4",
         PROJECT / "testing/subsystem4/host")


def main():
    errors = []
    modules = {}
    for root in ROOTS:
        for path in sorted(root.iterdir()):
            if path.suffix not in (".c", ".h"):
                continue
            source = path.read_text(encoding="utf-8")
            errors.extend(check_more(path, source))
            prefix = path.stem[:8]
            if prefix in modules and modules[prefix] != path.stem:
                errors.append(f"{path}: module first eight characters collide with {modules[prefix]}")
            modules[prefix] = path.stem
            for number, line in enumerate(source.splitlines(), 1):
                location = f"{path.relative_to(PROJECT)}:{number}"
                if len(line) > 80:
                    errors.append(f"{location}: line exceeds 80 characters")
                if "\t" in line:
                    errors.append(f"{location}: tab instead of spaces")
                if line.rstrip() != line:
                    errors.append(f"{location}: trailing whitespace")
            code = re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"',
                          "", source, flags=re.S)
            if path.name != "imu_numeric.h" and re.search(r"\bfloat\b", code):
                errors.append(f"{path}: use checked float32_t")
            if re.search(r"\b(?:short|long)\b|\bunsigned\s+int\b", code):
                errors.append(f"{path}: use fixed-width integer types")
            if re.search(r"\b(?:if|while|switch|for)\s*\([^\n]*\)[ \t]*\{", code):
                errors.append(f"{path}: control brace must start its own line")
            if re.search(r"(?:==|!=)\s*-?\d+\.\d+[fF]", code):
                errors.append(f"{path}: direct float equality comparison")
            for match in re.finditer(r"\bbool\s+(\w+)\s*(?=[;=,)])", code):
                if not match[1].startswith(("b_", "gb_")):
                    errors.append(f"{path}: Boolean name {match[1]} needs b prefix")
            for match in re.finditer(
                    r"\b(?:void|char|uint8_t|uint32_t|float32_t|bool|"
                    r"imu_\w+_t|gy511_\w+_t)\s*\*\s*(\w+)", code):
                if re.match(r"\s*\(", code[match.end():]):
                    continue  # A function returning a pointer is not a variable.
                if not match[1].startswith(("p_", "pb_", "pp_", "gp_", "gpb_")):
                    errors.append(f"{path}: pointer name {match[1]} needs p prefix")
            for match in re.finditer(r"(?m)^(\w+) \([^;]*?\)\n\{", source):
                depth = 1
                for item in re.finditer(
                        r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|[{}]',
                        source[match.end():], re.S):
                    if item[0] == "{":
                        depth += 1
                    elif item[0] == "}":
                        depth -= 1
                    if depth == 0:
                        end = match.end() + item.end()
                        lines = source[match.start():end].count("\n") + 1
                        if lines > 100:
                            errors.append(f"{path}: {match[1]} exceeds 100 lines")
                        break
            if re.search(r"!\(status\s*&|if\s*\(flgptn\s*&|while\s*\(1\)", code):
                errors.append(f"{path}: numeric condition needs explicit Boolean conversion")
            if re.search(r"\bwhile\s*\(\s*(?:true|1)\s*\)", code):
                errors.append(f"{path}: rule 8.4.c requires for (;;) for infinite loops")
            if re.search(r"for\s*\([^;]*;\s*\w+\s*<\s*[1-9]\d*[uU]?\b", code):
                errors.append(f"{path}: rule 8.4.a requires a named loop limit")
            if re.search(r"\b_Static_assert\b", code):
                errors.append(f"{path}: C11 assertion in C99 application code")
            if path.suffix == ".c":
                if not path.with_suffix(".h").exists():
                    errors.append(f"{path}: missing paired module header")
                if f'#include "{path.stem}.h"' not in source:
                    errors.append(f"{path}: must include its paired header")
                if re.search(r"\bmain\s*\(", code) and "main" not in path.stem:
                    errors.append(f"{path}: main entry filename must contain main")
            case_indent = None
            for number, line in enumerate(code.splitlines(), 1):
                stripped = line.lstrip()
                indent = len(line) - len(stripped)
                if re.match(r"(?:case\b.*|default)\s*:", stripped):
                    case_indent = indent
                elif stripped == "}" and case_indent is not None and indent < case_indent:
                    case_indent = None
                if stripped == "break;" and case_indent is not None and indent != case_indent:
                    errors.append(f"{path}:{number}: break must align with its case")
    if errors:
        print("\n".join(errors))
        return 1
    print("PASS: B4 selected BARR-C style checks")
    print("See CODING_STANDARD.md for the rule-by-rule manual review and scope.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
