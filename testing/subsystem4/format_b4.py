"""Format B4 and align switch breaks as required by BARR-C 8.3.a."""
from pathlib import Path
import argparse
import re
import subprocess

PROJECT = Path(__file__).resolve().parents[2]
ROOTS = (PROJECT / "app_program/subsystem4",
         PROJECT / "testing/subsystem4/host")


def align_breaks(source):
    case_indent = None
    result = []
    for line in source.splitlines():
        stripped = line.lstrip()
        indent = len(line) - len(stripped)
        if re.match(r"(?:case\b.*|default)\s*:", stripped):
            case_indent = indent
        elif stripped == "}" and case_indent is not None and indent < case_indent:
            case_indent = None
        if stripped == "break;" and case_indent is not None:
            line = " " * case_indent + stripped
        result.append(line.rstrip())
    return "\n".join(result).rstrip() + "\n\n"


def separate_blocks(source):
    """Keep natural statement/declaration blocks visibly separated (3.3.b)."""
    lines = source.splitlines()
    result = []
    declaration = re.compile(
        r'^\s+(?:(?:const|static|volatile)\s+)*'
        r'(?:u?int\d+_t|bool|float32_t|char|size_t|\w+_t|T_\w+|'
        r'UW|UB|INT|ER|SZ|SYSTIM|UINT)\s+(?:\*\s*)?\w+\s*'
        r'(?:=|;|\[[^\]]*\]\s*(?:=|;))')
    in_declaration = False
    previous_declaration = False
    for index, line in enumerate(lines):
        stripped = line.strip()
        previous = result[-1].strip() if result else ""
        starts_declaration = declaration.match(line) is not None
        if starts_declaration and previous_declaration:
            while result and not result[-1].strip():
                result.pop()
        if (starts_declaration and not previous_declaration
                and not in_declaration and previous
                and previous != '{'
                and not previous.startswith(('#', '/*', '*', '//'))):
            result.append('')
        in_declaration = in_declaration or starts_declaration
        if (re.match(r"(?:if|for|while|switch|do)\b", stripped)
                and previous and previous not in ("{", "}")
                and not previous.startswith(("#", "/*", "*", "//"))):
            result.append("")
        result.append(line)
        if in_declaration and stripped.endswith(';'):
            in_declaration = False
            previous_declaration = True
            if index + 1 < len(lines):
                following = lines[index+1]
                if (following.strip() and not declaration.match(following)
                        and not following.strip().startswith(('}', '#'))):
                    result.append('')
        elif stripped and not in_declaration:
            previous_declaration = False
        if stripped == "}" and index + 1 < len(lines):
            following = lines[index + 1].strip()
            if (following and not following.startswith(("}", "else", "while", "#"))
                    and following not in ("break;",)):
                result.append("")
    return "\n".join(result).rstrip() + "\n\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clang-format", default="clang-format")
    args = parser.parse_args()
    style = PROJECT / "app_program/subsystem4/.clang-format"
    for root in ROOTS:
        for path in sorted(root.iterdir()):
            if path.suffix not in (".c", ".h"):
                continue
            subprocess.run([args.clang_format, f"--style=file:{style}",
                            "-i", str(path)], check=True)
            path.write_text(separate_blocks(align_breaks(path.read_text(encoding="utf-8"))), encoding="utf-8",
                            newline="\n")


if __name__ == "__main__":
    main()
