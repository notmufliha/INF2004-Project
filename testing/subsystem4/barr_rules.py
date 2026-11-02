"""Additional token-based BARR-C checks; semantic review is still necessary."""
import re

LEXER = re.compile(
    r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|'
    r'\b(?:0[xX][0-9a-fA-F]+|\d+(?:\.\d+)?)[uUlLfF]*\b|'
    r'[A-Za-z_]\w*|==|!=|<=|>=|&&|\|\||->|<<|>>|\+\+|--|'
    r'\+=|-=|\*=|/=|%=|&=|\|=|\^=|[^\s]', re.S)
FUNCTION = re.compile(
    r'(?m)^((?:static\s+)?(?:const\s+)?\w+(?:\s*\*)?)\n'
    r'(\w+)\s+\([^;]*?\)\n\{')
TYPES = (r'(?:u?int\d+_t|bool|float32_t|char|size_t|void|ID|ER|SZ|INT|'
         r'UW|UB|UINT|SYSTIM|[a-z_]\w*_t)')


def tokenize(source):
    # Keep source offsets, but don't treat preprocessor conditionals as C ifs.
    source = re.sub(r'(?m)^\s*#[^\n]*',
                    lambda m: ' ' * len(m[0]), source)
    return [m for m in LEXER.finditer(source)
            if not m[0].startswith(('/*', '//'))]


def matching_pairs(tokens):
    pairs = {}
    stack = []
    for index, token in enumerate(tokens):
        if token[0] in ('(', '[', '{'):
            stack.append(index)
        elif token[0] in (')', ']', '}') and stack:
            other = stack.pop()
            pairs[index] = other
            pairs[other] = index
    return pairs


def check_more(path, source):
    errors = []

    def report(rule, detail, offset=0):
        line = source.count('\n', 0, offset) + 1
        errors.append(f'{path}:{line}: BARR-C {rule}: {detail}')

    raw = path.read_bytes()
    if any(value < 32 and value not in (10, 12) for value in raw):
        report('3.6', 'only LF and form feed control characters are permitted')
    if not source.endswith('/*** end of file ***/\n\n'):
        report('3.3.c', 'missing EOF marker followed by a blank line')
    if not re.fullmatch(r'[a-z][a-z0-9_]*', path.stem):
        report('4.1.a', 'invalid module name')
    if '@file' not in source:
        report('2.2.h', 'module needs a documentation comment')
    if path.suffix == '.h':
        if not re.search(r'#ifndef\s+(\w+)\s*\n#define\s+\1\b', source):
            report('4.2.b', 'missing matching include guard')
        if path.name != 's4_runtime.h' and '#include "s4_runtime.h"' in source:
            report('4.2.d', 'public header includes private runtime header')
    for match in re.finditer(r'^#define\s+(\w+)', source, re.M):
        if match[1] != match[1].upper():
            report('6.1.f', 'macro name must be uppercase', match.start())
    for match in re.finditer(r'/\*.*?\*/|//[^\n]*', source, re.S):
        body = match[0][2:-2] if match[0].startswith('/*') else match[0][2:]
        if any(value in body for value in ('/*', '//', '\\')):
            report('2.1.b', 'comment contains a forbidden token', match.start())

    code = re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"', '', source, flags=re.S)
    for match in re.finditer(r'\b(?:auto|register|goto|continue)\b|'
                            r'\b(?:abort|exit|setjmp|longjmp)\s*\(', code):
        report('1.7/8.5', 'forbidden or discouraged jump/keyword: ' + match[0])
    for match in re.finditer(r'\b'+TYPES+r'\s+(?:\*\s*)?(\w+)\s*(?=[;,=)\[])', code):
        name = match[1]
        if not re.fullmatch(r'[a-z][a-z0-9_]{2,30}', name):
            report('7.1', f'invalid variable name: {name}')
    if path.suffix == '.h':
        for match in re.finditer(r'\}\s*(\w+)\s*;', code):
            name = match[1]
            if not (name == path.stem+'_t' or name.startswith(path.stem+'_')):
                report('5.1.c', f'public type needs module prefix: {name}')

    saw_private = False
    for match in FUNCTION.finditer(source):
        private = match[1].startswith('static ')
        name = match[2]
        if '@brief' not in source[max(0, match.start()-240):match.start()]:
            report('2.2.h', f'function needs a documentation comment: {name}')
        if private:
            saw_private = True
        else:
            if saw_private:
                report('4.3.b', 'public function follows private definition', match.start())
            if name != 'main' and not name.startswith(path.stem+'_'):
                report('6.1.i', f'public function needs module prefix: {name}', match.start())
            header = path.with_suffix('.h').read_text(encoding='utf8')
            if not re.search(r'\b'+name+r'\s+\(', header):
                report('6.2.d', f'public function lacks own-header prototype: {name}')
        if not re.fullmatch(r'[a-z][a-z0-9_]{0,30}', name):
            report('6.1', f'invalid procedure name: {name}', match.start())

    tokens = tokenize(source)
    pairs = matching_pairs(tokens)
    values = [token[0] for token in tokens]
    roles = {}
    for index, value in enumerate(values):
        if value in ('&&', '||') and 0 < index < len(values)-1:
            left = values[index-1]
            right = values[index+1]
            left_complex = left == ']' or (index > 1 and values[index-2] in ('->', '.'))
            if left == ')' and index-1 in pairs:
                opening = pairs[index-1]
                left_complex = opening > 0 and bool(re.fullmatch(r'[a-zA-Z_]\w*', values[opening-1]))
                if opening > 0 and values[opening-1] in ('if', 'while', 'switch', 'for', 'return'):
                    left_complex = False
            right_complex = right in ('!', '~', '*', '&') or (
                right != '(' and index+2 < len(values) and
                values[index+2] in ('->', '.', '(', '['))
            if left_complex or right_complex:
                report('1.4.b', 'parenthesize each complex logical operand', tokens[index].start())
        if value in ('if', 'for', 'while', 'switch') and index+1 < len(values):
            opening = index+1
            if values[opening] != '(' or opening not in pairs:
                continue
            closing = pairs[opening]
            body = closing+1
            do_tail = value == 'while' and index > 0 and values[index-1] == '}'
            if body >= len(values) or (values[body] != '{' and not do_tail):
                report('1.3.a', f'{value} needs a braced body', tokens[index].start())
            elif not do_tail:
                roles[body] = value
            condition = values[opening+1:closing]
            if value == 'if' and any(t in ('=', '++', '--') for t in condition):
                report('8.2.c', 'side effect in conditional test', tokens[index].start())
            if value == 'while' and '=' in condition:
                report('8.4.b', 'assignment in loop condition', tokens[index].start())
            if value == 'switch' and body in pairs:
                if 'default' not in values[body:pairs[body]]:
                    report('8.3.b', 'switch needs a default', tokens[index].start())
        if value == 'else' and index+1 < len(values) and values[index+1] == '{':
            roles[index+1] = 'if'
        if value == 'else' and index+2 < len(values) and values[index+1] == 'if':
            condition_end = pairs.get(index+2)
            body_end = pairs.get(condition_end+1) if condition_end is not None else None
            if body_end is not None and values[body_end+1:body_end+2] != ['else']:
                report('8.2.d', 'else-if chain needs a final else', tokens[index].start())
        if value == '==' and index+1 < len(values):
            right = values[index+1]
            # Cast/sizeof operands are reviewed separately; catch simple constants.
            if re.fullmatch(r'\d\w*|[A-Z][A-Z0-9_]*|true|false', right):
                left = values[index-1] if index else ''
                if not re.fullmatch(r'\d\w*|[A-Z][A-Z0-9_]*|true|false', left):
                    report('8.6.a', 'place constant on the left of ==', tokens[index].start())
    stack = []
    for index, value in enumerate(values):
        if value == '{':
            stack.append(roles.get(index, 'block'))
            if stack.count('if') > 2:
                report('8.2.b', 'conditional nesting exceeds two levels', tokens[index].start())
        elif value == '}' and stack:
            stack.pop()
    if re.search(r'==\s*\(\w+\)\s*sizeof\b', code):
        report('8.6.a', 'place cast sizeof constant on the left of ==')
    return errors
