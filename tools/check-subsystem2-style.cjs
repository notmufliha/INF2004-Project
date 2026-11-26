/* Check the mechanically enforceable subset of BARR-C used by subsystem 2. */
const fs = require('fs');
const path = require('path');
const root = process.argv[2] ? path.resolve(process.argv[2]) :
    path.resolve(__dirname, '../app_program/subsystem2');
let failures = 0;
function fail(file, line, message) {
    console.error(`${file}:${line}: ${message}`);
    failures++;
}
for (const file of fs.readdirSync(root).filter(name => /\.[ch]$/.test(name))) {
    const source = fs.readFileSync(path.join(root, file), 'utf8');
    const lines = source.split('\n');
    lines.forEach((line, index) => {
        if (line.includes('\t')) fail(file, index + 1, 'tab character');
        if (line.length > 80) fail(file, index + 1, 'line exceeds 80 columns');
        if (/[ \t]+$/.test(line)) fail(file, index + 1, 'trailing whitespace');
    });
    const code = source.replace(/\/\*[\s\S]*?\*\/|\/\/[^\n]*|"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'/g,
        token => token.replace(/[^\n]/g, ' '))
        // Directives (including continued #if expressions) are not C bodies.
        .replace(/^[ \t]*#(?:[^\n]*\\\n)*[^\n]*/gm,
            token => token.replace(/[^\n]/g, ' '));
    for (const match of code.matchAll(/\b(?:long|short|BOOL)\b|\bwhile\s*\(\s*1\s*\)/g)) {
        fail(file, code.slice(0, match.index).split('\n').length,
            `disallowed token: ${match[0]}`);
    }
    for (const match of code.matchAll(/^(?:(?:static|EXPORT)\s+)?[A-Za-z_]\w*(?:\s*\*)?\s+(\w+)\s*\([^;{}]*?\)\s*\n\{/gm)) {
        let depth = 1;
        let end = match.index + match[0].length;
        for (; depth && end < code.length; end++) {
            if (code[end] === '{') depth++;
            if (code[end] === '}') depth--;
        }
        const length = code.slice(match.index, end).split('\n').length;
        if (length > 100) fail(file, code.slice(0, match.index).split('\n').length,
            `${match[1]} exceeds 100 lines (${length})`);
    }
    for (const match of code.matchAll(/\b(?:if|while|for|switch)\s*\(/g)) {
        let depth = 1;
        let end = match.index + match[0].length;
        for (; depth && end < code.length; end++) {
            if (code[end] === '(') depth++;
            if (code[end] === ')') depth--;
        }
        const rest = code.slice(end);
        if (!/^\s*\{/.test(rest)) fail(file,
            code.slice(0, match.index).split('\n').length, 'missing control block braces');
        else if (!/^\s*\n\s*\{/.test(rest)) fail(file,
            code.slice(0, match.index).split('\n').length, 'brace must start on the next line');
    }
}
if (failures) process.exitCode = 1;
else console.log('PASS: subsystem2 automated BARR-C subset checks');
