"""Guard against false positives and missed unbraced control statements."""
import importlib.util
from pathlib import Path
import unittest
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "layout_checker", ROOT / "tools/check_application_style.py")
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


class LayoutCheckerTests(unittest.TestCase):
    def test_multiline_preprocessor_condition_is_not_a_body(self):
        code = '#if (FIRST != 0) && \\\n    (SECOND != 1)\n#error invalid\n#endif\n'
        self.assertEqual([], checker.brace_findings(code))

    def test_unbraced_controls_are_rejected(self):
        for code in ('if (ready) return;', 'for (;;) tick();',
                     'while (ready) tick();', 'if (ready) {} else tick();',
                     'do tick(); while (ready);'):
            with self.subTest(code=code):
                self.assertTrue(checker.brace_findings(code))

    def test_braced_controls_and_do_tail(self):
        code = ('do { tick(); } while (ready);\n'
                'if (ready) {} else if (other) {} else {}\n'
                'for (;;) {} while (ready) {}\n')
        self.assertEqual([], checker.brace_findings(code))

    def test_comments_and_strings_are_not_statements(self):
        self.assertEqual([], checker.brace_findings(
            '/* if (ready) tick(); */ const char *p = "while (x) y();";'))

    @unittest.skipUnless(shutil.which('node'), 'Node.js is required')
    def test_b2_directives_and_split_function_definition(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / 'fixture.c'
            command = ['node', str(ROOT / 'tools/check-subsystem2-style.cjs'),
                       directory]
            fixture.write_text('#if (A != 0) && \\\n (B != 1)\n#endif\n')
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(0, result.returncode, result.stderr)
            fixture.write_text('static void\nfixture (void)\n{\n' +
                               '    tick();\n' * 101 + '}\n')
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(0, result.returncode)
            self.assertIn('exceeds 100 lines', result.stderr)


if __name__ == "__main__":
    unittest.main()
