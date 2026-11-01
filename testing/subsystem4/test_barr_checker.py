"""Regression cases for rules missed by the previous B4 style checker."""
import unittest
from types import SimpleNamespace
from barr_rules import check_more


class FixtureFile:
    suffix = '.c'
    name = 'fixture.c'
    stem = 'fixture'

    def __init__(self, source):
        self.source = source

    def __str__(self):
        return self.name

    def read_bytes(self):
        return self.source.encode('utf8')

    def with_suffix(self, suffix):
        return SimpleNamespace(read_text=lambda **kwargs: 'int32_t fixture_read (void);')


def findings(body, name='fixture_read'):
    source = ('/** @file fixture.c\n * @brief Test fixture.\n */\n'
              '#include "fixture.h"\n\n'
              '/** @brief Exercise a checker regression. */\n'
              'int32_t\n' + name + ' (void)\n{\n' + body +
              '\n    return 0;\n}\n\n/*** end of file ***/\n\n')
    return check_more(FixtureFile(source), source)


class CheckerRegressionTests(unittest.TestCase):
    def test_valid_constant_first_condition(self):
        self.assertEqual([], findings(
            '    uint32_t number = 0U;\n\n'
            '    if (0U == number)\n    {\n        return 1;\n    }'))

    def test_constant_on_wrong_side(self):
        self.assertTrue(any('8.6.a' in item for item in findings(
            '    if (number == 0U)\n    {\n        return 1;\n    }')))

    def test_deep_conditionals(self):
        self.assertTrue(any('8.2.b' in item for item in findings(
            '    if (first)\n    {\n        if (second)\n        {\n'
            '            if (third)\n            {\n'
            '                return 1;\n            }\n        }\n    }')))

    def test_else_if_without_final_else(self):
        self.assertTrue(any('8.2.d' in item for item in findings(
            '    if (first)\n    {\n        return 1;\n    }\n'
            '    else if (second)\n    {\n        return 2;\n    }')))

    def test_public_function_prefix(self):
        self.assertTrue(any('6.1.i' in item for item in findings('', 'read')))

    def test_complex_logical_operand(self):
        self.assertTrue(any('1.4.b' in item for item in findings(
            '    if (p_state->b_ready && b_valid)\n'
            '    {\n        return 1;\n    }')))

    def test_unbraced_control_body(self):
        self.assertTrue(any('1.3.a' in item for item in findings(
            '    if (b_valid)\n        return 1;')))


if __name__ == '__main__':
    unittest.main()
