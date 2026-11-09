/**
 * @file test_barcode.c
 * @brief On-target tests for the Code 39 decoder.
 */
#include "test_barcode.h"

#include "../app.h"

#if (0 != APP_TEST)
#include "../test_report.h"
#include "cfg_barcode.h"
#include "code39.h"

#define TEST_NARROW (20U)
#define TEST_WIDE (50U)
#define TEST_A_INDEX (10U)

static uint8_t const g_star_a_star[BAR_ELEMENTS] = {
	TEST_NARROW, TEST_WIDE, TEST_NARROW, TEST_NARROW, TEST_WIDE,
	TEST_NARROW, TEST_WIDE, TEST_NARROW, TEST_NARROW, TEST_NARROW,
	TEST_WIDE, TEST_NARROW, TEST_NARROW, TEST_NARROW, TEST_NARROW,
	TEST_WIDE, TEST_NARROW, TEST_NARROW, TEST_WIDE, TEST_NARROW,
	TEST_NARROW, TEST_WIDE, TEST_NARROW, TEST_NARROW, TEST_WIDE,
	TEST_NARROW, TEST_WIDE, TEST_NARROW, TEST_NARROW
};
#endif

void test_barcode_run(void)
{
#if (0 != APP_TEST)
	uint8_t reverse[BAR_ELEMENTS];
	uint32_t char_index = 0U;
	uint32_t index;
	bool forward_ok;
	bool reverse_ok;
	uint32_t short_result;

	TEST_INIT();
	forward_ok = code39_decode(g_star_a_star, BAR_ELEMENTS, &char_index) ==
		CODE39_OK && char_index == TEST_A_INDEX;
	for (index = 0U; index < BAR_ELEMENTS; index++) {
		reverse[index] = g_star_a_star[BAR_ELEMENTS - 1U - index];
	}
	reverse_ok = code39_decode(reverse, BAR_ELEMENTS, &char_index) ==
		CODE39_OK && char_index == TEST_A_INDEX;
	short_result = code39_decode(g_star_a_star, BAR_ELEMENTS - 1U,
				     &char_index);

	TEST_CHECK("S3-U03", forward_ok, "Code 39 *A* decodes forward");
	TEST_CHECK("S3-U03", reverse_ok, "Code 39 *A* decodes in reverse");
	TEST_CHECK("S3-U05", short_result == CODE39_INVALID,
		   "incomplete Code 39 capture is rejected");
	TEST_SUMMARY("barcode");
#endif
}
