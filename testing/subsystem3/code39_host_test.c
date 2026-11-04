#include "../../app_program/subsystem3/code39.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t encode_symbol(char symbol, uint32_t narrow, uint32_t wide,
			      uint32_t *widths, uint32_t offset)
{
	uint16_t pattern;
	uint32_t i;

	assert(code39_pattern_for(symbol, &pattern));
	for (i = 0U; i < 9U; ++i) {
		widths[offset + i] = ((pattern >> (8U - i)) & 1U) != 0U
			? wide : narrow;
	}
	return offset + 9U;
}

static uint32_t encode_payload(char payload, uint32_t *widths,
			       uint32_t narrow, uint32_t wide)
{
	uint32_t count = encode_symbol('*', narrow, wide, widths, 0U);

	widths[count++] = narrow;
	count = encode_symbol(payload, narrow, wide, widths, count);
	widths[count++] = narrow;
	return encode_symbol('*', narrow, wide, widths, count);
}

static void check_character(char expected)
{
	uint32_t widths[CODE39_MAX_ELEMENTS];
	uint32_t reverse_widths[CODE39_MAX_ELEMENTS];
	uint32_t count = encode_payload(expected, widths, 100U, 250U);
	char decoded[CODE39_MAX_PAYLOAD + 1U];
	bool reversed = true;
	uint32_t i;

	assert(count == CODE39_MIN_ELEMENTS);
	assert(code39_decode(widths, count, decoded, sizeof(decoded), &reversed));
	assert(strcmp(decoded, (char[2]){expected, '\0'}) == 0);
	assert(!reversed);
	for (i = 0U; i < count; ++i) {
		reverse_widths[i] = widths[count - 1U - i];
	}
	assert(code39_decode(reverse_widths, count, decoded, sizeof(decoded),
			     &reversed));
	assert(strcmp(decoded, (char[2]){expected, '\0'}) == 0);
	assert(reversed);
}

int main(void)
{
	uint32_t widths[CODE39_MAX_ELEMENTS];
	char decoded[CODE39_MAX_PAYLOAD + 1U];
	char letter;

	for (letter = 'A'; letter <= 'Z'; ++letter) {
		check_character(letter);
	}

	assert(encode_payload('A', widths, 100U, 250U) ==
	       CODE39_MIN_ELEMENTS);
	assert(!code39_decode(widths, CODE39_MIN_ELEMENTS - 1U, decoded,
			      sizeof(decoded), 0));
	assert(!code39_decode(widths, CODE39_MIN_ELEMENTS, decoded, 1U, 0));
	for (uint32_t i = 0U; i < CODE39_MIN_ELEMENTS; ++i) {
		widths[i] = 100U;
	}
	assert(!code39_decode(widths, CODE39_MIN_ELEMENTS, decoded,
			      sizeof(decoded), 0));

	puts("Code 39: A-Z decode forward and reverse; invalid scans rejected");
	return 0;
}
