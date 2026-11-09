#include "../../app_program/subsystem3/code39.h"
#include "../../app_program/subsystem3/cfg_barcode.h"

#include <assert.h>
#include <stdio.h>
#include <stdint.h>

static uint8_t const g_star_a_star[BAR_ELEMENTS] = {
	20U, 50U, 20U, 20U, 50U, 20U, 50U, 20U, 20U, 20U,
	50U, 20U, 20U, 20U, 20U, 50U, 20U, 20U, 50U, 20U,
	20U, 50U, 20U, 20U, 50U, 20U, 50U, 20U, 20U
};

int main(void)
{
	uint8_t reverse[BAR_ELEMENTS];
	uint32_t char_index = 0U;
	uint32_t index;

	assert(code39_decode(g_star_a_star, BAR_ELEMENTS, &char_index) ==
	       CODE39_OK);
	assert(char_index == 10U);
	assert(code39_char_ascii(char_index) == (uint32_t)'A');

	for (index = 0U; index < BAR_ELEMENTS; index++) {
		reverse[index] = g_star_a_star[BAR_ELEMENTS - 1U - index];
	}
	assert(code39_decode(reverse, BAR_ELEMENTS, &char_index) == CODE39_OK);
	assert(char_index == 10U);
	assert(code39_decode(g_star_a_star, BAR_ELEMENTS - 1U, &char_index) ==
	       CODE39_INVALID);

	puts("Code 39: A decoded forward and reverse; incomplete capture rejected");
	return 0;
}
