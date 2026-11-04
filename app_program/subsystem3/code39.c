#include "code39.h"

typedef struct {
	char symbol;
	uint16_t pattern;
} CODE39_ENTRY;

static const CODE39_ENTRY g_code39_table[] = {
	{'0', 0x034U}, {'1', 0x121U}, {'2', 0x061U}, {'3', 0x160U},
	{'4', 0x031U}, {'5', 0x130U}, {'6', 0x070U}, {'7', 0x025U},
	{'8', 0x124U}, {'9', 0x064U}, {'A', 0x109U}, {'B', 0x049U},
	{'C', 0x148U}, {'D', 0x019U}, {'E', 0x118U}, {'F', 0x058U},
	{'G', 0x00DU}, {'H', 0x10CU}, {'I', 0x04CU}, {'J', 0x01CU},
	{'K', 0x103U}, {'L', 0x043U}, {'M', 0x142U}, {'N', 0x013U},
	{'O', 0x112U}, {'P', 0x052U}, {'Q', 0x007U}, {'R', 0x106U},
	{'S', 0x046U}, {'T', 0x016U}, {'U', 0x181U}, {'V', 0x0C1U},
	{'W', 0x1C0U}, {'X', 0x091U}, {'Y', 0x190U}, {'Z', 0x0D0U},
	{'-', 0x085U}, {'.', 0x184U}, {' ', 0x0C4U}, {'*', 0x094U},
	{'$', 0x0A8U}, {'/', 0x0A2U}, {'+', 0x08AU}, {'%', 0x02AU},
};

#define CODE39_TABLE_SIZE \
	(sizeof(g_code39_table) / sizeof(g_code39_table[0]))

static bool code39_decode_direction(const uint32_t *widths, uint32_t count,
				    char *out, uint32_t out_size,
				    bool reverse_scan);
static bool code39_decode_character(const uint32_t *widths,
				    bool reverse_scan, char *symbol);

bool code39_pattern_for(char symbol, uint16_t *pattern)
{
	uint32_t i;

	if (pattern == 0) {
		return false;
	}
	for (i = 0U; i < CODE39_TABLE_SIZE; ++i) {
		if (g_code39_table[i].symbol == symbol) {
			*pattern = g_code39_table[i].pattern;
			return true;
		}
	}
	return false;
}

bool code39_decode(const uint32_t *widths, uint32_t count, char *out,
		   uint32_t out_size, bool *reversed)
{
	if (widths == 0 || out == 0 || out_size == 0U) {
		return false;
	}
	if (code39_decode_direction(widths, count, out, out_size, false)) {
		if (reversed != 0) {
			*reversed = false;
		}
		return true;
	}
	if (code39_decode_direction(widths, count, out, out_size, true)) {
		if (reversed != 0) {
			*reversed = true;
		}
		return true;
	}
	return false;
}

static bool code39_decode_direction(const uint32_t *widths, uint32_t count,
				    char *out, uint32_t out_size,
				    bool reverse_scan)
{
	uint32_t characters;
	uint32_t payload_length;
	uint32_t i;

	if (count < CODE39_MIN_ELEMENTS || count > CODE39_MAX_ELEMENTS ||
	    (count % 10U) != 9U) {
		return false;
	}
	characters = (count + 1U) / 10U;
	if (characters < 3U || characters > (CODE39_MAX_PAYLOAD + 2U)) {
		return false;
	}
	payload_length = characters - 2U;
	if (out_size <= payload_length) {
		return false;
	}

	for (i = 0U; i < characters; ++i) {
		char symbol;
		uint32_t offset = i * 10U;
		uint32_t j;

		if (!code39_decode_character(&widths[offset],
					     reverse_scan, &symbol)) {
			return false;
		}
		if (i == 0U || i == characters - 1U) {
			if (symbol != '*') {
				return false;
			}
		} else if (symbol == '*') {
			return false;
		} else {
			uint32_t payload_index = i - 1U;

			if (reverse_scan) {
				payload_index = payload_length - i;
			}
			out[payload_index] = symbol;
		}

		if (i + 1U < characters) {
			uint32_t gap_index = offset + 9U;

			if (widths[gap_index] == 0U) {
				return false;
			}
		}
		for (j = 0U; j < 9U; ++j) {
			uint32_t width_index = offset + j;

			if (widths[width_index] == 0U) {
				return false;
			}
		}
	}
	out[payload_length] = '\0';
	return true;
}

static bool code39_decode_character(const uint32_t *widths,
				    bool reverse_scan, char *symbol)
{
	int64_t adjusted[9];
	int64_t sorted[9];
	int64_t min_bar = INT64_MAX;
	int64_t min_space = INT64_MAX;
	int64_t bias;
	int64_t threshold;
	int64_t narrow_median;
	int64_t wide_median;
	uint16_t pattern = 0U;
	uint32_t i;
	uint32_t j;

	for (i = 0U; i < 9U; ++i) {
		uint32_t index = reverse_scan ? 8U - i : i;

		if (widths[index] == 0U) {
			return false;
		}
		if ((i & 1U) == 0U) {
			if ((int64_t)widths[index] < min_bar) {
				min_bar = (int64_t)widths[index];
			}
		} else if ((int64_t)widths[index] < min_space) {
			min_space = (int64_t)widths[index];
		}
	}

	bias = (min_space - min_bar) / 2;
	for (i = 0U; i < 9U; ++i) {
		uint32_t index = reverse_scan ? 8U - i : i;
		int64_t correction = ((i & 1U) == 0U) ? bias : -bias;

		adjusted[i] = (int64_t)widths[index] + correction;
		if (adjusted[i] <= 0) {
			return false;
		}
		sorted[i] = adjusted[i];
	}
	for (i = 1U; i < 9U; ++i) {
		int64_t value = sorted[i];

		j = i;
		while (j > 0U && sorted[j - 1U] > value) {
			sorted[j] = sorted[j - 1U];
			--j;
		}
		sorted[j] = value;
	}

	narrow_median = sorted[4];
	wide_median = sorted[7];
	if (sorted[5] == 0 || sorted[6] <= sorted[5] ||
	    (wide_median * 10 < narrow_median * 15) ||
	    (wide_median * 10 > narrow_median * 35)) {
		return false;
	}
	threshold = (sorted[5] + sorted[6]) / 2;
	for (i = 0U; i < 9U; ++i) {
		if (adjusted[i] > threshold) {
			pattern = (uint16_t)((pattern << 1U) | 1U);
		} else {
			pattern = (uint16_t)(pattern << 1U);
		}
	}
	for (i = 0U; i < CODE39_TABLE_SIZE; ++i) {
		if (g_code39_table[i].pattern == pattern) {
			*symbol = g_code39_table[i].symbol;
			return true;
		}
	}
	return false;
}
