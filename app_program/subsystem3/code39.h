#ifndef B3_CODE39_H
#define B3_CODE39_H

#include <stdbool.h>
#include <stdint.h>

#define CODE39_MAX_PAYLOAD (8U)
#define CODE39_MAX_ELEMENTS ((10U * (CODE39_MAX_PAYLOAD + 2U)) - 1U)
#define CODE39_MIN_ELEMENTS (29U)

bool code39_pattern_for(char symbol, uint16_t *pattern);
bool code39_decode(const uint32_t *widths, uint32_t count, char *out,
		   uint32_t out_size, bool *reversed);

#endif
