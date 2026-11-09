#include "b3_code39.h"

static void reset_candidate(B3_CODE39_DECODER *decoder)
{
    decoder->run_count = 0U;
    decoder->in_barcode = 0U;
    decoder->payload_valid = 0U;
    decoder->payload = '\0';
}

void b3_code39_reset(B3_CODE39_DECODER *decoder)
{
    if (decoder != NULL) {
        *decoder = (B3_CODE39_DECODER){0};
    }
}

static char decode_character(const B3_CODE39_DECODER *decoder,
                             uint8_t offset, uint32_t *narrow_max_us)
{
    /* Code 39's nine-element wide masks, with the first element in bit 8. */
    uint32_t sorted[9];
    uint16_t wide_mask = 0U;
    uint8_t wide_count = 0U;
    uint8_t index;

    for (index = 0U; index < 9U; ++index) {
        uint8_t expected_level = (index & 1U) == 0U ?
            decoder->levels[offset] :
            (uint8_t)(decoder->levels[offset] ^ 1U);
        uint32_t width = decoder->widths_us[offset + index];
        uint8_t insert = index;

        if (width == 0U || decoder->levels[offset + index] != expected_level) {
            return '\0';
        }
        sorted[index] = width;
        while (insert > 0U && sorted[insert] < sorted[insert - 1U]) {
            uint32_t value = sorted[insert];
            sorted[insert] = sorted[insert - 1U];
            sorted[insert - 1U] = value;
            insert--;
        }
    }

    *narrow_max_us = sorted[5];
    if ((uint64_t)sorted[6] * 2U < (uint64_t)sorted[5] * 3U) {
        return '\0';
    }

    for (index = 0U; index < 9U; ++index) {
        if (decoder->widths_us[offset + index] >= sorted[6]) {
            wide_mask |= (uint16_t)(1U << (8U - index));
            wide_count++;
        }
    }
    if (wide_count != 3U) {
        return '\0';
    }

    if (wide_mask == 0x094U) return '*';
    if (wide_mask == 0x109U) return 'A';
    if (wide_mask == 0x0D0U) return 'Z';
    return '?';
}

static bool valid_intercharacter_gap(const B3_CODE39_DECODER *decoder,
				     uint8_t gap_index, uint8_t next_index,
				     uint32_t narrow_max_us)
{
	return decoder->levels[gap_index] !=
			decoder->levels[gap_index - 1U] &&
		decoder->levels[next_index] == decoder->levels[gap_index - 1U] &&
		decoder->widths_us[gap_index] != 0U &&
		(uint64_t)decoder->widths_us[gap_index] * 2U >= narrow_max_us &&
		(uint64_t)decoder->widths_us[gap_index] <=
			(uint64_t)narrow_max_us * 2U;
}

static void append_run(B3_CODE39_DECODER *decoder, uint8_t level,
                       uint32_t width_us, char *symbol, bool *decoded)
{
    uint32_t narrow_max_us;
    char character;
    uint8_t index;

    *decoded = false;
    if (width_us == 0U) {
        reset_candidate(decoder);
        return;
    }

    if (decoder->in_barcode == 0U) {
        if (decoder->run_count == 9U) {
            for (index = 1U; index < 9U; ++index) {
                decoder->widths_us[index - 1U] = decoder->widths_us[index];
                decoder->levels[index - 1U] = decoder->levels[index];
            }
            decoder->run_count = 8U;
        }
        index = decoder->run_count;
        decoder->widths_us[index] = width_us;
        decoder->levels[index] = level;
        decoder->run_count++;
        if (decoder->run_count == 9U &&
            decode_character(decoder, 0U, &narrow_max_us) == '*') {
            decoder->in_barcode = 1U;
            decoder->payload_valid = 0U;
            decoder->payload = '\0';
        }
        return;
    }

    if (decoder->run_count >= B3_CODE39_RUN_COUNT) {
        reset_candidate(decoder);
        return;
    }
    index = decoder->run_count;
    decoder->widths_us[index] = width_us;
    decoder->levels[index] = level;
    decoder->run_count++;

    if (decoder->run_count == 19U) {
        character = decode_character(decoder, 10U, &narrow_max_us);
        if (!valid_intercharacter_gap(decoder, 9U, 10U, narrow_max_us)) {
            decoder->payload_valid = 0U;
        } else if (character == 'A' || character == 'Z') {
            decoder->payload = character;
            decoder->payload_valid = 1U;
        } else {
            decoder->payload_valid = 0U;
        }
        return;
    }

    if (decoder->run_count == B3_CODE39_RUN_COUNT) {
        character = decode_character(decoder, 20U, &narrow_max_us);
        if (valid_intercharacter_gap(decoder, 19U, 20U, narrow_max_us) &&
            character == '*' && decoder->payload_valid != 0U) {
            *symbol = decoder->payload;
            *decoded = true;
        }
        reset_candidate(decoder);
    }
}

bool b3_code39_feed(B3_CODE39_DECODER *decoder, uint8_t level,
                    uint32_t timestamp_us, char *symbol)
{
    uint32_t width_us;
    bool decoded;

    if (decoder == NULL || symbol == NULL || level > 1U) {
        return false;
    }
    if (decoder->have_level == 0U) {
        decoder->current_level = level;
        decoder->run_start_us = timestamp_us;
        decoder->have_level = 1U;
        return false;
    }
    if (level == decoder->current_level) {
        return false;
    }

    width_us = timestamp_us - decoder->run_start_us;
    append_run(decoder, decoder->current_level, width_us, symbol, &decoded);
    decoder->current_level = level;
    decoder->run_start_us = timestamp_us;
    return decoded;
}
