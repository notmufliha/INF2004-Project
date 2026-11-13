#include "b3_code39.h"

#include <stddef.h>
#include <stdint.h>

/* Full Code 39 set: nine elements (bar, space, ... bar), 1 = wide, first
 * element in bit 8.  Every character has exactly three wide elements. */
static const struct {
    uint16_t mask;
    char character;
} code39_table[] = {
    { 0x034U, '0' }, { 0x121U, '1' }, { 0x061U, '2' }, { 0x160U, '3' },
    { 0x031U, '4' }, { 0x130U, '5' }, { 0x070U, '6' }, { 0x025U, '7' },
    { 0x124U, '8' }, { 0x064U, '9' }, { 0x109U, 'A' }, { 0x049U, 'B' },
    { 0x148U, 'C' }, { 0x019U, 'D' }, { 0x118U, 'E' }, { 0x058U, 'F' },
    { 0x00DU, 'G' }, { 0x10CU, 'H' }, { 0x04CU, 'I' }, { 0x01CU, 'J' },
    { 0x103U, 'K' }, { 0x043U, 'L' }, { 0x142U, 'M' }, { 0x013U, 'N' },
    { 0x112U, 'O' }, { 0x052U, 'P' }, { 0x007U, 'Q' }, { 0x106U, 'R' },
    { 0x046U, 'S' }, { 0x016U, 'T' }, { 0x181U, 'U' }, { 0x0C1U, 'V' },
    { 0x1C0U, 'W' }, { 0x091U, 'X' }, { 0x190U, 'Y' }, { 0x0D0U, 'Z' },
    { 0x085U, '-' }, { 0x184U, '.' }, { 0x0C4U, ' ' }, { 0x094U, '*' },
    { 0x0A8U, '$' }, { 0x0A2U, '/' }, { 0x08AU, '+' }, { 0x02AU, '%' },
};

#define CODE39_START_STOP_MASK (0x094U)

char b3_code39_lookup(uint16_t wide_mask)
{
    size_t index;

    for (index = 0U; index < sizeof(code39_table) / sizeof(code39_table[0]);
         ++index) {
        if (code39_table[index].mask == wide_mask) {
            return code39_table[index].character;
        }
    }
    return '\0';
}

/* A strip crossed from the far end delivers each character's nine
 * elements in reverse order: mirror the mask, then use the forward table. */
static uint16_t mirror_mask(uint16_t mask)
{
    uint16_t mirrored = 0U;
    uint8_t bit;

    for (bit = 0U; bit < 9U; ++bit) {
        if ((mask & (1U << bit)) != 0U) {
            mirrored |= (uint16_t)(1U << (8U - bit));
        }
    }
    return mirrored;
}

static void reject(B3_CODE39_DECODER *decoder, B3_CODE39_REJECT reason)
{
    decoder->last_reject = (uint8_t)reason;
    decoder->last_reject_char = decoder->payload;
    decoder->reject_count++;
}

static void reset_candidate(B3_CODE39_DECODER *decoder)
{
    decoder->run_count = 0U;
    decoder->in_barcode = 0U;
    decoder->payload_valid = 0U;
    decoder->payload = '\0';
    decoder->reversed = 0U;
}

void b3_code39_reset(B3_CODE39_DECODER *decoder)
{
    if (decoder != NULL) {
        *decoder = (B3_CODE39_DECODER){0};
    }
}

/* Insertion sort of a few widths, ascending. */
static void sort_widths(uint32_t *widths, uint8_t count)
{
    uint8_t index;

    for (index = 1U; index < count; ++index) {
        uint32_t value = widths[index];
        uint8_t insert = index;

        while (insert > 0U && widths[insert - 1U] > value) {
            widths[insert] = widths[insert - 1U];
            insert--;
        }
        widths[insert] = value;
    }
}

/* True when "wider" is clearly a wide element next to "narrower". */
static bool clearly_wider(uint32_t narrower, uint32_t wider)
{
    return (uint64_t)wider * 10U >=
        (uint64_t)narrower * B3_CODE39_WIDE_RATIO_X10;
}

/* Wide/narrow mask of the nine elements at offset, or 0 when the widths do
 * not split cleanly.  Bars (elements 0, 2, 4, 6, 8) and spaces (1, 3, 5, 7)
 * are split separately: the IR spot blurs every edge, so on the car each
 * bar reads thinner and each space wider than printed, and a narrow space
 * can read as long as a wide bar.  Comparing all nine widths together then
 * rejects good barcodes.  Every Code 39 character has either two wide bars
 * and one wide space, or (only $ / + %) no wide bar and three wide spaces.
 * narrow_space_us receives the widest narrow space, for the gap check. */
static uint16_t element_mask(const B3_CODE39_DECODER *decoder, uint8_t offset,
                             uint32_t *narrow_space_us)
{
    uint32_t bars[5];
    uint32_t spaces[4];
    uint32_t bar_cut;
    uint32_t space_cut;
    uint16_t wide_mask = 0U;
    uint8_t wide_count = 0U;
    uint8_t index;

    for (index = 0U; index < 9U; ++index) {
        uint8_t expected_level = (index & 1U) == 0U ?
            decoder->levels[offset] :
            (uint8_t)(decoder->levels[offset] ^ 1U);
        uint32_t width = decoder->widths_us[offset + index];

        if (width == 0U || decoder->levels[offset + index] != expected_level) {
            return 0U;
        }
        if ((index & 1U) == 0U) {
            bars[index / 2U] = width;
        } else {
            spaces[index / 2U] = width;
        }
    }
    sort_widths(bars, 5U);
    sort_widths(spaces, 4U);

    if (clearly_wider(bars[2], bars[3]) && clearly_wider(spaces[2], spaces[3])) {
        bar_cut = bars[3];       /* two wide bars, one wide space */
        space_cut = spaces[3];
        *narrow_space_us = spaces[2];
    } else if (!clearly_wider(bars[0], bars[4]) &&
               clearly_wider(spaces[0], spaces[1])) {
        bar_cut = UINT32_MAX;    /* $ / + %: no wide bar, three wide spaces */
        space_cut = spaces[1];
        *narrow_space_us = spaces[0];
    } else {
        return 0U;
    }

    for (index = 0U; index < 9U; ++index) {
        uint32_t cut = (index & 1U) == 0U ? bar_cut : space_cut;

        if (decoder->widths_us[offset + index] >= cut) {
            wide_mask |= (uint16_t)(1U << (8U - index));
            wide_count++;
        }
    }
    return wide_count == 3U ? wide_mask : 0U;
}

/* The gap between characters is a narrow space, so compare it with the
 * neighbouring character's narrow spaces (same colour, same blur). */
static bool valid_intercharacter_gap(const B3_CODE39_DECODER *decoder,
                                     uint8_t gap_index, uint8_t next_index,
                                     uint32_t narrow_space_us)
{
    return decoder->levels[gap_index] != decoder->levels[gap_index - 1U] &&
        decoder->levels[next_index] == decoder->levels[gap_index - 1U] &&
        decoder->widths_us[gap_index] != 0U &&
        (uint64_t)decoder->widths_us[gap_index] * 2U >= narrow_space_us &&
        (uint64_t)decoder->widths_us[gap_index] <=
            (uint64_t)narrow_space_us * 2U;
}

static void append_run(B3_CODE39_DECODER *decoder, uint8_t level,
                       uint32_t width_us, char *symbol, bool *decoded)
{
    uint32_t narrow_space_us = 0U;
    uint16_t mask;
    char character;
    uint8_t index;

    if (width_us == 0U) {
        reset_candidate(decoder);
        return;
    }

    if (decoder->in_barcode == 0U) {
        /* Slide a nine-element window until a start character appears,
         * read either forwards or backwards. */
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
        if (decoder->run_count == 9U) {
            mask = element_mask(decoder, 0U, &narrow_space_us);
            if (mask == CODE39_START_STOP_MASK ||
                mask == mirror_mask(CODE39_START_STOP_MASK)) {
                decoder->in_barcode = 1U;
                decoder->payload_valid = 0U;
                decoder->payload = '\0';
                decoder->reversed = mask == CODE39_START_STOP_MASK ? 0U : 1U;
            }
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
        mask = element_mask(decoder, 10U, &narrow_space_us);
        if (decoder->reversed != 0U) {
            mask = mirror_mask(mask);
        }
        character = mask != 0U ? b3_code39_lookup(mask) : '\0';
        decoder->payload = character;
        if (mask == 0U) {
            reject(decoder, B3_CODE39_REJECT_RATIO);
            reset_candidate(decoder);
        } else if (character == '\0' || character == '*') {
            reject(decoder, B3_CODE39_REJECT_PATTERN);
            reset_candidate(decoder);
        } else if (!valid_intercharacter_gap(decoder, 9U, 10U,
                                             narrow_space_us)) {
            reject(decoder, B3_CODE39_REJECT_GAP);
            reset_candidate(decoder);
        } else {
            decoder->payload_valid = 1U;
        }
        return;
    }

    if (decoder->run_count == B3_CODE39_RUN_COUNT) {
        mask = element_mask(decoder, 20U, &narrow_space_us);
        if (decoder->reversed != 0U) {
            mask = mirror_mask(mask);
        }
        if (decoder->payload_valid != 0U && mask == CODE39_START_STOP_MASK &&
            valid_intercharacter_gap(decoder, 19U, 20U, narrow_space_us)) {
            *symbol = decoder->payload;
            *decoded = true;
            decoder->last_reversed = decoder->reversed;
        } else if (decoder->payload_valid != 0U) {
            reject(decoder, B3_CODE39_REJECT_STOP);
        }
        reset_candidate(decoder);
    }
}

/* Commit the pending level change: the element before it is complete. */
static bool commit_pending(B3_CODE39_DECODER *decoder, char *symbol)
{
    bool decoded = false;

    append_run(decoder, decoder->current_level,
               decoder->pending_us - decoder->run_start_us, symbol, &decoded);
    decoder->current_level = decoder->pending_level;
    decoder->run_start_us = decoder->pending_us;
    decoder->have_pending = 0U;
    return decoded;
}

bool b3_code39_feed(B3_CODE39_DECODER *decoder, uint8_t level,
                    uint32_t timestamp_us, char *symbol)
{
    bool decoded = false;

    if (decoder == NULL || symbol == NULL || level > 1U) {
        return false;
    }
    if (decoder->have_level == 0U) {
        decoder->current_level = level;
        decoder->run_start_us = timestamp_us;
        decoder->have_level = 1U;
        return false;
    }

    if (decoder->have_pending != 0U) {
        if (level == decoder->pending_level) {
            return false; /* repeated edge, same level */
        }
        if (timestamp_us - decoder->pending_us < B3_CODE39_MIN_RUN_US) {
            /* Flipped straight back: the pending change was a spike.
             * Drop it; the element in progress simply continues. */
            decoder->have_pending = 0U;
            return false;
        }
        decoded = commit_pending(decoder, symbol);
    }

    if (level == decoder->current_level) {
        return decoded;
    }
    decoder->pending_level = level;
    decoder->pending_us = timestamp_us;
    decoder->have_pending = 1U;
    return decoded;
}

bool b3_code39_poll(B3_CODE39_DECODER *decoder, uint32_t now_us,
                    char *symbol)
{
    if (decoder == NULL || symbol == NULL || decoder->have_pending == 0U ||
        now_us - decoder->pending_us < B3_CODE39_MIN_RUN_US) {
        return false;
    }
    return commit_pending(decoder, symbol);
}
