#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../../app_program/subsystem3/b3_code39.h"

#define CODE39_START_STOP (0x094U)
#define CODE39_A (0x109U)
#define CODE39_B (0x049U)
#define CODE39_Z (0x0D0U)

static B3_CODE39_DECODER decoder;
static uint32_t now_us;
static char decoded_symbol;
static uint32_t decoded_count;
static uint8_t inverted;

static void transition(uint8_t level, uint32_t width_us)
{
    now_us += width_us;
    if (b3_code39_feed(&decoder, level, now_us, &decoded_symbol)) {
        decoded_count++;
    }
}

static void emit_character(uint16_t pattern, uint32_t narrow_us,
                           uint32_t wide_us)
{
    for (uint8_t index = 0U; index < 9U; ++index) {
        uint32_t width = (pattern & (1U << (8U - index))) != 0U ?
            wide_us : narrow_us;
        uint8_t next_level = (index & 1U) == 0U ? 1U : 0U;
        transition((uint8_t)(next_level ^ inverted), width);
    }
}

static void emit_barcode(uint16_t payload, uint32_t scale,
                         uint32_t intercharacter_gap, uint8_t reverse_polarity)
{
    b3_code39_reset(&decoder);
    now_us = UINT32_MAX - 1000U;
    decoded_count = 0U;
    decoded_symbol = '\0';
    inverted = reverse_polarity;
    (void)b3_code39_feed(&decoder, inverted, now_us, &decoded_symbol);
    emit_character(CODE39_START_STOP, scale, scale * 2U);
    transition(inverted, intercharacter_gap);
    emit_character(payload, scale + 10U, (scale + 10U) * 2U);
    transition(inverted, intercharacter_gap + 10U);
    emit_character(CODE39_START_STOP, scale - 10U, (scale - 10U) * 2U);
    transition((uint8_t)(1U ^ inverted), scale - 10U);
}

/* Element widths of *X* in scan order: 9 + gap + 9 + gap + 9 = 29. */
static uint8_t build_elements(uint32_t *widths, uint16_t first, uint16_t middle,
                              uint16_t last, uint32_t scale, uint32_t gap)
{
    const uint16_t characters[3] = { first, middle, last };
    uint8_t count = 0U;

    for (uint8_t c = 0U; c < 3U; ++c) {
        for (uint8_t index = 0U; index < 9U; ++index) {
            widths[count++] = (characters[c] & (1U << (8U - index))) != 0U ?
                scale * 2U : scale;
        }
        if (c < 2U) widths[count++] = gap;
    }
    return count;
}

/* Feed elements as the car crossing the strip from the far end would see
 * them: last element first.  Levels still alternate, starting with a bar. */
static void emit_reversed(uint16_t payload, uint32_t scale, uint32_t gap,
                          uint8_t reverse_polarity, uint16_t stop_pattern)
{
    uint32_t widths[29];
    uint8_t count = build_elements(widths, CODE39_START_STOP, payload,
                                   stop_pattern, scale, gap);
    uint8_t level = reverse_polarity;

    b3_code39_reset(&decoder);
    now_us = 5000U;
    decoded_count = 0U;
    decoded_symbol = '\0';
    (void)b3_code39_feed(&decoder, level, now_us, &decoded_symbol);
    for (uint8_t i = count; i > 0U; --i) {
        /* Mild speed change along the strip, as when the car slows. */
        uint32_t width = widths[i - 1U] + (uint32_t)(count - i) * 2U;
        level ^= 1U;
        transition(level, width);
    }
}

int main(void)
{
    /* Barcode crossed from the other end decodes to the same symbol. */
    emit_reversed(CODE39_A, 100U, 100U, 0U, CODE39_START_STOP);
    assert(decoded_count == 1U && decoded_symbol == 'A');
    emit_reversed(CODE39_Z, 150U, 150U, 1U, CODE39_START_STOP);
    assert(decoded_count == 1U && decoded_symbol == 'Z');
    emit_reversed(CODE39_B, 100U, 100U, 0U, CODE39_START_STOP);
    assert(decoded_count == 0U);
    /* Reversed start with a forward-only stop is rejected. */
    emit_reversed(CODE39_A, 100U, 100U, 0U, CODE39_A);
    assert(decoded_count == 0U);

    emit_barcode(CODE39_A, 100U, 100U, 0U);
    assert(decoded_count == 1U && decoded_symbol == 'A');

    emit_barcode(CODE39_Z, 120U, 100U, 1U);
    assert(decoded_count == 1U && decoded_symbol == 'Z');

    emit_barcode(CODE39_B, 100U, 100U, 0U);
    assert(decoded_count == 0U);

    emit_barcode(CODE39_A, 100U, 10U, 0U);
    assert(decoded_count == 0U);

    emit_barcode(CODE39_A, 100U, 100U, 0U);
    assert(decoded_count == 1U);

    b3_code39_reset(&decoder);
    now_us = 100U;
    decoded_count = 0U;
    decoded_symbol = '\0';
    (void)b3_code39_feed(&decoder, 0U, now_us, &decoded_symbol);
    emit_character(CODE39_START_STOP, 100U, 130U);
    transition(0U, 100U);
    emit_character(CODE39_A, 100U, 130U);
    transition(0U, 100U);
    emit_character(CODE39_START_STOP, 100U, 130U);
    transition(1U, 100U);
    assert(decoded_count == 0U);

    puts("PASS: Code 39 A/Z decoding both directions, speed variation, wrap and rejection");
    return 0;
}
