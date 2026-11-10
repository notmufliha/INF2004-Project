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

int main(void)
{
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

    puts("PASS: Code 39 A/Z decoding, speed variation, wrap and rejection");
    return 0;
}
