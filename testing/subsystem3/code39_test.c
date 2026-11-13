/*
 * Host test for app_program/subsystem3/b3_code39.c (no Pico needed).
 * Widths are realistic: a ~3.4 mm narrow bar at ~40-80 mm/s is ~10 ms.
 *
 *   C39-01  full Code 39 table: 44 distinct masks, three wide elements each
 *   C39-02  every character decodes forwards and backwards (both polarities)
 *   C39-03  speed drift across the strip and timer wrap-around
 *   C39-04  sensor chatter at edges and a spike inside a bar are ignored
 *   C39-05  rejects: weak wide/narrow ratio, bad gap, wrong stop direction,
 *           with the reject reason recorded for the serial log
 *   C39-06  sensor blur: bars read thinner, spaces wider (narrow space as
 *           long as a wide bar), every character, both directions
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../../app_program/subsystem3/b3_code39.h"

#define STAR (0x094U)
#define NARROW_US (10000U)

static const char all_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

static B3_CODE39_DECODER decoder;
static uint32_t now_us;
static char decoded_symbol;
static uint32_t decoded_count;

static uint16_t mask_of(char c)
{
    for (uint16_t mask = 0U; mask < 512U; ++mask) {
        if (b3_code39_lookup(mask) == c) return mask;
    }
    return 0U;
}

static void feed(uint8_t level, uint32_t at)
{
    char symbol = '\0';
    if (b3_code39_feed(&decoder, level, at, &symbol)) {
        decoded_count++;
        decoded_symbol = symbol;
    }
}

static void finish(void)
{
    char symbol = '\0';
    if (b3_code39_poll(&decoder, now_us + B3_CODE39_MIN_RUN_US, &symbol)) {
        decoded_count++;
        decoded_symbol = symbol;
    }
}

/* Element widths of *X* in scan order: 9 + gap + 9 + gap + 9 = 29. */
static uint8_t build(uint32_t *widths, uint16_t payload, uint16_t stop,
                     uint32_t narrow, uint32_t wide, uint32_t gap)
{
    const uint16_t chars[3] = { STAR, payload, stop };
    uint8_t count = 0U;
    for (uint8_t c = 0U; c < 3U; ++c) {
        for (uint8_t i = 0U; i < 9U; ++i) {
            widths[count++] = (chars[c] & (1U << (8U - i))) != 0U ?
                wide : narrow;
        }
        if (c < 2U) widths[count++] = gap;
    }
    return count;
}

/* Bar/space blur applied by scan(): bars (even elements) lose it, spaces
 * (odd elements, including the gaps) gain it. */
static uint32_t blur_us;

/* Cross the strip; reversed = from the far end.  drift_us is added per
 * element to mimic the car slowing down across the barcode. */
static void scan(uint16_t payload, uint16_t stop, uint32_t narrow,
                 uint32_t wide, uint32_t gap, uint8_t polarity,
                 uint8_t reversed, uint32_t drift_us, uint32_t start_us)
{
    uint32_t widths[29];
    uint8_t count = build(widths, payload, stop, narrow, wide, gap);
    for (uint8_t n = 0U; n < count; ++n) {
        if ((n & 1U) == 0U) widths[n] -= blur_us;
        else widths[n] += blur_us;
    }
    uint8_t level = polarity; /* quiet zone level */

    b3_code39_reset(&decoder);
    decoded_count = 0U;
    decoded_symbol = '\0';
    now_us = start_us;
    feed(level, now_us);
    now_us += 50000U; /* quiet zone */
    for (uint8_t n = 0U; n < count; ++n) {
        uint8_t i = reversed != 0U ? (uint8_t)(count - 1U - n) : n;
        level ^= 1U;
        feed(level, now_us);
        now_us += widths[i] + (uint32_t)n * drift_us;
    }
    level ^= 1U;
    feed(level, now_us); /* back to the quiet zone */
    now_us += 50000U;
    finish();
}

int main(void)
{
    /* C39-01 */
    for (const char *c = all_chars; *c != '\0'; ++c) {
        uint16_t mask = mask_of(*c);
        unsigned bits = 0U;
        assert(mask != 0U);
        for (unsigned b = 0U; b < 9U; ++b) bits += (mask >> b) & 1U;
        assert(bits == 3U);
        for (const char *d = c + 1; *d != '\0'; ++d) assert(mask_of(*d) != mask);
    }
    assert(mask_of('*') == STAR);
    puts("PASS C39-01 full table");

    /* C39-02 */
    for (const char *c = all_chars; *c != '\0'; ++c) {
        for (uint8_t polarity = 0U; polarity < 2U; ++polarity) {
            for (uint8_t reversed = 0U; reversed < 2U; ++reversed) {
                scan(mask_of(*c), STAR, NARROW_US, NARROW_US * 5U / 2U,
                     NARROW_US, polarity, reversed, 0U, 1000U);
                assert(decoded_count == 1U && decoded_symbol == *c);
                assert(decoder.last_reversed == reversed);
            }
        }
    }
    puts("PASS C39-02 all characters, both directions and polarities");

    /* C39-03: slowing by 150 us per element, and timer wrap mid-scan. */
    scan(mask_of('A'), STAR, NARROW_US, NARROW_US * 2U, NARROW_US, 0U, 0U,
         150U, UINT32_MAX - 300000U);
    assert(decoded_count == 1U && decoded_symbol == 'A');
    scan(mask_of('Z'), STAR, NARROW_US, NARROW_US * 2U, NARROW_US, 1U, 1U,
         150U, 7U);
    assert(decoded_count == 1U && decoded_symbol == 'Z');
    puts("PASS C39-03 speed drift and timer wrap");

    /* C39-04: chatter.  Feed *A* by hand with bounces at every edge and a
     * 1 ms spike in the middle of the first wide bar. */
    {
        uint32_t widths[29];
        uint8_t count = build(widths, mask_of('A'), STAR, NARROW_US,
                              NARROW_US * 5U / 2U, NARROW_US);
        uint8_t level = 0U;
        b3_code39_reset(&decoder);
        decoded_count = 0U;
        now_us = 1000U;
        feed(level, now_us);
        now_us += 50000U;
        for (uint8_t i = 0U; i < count; ++i) {
            uint32_t t = now_us;
            level ^= 1U;
            feed(level, t);            /* real edge */
            feed(level ^ 1U, t + 300U); /* bounce back */
            feed(level, t + 700U);     /* settle */
            if (i == 2U) {             /* spike inside a wide bar */
                feed(level ^ 1U, t + 12000U);
                feed(level, t + 13000U);
            }
            now_us += widths[i];
        }
        level ^= 1U;
        feed(level, now_us);
        now_us += 50000U;
        finish();
        assert(decoded_count == 1U && decoded_symbol == 'A');
    }
    puts("PASS C39-04 chatter and spikes ignored");

    /* C39-05 rejects with reasons */
    scan(mask_of('A'), STAR, NARROW_US, NARROW_US * 13U / 10U, NARROW_US,
         0U, 0U, 0U, 1000U); /* wide only 1.3x narrow */
    assert(decoded_count == 0U);
    scan(mask_of('A'), STAR, NARROW_US, NARROW_US * 5U / 2U, NARROW_US * 3U,
         0U, 0U, 0U, 1000U); /* gap 3x narrow */
    assert(decoded_count == 0U && decoder.reject_count == 1U &&
           decoder.last_reject == B3_CODE39_REJECT_GAP);
    scan(mask_of('A'), mask_of('B'), NARROW_US, NARROW_US * 5U / 2U,
         NARROW_US, 0U, 0U, 0U, 1000U); /* stop is not '*' */
    assert(decoded_count == 0U &&
           decoder.last_reject == B3_CODE39_REJECT_STOP &&
           decoder.last_reject_char == 'A');
    puts("PASS C39-05 rejects with reasons");

    /* C39-06: printed narrow 10 ms / wide 25 ms, blurred by 6 ms per edge
     * pair: bars 4 / 19 ms, spaces 16 / 31 ms.  A narrow space (16) is
     * almost a wide bar (19): one shared wide/narrow split rejects it. */
    blur_us = 6000U;
    for (const char *c = all_chars; *c != '\0'; ++c) {
        for (uint8_t reversed = 0U; reversed < 2U; ++reversed) {
            scan(mask_of(*c), STAR, NARROW_US, NARROW_US * 5U / 2U,
                 NARROW_US, 0U, reversed, 0U, 1000U);
            assert(decoded_count == 1U && decoded_symbol == *c);
            assert(decoder.last_reversed == reversed);
        }
    }
    /* Opposite bias (bars wider, spaces thinner) also reads. */
    blur_us = (uint32_t)-4000;
    scan(mask_of('Z'), STAR, NARROW_US, NARROW_US * 5U / 2U, NARROW_US,
         1U, 1U, 0U, 1000U);
    assert(decoded_count == 1U && decoded_symbol == 'Z');
    blur_us = 0U;
    puts("PASS C39-06 bar/space blur");

    puts("PASS: Code 39 full set both directions, speed drift, chatter, wrap and rejection");
    return 0;
}
