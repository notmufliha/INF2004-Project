#ifndef B3_CODE39_H
#define B3_CODE39_H

#include <stdbool.h>
#include <stdint.h>

/* *X*: 9 elements + gap + 9 + gap + 9. */
#define B3_CODE39_RUN_COUNT (29U)
/* A level change that flips back within this time is sensor chatter, not a
 * bar or space; it is merged into the surrounding element.  A narrow bar is
 * ~3.4 mm, i.e. >= 8 ms even at full speed (408 mm/s). */
#define B3_CODE39_MIN_RUN_US (3000U)
/* Longer than a wide element at the robot's minimum scan speed. */
#define B3_CODE39_IDLE_US (1000000U)
/* Narrowest wide element must be at least 1.4x the widest narrow one
 * (value from a team car that read 20/20 barcodes on the track). */
#define B3_CODE39_WIDE_RATIO_X10 (14U)

typedef enum {
    B3_CODE39_REJECT_NONE = 0,
    B3_CODE39_REJECT_RATIO,   /* no clear wide/narrow split, or not 3 wide */
    B3_CODE39_REJECT_PATTERN, /* not a Code 39 character */
    B3_CODE39_REJECT_GAP,     /* gap between characters is not narrow */
    B3_CODE39_REJECT_STOP,    /* stop character missing or wrong direction */
    B3_CODE39_REJECT_START,   /* enough elements seen, but no start '*' */
    B3_CODE39_REJECT_INCOMPLETE /* scan stopped after a start '*' */
} B3_CODE39_REJECT;

typedef struct {
    uint32_t widths_us[B3_CODE39_RUN_COUNT];
    uint8_t levels[B3_CODE39_RUN_COUNT];
    uint32_t run_start_us;
    uint32_t pending_us;      /* time of a level change not yet confirmed */
    uint8_t run_count;
    uint8_t current_level;
    uint8_t pending_level;
    uint8_t have_level;
    uint8_t have_pending;
    uint8_t in_barcode;
    uint8_t payload_valid;
    uint8_t reversed;         /* 1 when the start character read backwards */
    char payload;
    /* Diagnostics for the serial log. */
    uint8_t last_reversed;    /* direction of the last decoded symbol */
    uint8_t last_reject;      /* B3_CODE39_REJECT of the last failed scan */
    char last_reject_char;    /* payload character seen when rejected */
    uint16_t reject_count;    /* incremented on every failed scan */
} B3_CODE39_DECODER;

void b3_code39_reset(B3_CODE39_DECODER *decoder);
/* Feed one sensor level change; true and *symbol when a scan completes. */
bool b3_code39_feed(B3_CODE39_DECODER *decoder, uint8_t level,
                    uint32_t timestamp_us, char *symbol);
/* Call periodically: confirms a pending level change once it has lasted
 * B3_CODE39_MIN_RUN_US, so the last bar of a barcode is decoded without
 * waiting for another edge. Also reports/resets abandoned scans after
 * B3_CODE39_IDLE_US without an edge, ready for a fresh strip. */
bool b3_code39_poll(B3_CODE39_DECODER *decoder, uint32_t now_us,
                    char *symbol);
/* Code 39 character for a 9-element wide mask (first element in bit 8),
 * or '\0' if none.  Exposed for tests. */
char b3_code39_lookup(uint16_t wide_mask);

#endif
