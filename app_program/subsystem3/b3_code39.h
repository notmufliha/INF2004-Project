#ifndef B3_CODE39_H
#define B3_CODE39_H

#include <stdbool.h>
#include <stdint.h>

/* *X*: 9 elements + gap + 9 + gap + 9. */
#define B3_CODE39_RUN_COUNT (29U)
/* *X or X*: one frame character lost at the edge of the strip. */
#define B3_CODE39_PARTIAL_COUNT (19U)
/* Elements kept for one scan: a whole *X* plus room for stray marks. */
#define B3_CODE39_MAX_ELEMENTS (48U)
/* A level change that flips back within this time is sensor chatter, not a
 * bar or space; it is merged into the surrounding element.  A narrow bar is
 * ~3.4 mm, i.e. >= 8 ms even at full speed (408 mm/s). */
#define B3_CODE39_MIN_RUN_US (3000U)
/* No edge for this long ends a scan (the floor after the strip): the
 * buffered elements are then decoded as a whole.  The widest element, a
 * blurred 9 mm wide space at the slowest following speed (~40 mm/s), is
 * ~250 ms.  An element at least this long also separates two scans. */
#define B3_CODE39_QUIET_US (500000U)
/* Within one character, the narrowest wide bar must be at least 1.4x the
 * widest narrow bar, and likewise for spaces.  Bars and spaces are compared
 * separately because sensor blur makes bars read thinner, spaces wider. */
#define B3_CODE39_WIDE_RATIO_X10 (14U)

typedef enum
{
    B3_CODE39_REJECT_NONE = 0,
    B3_CODE39_REJECT_RATIO,   /* no clear wide/narrow split, or not 3 wide */
    B3_CODE39_REJECT_PATTERN, /* not a Code 39 character, or reads disagree */
    B3_CODE39_REJECT_GAP,     /* gap between characters is not narrow */
    B3_CODE39_REJECT_STOP,    /* stop character missing or wrong direction */
    B3_CODE39_REJECT_NO_START /* bars seen, but no '*' frame character */
} B3_CODE39_REJECT;

typedef struct
{
    /* Elements of the scan in progress, in the order they were crossed. */
    uint32_t widths_us[B3_CODE39_MAX_ELEMENTS];
    uint8_t  levels[B3_CODE39_MAX_ELEMENTS];
    uint8_t  count;
    uint8_t  scan_done; /* this scan already produced its symbol */
    uint32_t run_start_us;
    uint32_t pending_us; /* time of a level change not yet confirmed */
    uint8_t  current_level;
    uint8_t  pending_level;
    uint8_t  have_level;
    uint8_t  have_pending;
    uint8_t  skip_first; /* element before the first edge: not a bar */
    /* Diagnostics for the serial log. */
    uint8_t  last_reversed;    /* direction of the last decoded symbol */
    uint8_t  last_partial;     /* last symbol came from a *X or X* read */
    uint8_t  last_reject;      /* B3_CODE39_REJECT of the last failed scan */
    char     last_reject_char; /* payload character seen when rejected */
    uint16_t reject_count;     /* incremented on every failed scan */
    /* Widths of the last rejected scan, so the log shows why it failed. */
    uint32_t reject_widths_us[B3_CODE39_MAX_ELEMENTS];
    uint8_t  reject_elements;
} B3_CODE39_DECODER;

void b3_code39_reset (B3_CODE39_DECODER * decoder);
/* Feed one sensor level change; true and *symbol when a scan completes. */
bool b3_code39_feed (B3_CODE39_DECODER * decoder,
    uint8_t                              level,
    uint32_t                             timestamp_us,
    char *                               symbol);
/* Call periodically: confirms a pending level change once it has lasted
 * B3_CODE39_MIN_RUN_US, so the last bar of a barcode is decoded without
 * waiting for another edge, and ends a scan after B3_CODE39_QUIET_US
 * without an edge (decoding partial *X / X* reads then). */
bool b3_code39_poll (
    B3_CODE39_DECODER * decoder, uint32_t now_us, char * symbol);
/* Code 39 character for a 9-element wide mask (first element in bit 8),
 * or '\0' if none.  Exposed for tests. */
char b3_code39_lookup (uint16_t wide_mask);

#endif

/*** end of file ***/

