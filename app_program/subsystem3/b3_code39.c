#include "b3_code39.h"

#include <stddef.h>
#include <stdint.h>

/* Full Code 39 set: nine elements (bar, space, ... bar), 1 = wide, first
 * element in bit 8.  Every character has exactly three wide elements. */
static const struct
{
    uint16_t mask;
    char     character;
} code39_table[] = {
    {0x034U, '0'},
    {0x121U, '1'},
    {0x061U, '2'},
    {0x160U, '3'},
    {0x031U, '4'},
    {0x130U, '5'},
    {0x070U, '6'},
    {0x025U, '7'},
    {0x124U, '8'},
    {0x064U, '9'},
    {0x109U, 'A'},
    {0x049U, 'B'},
    {0x148U, 'C'},
    {0x019U, 'D'},
    {0x118U, 'E'},
    {0x058U, 'F'},
    {0x00DU, 'G'},
    {0x10CU, 'H'},
    {0x04CU, 'I'},
    {0x01CU, 'J'},
    {0x103U, 'K'},
    {0x043U, 'L'},
    {0x142U, 'M'},
    {0x013U, 'N'},
    {0x112U, 'O'},
    {0x052U, 'P'},
    {0x007U, 'Q'},
    {0x106U, 'R'},
    {0x046U, 'S'},
    {0x016U, 'T'},
    {0x181U, 'U'},
    {0x0C1U, 'V'},
    {0x1C0U, 'W'},
    {0x091U, 'X'},
    {0x190U, 'Y'},
    {0x0D0U, 'Z'},
    {0x085U, '-'},
    {0x184U, '.'},
    {0x0C4U, ' '},
    {0x094U, '*'},
    {0x0A8U, '$'},
    {0x0A2U, '/'},
    {0x08AU, '+'},
    {0x02AU, '%'},
};

#define CODE39_START_STOP_MASK (0x094U)

char
b3_code39_lookup (uint16_t wide_mask)
{
    size_t index;

    for (index = 0U; index < sizeof(code39_table) / sizeof(code39_table[0]);
        ++index)
    {
        if (code39_table[index].mask == wide_mask)
        {
            return code39_table[index].character;
        }
    }
    return '\0';
}

/* A strip crossed from the far end delivers each character's nine
 * elements in reverse order: mirror the mask, then use the forward table. */
static uint16_t
mirror_mask (uint16_t mask)
{
    uint16_t mirrored = 0U;
    uint8_t  bit;

    for (bit = 0U; bit < 9U; ++bit)
    {
        if ((mask & (1U << bit)) != 0U)
        {
            mirrored |= (uint16_t)(1U << (8U - bit));
        }
    }
    return mirrored;
}

void
b3_code39_reset (B3_CODE39_DECODER * decoder)
{
    if (decoder != NULL)
    {
        *decoder            = (B3_CODE39_DECODER){0};
        decoder->skip_first = 1U;
    }
}

/* Insertion sort of a few widths, ascending. */
static void
sort_widths (uint32_t * widths, uint8_t count)
{
    uint8_t index;

    for (index = 1U; index < count; ++index)
    {
        uint32_t value  = widths[index];
        uint8_t  insert = index;

        while (insert > 0U && widths[insert - 1U] > value)
        {
            widths[insert] = widths[insert - 1U];
            insert--;
        }
        widths[insert] = value;
    }
}

/* True when "wider" is clearly a wide element next to "narrower". */
static bool
clearly_wider (uint32_t narrower, uint32_t wider)
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
static uint16_t
element_mask (const B3_CODE39_DECODER * decoder,
    uint8_t                             offset,
    uint32_t *                          narrow_space_us)
{
    uint32_t bars[5];
    uint32_t spaces[4];
    uint32_t bar_cut;
    uint32_t space_cut;
    uint16_t wide_mask  = 0U;
    uint8_t  wide_count = 0U;
    uint8_t  index;

    for (index = 0U; index < 9U; ++index)
    {
        uint8_t  expected_level = (index & 1U) == 0U
             ? decoder->levels[offset]
             : (uint8_t)(decoder->levels[offset] ^ 1U);
        uint32_t width          = decoder->widths_us[offset + index];

        if (width == 0U || decoder->levels[offset + index] != expected_level)
        {
            return 0U;
        }
        if ((index & 1U) == 0U)
        {
            bars[index / 2U] = width;
        }
        else
        {
            spaces[index / 2U] = width;
        }
    }
    sort_widths(bars, 5U);
    sort_widths(spaces, 4U);

    if (clearly_wider(bars[2], bars[3]) && clearly_wider(spaces[2], spaces[3]))
    {
        bar_cut          = bars[3]; /* two wide bars, one wide space */
        space_cut        = spaces[3];
        *narrow_space_us = spaces[2];
    }
    else if (!clearly_wider(bars[0], bars[4]) &&
        clearly_wider(spaces[0], spaces[1]))
    {
        bar_cut   = UINT32_MAX; /* $ / + %: no wide bar, three wide spaces */
        space_cut = spaces[1];
        *narrow_space_us = spaces[0];
    }
    else
    {
        return 0U;
    }

    for (index = 0U; index < 9U; ++index)
    {
        uint32_t cut = (index & 1U) == 0U ? bar_cut : space_cut;

        if (decoder->widths_us[offset + index] >= cut)
        {
            wide_mask |= (uint16_t)(1U << (8U - index));
            wide_count++;
        }
    }
    return wide_count == 3U ? wide_mask : 0U;
}

/* The gap between characters is a narrow space, so compare it with the
 * neighbouring character's narrow spaces (same colour, same blur). */
static bool
valid_intercharacter_gap (const B3_CODE39_DECODER * decoder,
    uint8_t                                         gap_index,
    uint8_t                                         next_index,
    uint32_t                                        narrow_space_us)
{
    return decoder->levels[gap_index] != decoder->levels[gap_index - 1U] &&
        decoder->levels[next_index] == decoder->levels[gap_index - 1U] &&
        decoder->widths_us[gap_index] != 0U &&
        (uint64_t)decoder->widths_us[gap_index] * 2U >= narrow_space_us &&
        (uint64_t)decoder->widths_us[gap_index] <=
        (uint64_t)narrow_space_us * 2U;
}

/* One character position of the scan, read both ways. */
typedef struct
{
    uint16_t mask;    /* 0: no clean wide/narrow split */
    char     forward; /* character read in crossing order, or '\0' */
    char     reverse; /* character if the strip was crossed backwards */
    uint32_t narrow_space_us;
} B3_CODE39_BLOCK;

static void
read_block (
    const B3_CODE39_DECODER * decoder, uint8_t offset, B3_CODE39_BLOCK * block)
{
    block->narrow_space_us = 0U;
    block->mask    = element_mask(decoder, offset, &block->narrow_space_us);
    block->forward = block->mask != 0U ? b3_code39_lookup(block->mask) : '\0';
    block->reverse =
        block->mask != 0U ? b3_code39_lookup(mirror_mask(block->mask)) : '\0';
}

static bool
is_payload (char character)
{
    return character != '\0' && character != '*';
}

static bool
gap_ok (const B3_CODE39_DECODER * decoder,
    uint8_t                       gap_index,
    const B3_CODE39_BLOCK *       next)
{
    return valid_intercharacter_gap(
        decoder, gap_index, (uint8_t)(gap_index + 1U), next->narrow_space_us);
}

/* Whole "*X*" starting at offset, crossed either way. */
static bool
full_read (const B3_CODE39_DECODER * decoder,
    uint8_t                          offset,
    char *                           symbol,
    uint8_t *                        reversed)
{
    B3_CODE39_BLOCK start;
    B3_CODE39_BLOCK payload;
    B3_CODE39_BLOCK stop;

    read_block(decoder, offset, &start);
    read_block(decoder, (uint8_t)(offset + 10U), &payload);
    read_block(decoder, (uint8_t)(offset + 20U), &stop);
    if (start.mask == 0U || payload.mask == 0U || stop.mask == 0U ||
        !gap_ok(decoder, (uint8_t)(offset + 9U), &payload) ||
        !gap_ok(decoder, (uint8_t)(offset + 19U), &stop))
    {
        return false;
    }
    if (start.forward == '*' && stop.forward == '*' &&
        is_payload(payload.forward))
    {
        *symbol   = payload.forward;
        *reversed = 0U;
        return true;
    }
    if (start.reverse == '*' && stop.reverse == '*' &&
        is_payload(payload.reverse))
    {
        *symbol   = payload.reverse;
        *reversed = 1U;
        return true;
    }
    return false;
}

/* A frame character next to a payload, and the scan direction. */
typedef struct
{
    char    frame;
    char    payload;
    uint8_t reversed;
} B3_CODE39_READING;

/* Two neighbouring characters at offset: a frame '*' next to a payload
 * ("*X" with the stop lost, or "X*" with the start lost), either way.
 * Returns the payload, or '\0'; *conflict when two readings disagree. */
static char
partial_read (const B3_CODE39_DECODER * decoder,
    uint8_t                             offset,
    uint8_t *                           reversed,
    bool *                              conflict)
{
    B3_CODE39_BLOCK           first;
    B3_CODE39_BLOCK           second;
    char                      found          = '\0';
    uint8_t                   found_reversed = 0U;
    const B3_CODE39_READING * candidate;
    uint8_t                   index;

    read_block(decoder, offset, &first);
    read_block(decoder, (uint8_t)(offset + 10U), &second);
    if (first.mask == 0U || second.mask == 0U ||
        !gap_ok(decoder, (uint8_t)(offset + 9U), &second))
    {
        return '\0';
    }
    {
        const B3_CODE39_READING readings[4] = {
            {first.forward, second.forward, 0U}, /* "*X" forwards  */
            {second.forward, first.forward, 0U}, /* "X*" forwards  */
            {first.reverse, second.reverse, 1U}, /* backwards, start */
            {second.reverse, first.reverse, 1U}, /* backwards, end   */
        };
        for (index = 0U; index < 4U; ++index)
        {
            candidate = &readings[index];
            if (candidate->frame != '*' || !is_payload(candidate->payload))
            {
                continue;
            }
            if (found != '\0' &&
                (found != candidate->payload ||
                    found_reversed != candidate->reversed))
            {
                *conflict = true;
                return '\0';
            }
            found          = candidate->payload;
            found_reversed = candidate->reversed;
        }
    }
    *reversed = found_reversed;
    return found;
}

static void
reject (B3_CODE39_DECODER * decoder, B3_CODE39_REJECT reason, char character)
{
    uint8_t index;

    decoder->last_reject      = (uint8_t)reason;
    decoder->last_reject_char = character;
    decoder->reject_count++;
    for (index = 0U; index < decoder->count; ++index)
    {
        decoder->reject_widths_us[index] = decoder->widths_us[index];
    }
    decoder->reject_elements = decoder->count;
}

/* Why a scan with bars in it gave no symbol: look beside the first '*'. */
static void
diagnose (B3_CODE39_DECODER * decoder, bool conflict)
{
    B3_CODE39_BLOCK block;
    B3_CODE39_BLOCK next;
    uint8_t         offset;

    if (conflict)
    {
        reject(decoder, B3_CODE39_REJECT_PATTERN, '?');
        return;
    }
    for (offset = 0U; (uint8_t)(offset + 9U) <= decoder->count; ++offset)
    {
        read_block(decoder, offset, &block);
        if (block.forward != '*' && block.reverse != '*')
        {
            continue;
        }
        if ((uint8_t)(offset + 19U) > decoder->count)
        {
            reject(decoder, B3_CODE39_REJECT_STOP, '\0');
            return;
        }
        read_block(decoder, (uint8_t)(offset + 10U), &next);
        if (next.mask == 0U)
        {
            reject(decoder, B3_CODE39_REJECT_RATIO, '\0');
        }
        else if (!is_payload(next.forward) && !is_payload(next.reverse))
        {
            reject(decoder, B3_CODE39_REJECT_PATTERN, '\0');
        }
        else
        {
            reject(decoder,
                B3_CODE39_REJECT_GAP,
                block.forward == '*' ? next.forward : next.reverse);
        }
        return;
    }
    reject(decoder, B3_CODE39_REJECT_NO_START, '\0');
}

static void
emit (B3_CODE39_DECODER * decoder,
    char                  character,
    uint8_t               reversed,
    uint8_t               partial,
    char *                symbol)
{
    *symbol                = character;
    decoder->last_reversed = reversed;
    decoder->last_partial  = partial;
    decoder->scan_done     = 1U;
}

/* The scan is over (quiet floor): decode whatever it holds, then start a
 * new one.  A scan already decoded while streaming is simply dropped. */
static bool
finish_scan (B3_CODE39_DECODER * decoder, char * symbol)
{
    bool    decoded = false;
    uint8_t offset;
    uint8_t reversed = 0U;
    char    character;

    if (decoder->scan_done == 0U && decoder->count >= 9U)
    {
        for (offset = 0U;
            (uint8_t)(offset + B3_CODE39_RUN_COUNT) <= decoder->count;
            ++offset)
        {
            if (full_read(decoder, offset, &character, &reversed))
            {
                emit(decoder, character, reversed, 0U, symbol);
                decoded = true;
                break;
            }
        }
        if (!decoded)
        {
            char    found          = '\0';
            uint8_t found_reversed = 0U;
            bool    conflict       = false;

            for (offset = 0U;
                (uint8_t)(offset + B3_CODE39_PARTIAL_COUNT) <= decoder->count &&
                !conflict;
                ++offset)
            {
                character = partial_read(decoder, offset, &reversed, &conflict);
                if (character == '\0')
                {
                    continue;
                }
                if (found != '\0' &&
                    (found != character || found_reversed != reversed))
                {
                    conflict = true;
                }
                found          = character;
                found_reversed = reversed;
            }
            if (found != '\0' && !conflict)
            {
                emit(decoder, found, found_reversed, 1U, symbol);
                decoded = true;
            }
            else
            {
                diagnose(decoder, conflict);
            }
        }
    }
    decoder->count     = 0U;
    decoder->scan_done = 0U;
    return decoded;
}

static void
append_run (B3_CODE39_DECODER * decoder,
    uint8_t                     level,
    uint32_t                    width_us,
    char *                      symbol,
    bool *                      decoded)
{
    uint8_t index;
    uint8_t reversed = 0U;
    char    character;

    if (decoder->skip_first != 0U)
    {
        /* The floor before the first edge seen: not part of a barcode. */
        decoder->skip_first = 0U;
        return;
    }
    if (width_us >= B3_CODE39_QUIET_US)
    {
        /* Long floor (or a long black like the junction tape): it ends the
         * scan before it and is not itself an element. */
        if (finish_scan(decoder, symbol))
        {
            *decoded = true;
        }
        return;
    }
    if (decoder->count >= B3_CODE39_MAX_ELEMENTS)
    {
        /* Keep the most recent elements; drop a bar/space pair so the
         * colours stay aligned. */
        for (index = 2U; index < B3_CODE39_MAX_ELEMENTS; ++index)
        {
            decoder->widths_us[index - 2U] = decoder->widths_us[index];
            decoder->levels[index - 2U]    = decoder->levels[index];
        }
        decoder->count = (uint8_t)(B3_CODE39_MAX_ELEMENTS - 2U);
    }
    decoder->widths_us[decoder->count] = width_us;
    decoder->levels[decoder->count]    = level;
    decoder->count++;

    /* Report a clean "*X*" as soon as its last bar ends. */
    if (decoder->scan_done == 0U && decoder->count >= B3_CODE39_RUN_COUNT &&
        full_read(decoder,
            (uint8_t)(decoder->count - B3_CODE39_RUN_COUNT),
            &character,
            &reversed))
    {
        emit(decoder, character, reversed, 0U, symbol);
        *decoded = true;
    }
}

/* Commit the pending level change: the element before it is complete. */
static bool
commit_pending (B3_CODE39_DECODER * decoder, char * symbol)
{
    bool decoded = false;

    append_run(decoder,
        decoder->current_level,
        decoder->pending_us - decoder->run_start_us,
        symbol,
        &decoded);
    decoder->current_level = decoder->pending_level;
    decoder->run_start_us  = decoder->pending_us;
    decoder->have_pending  = 0U;
    return decoded;
}

bool
b3_code39_feed (B3_CODE39_DECODER * decoder,
    uint8_t                         level,
    uint32_t                        timestamp_us,
    char *                          symbol)
{
    bool decoded = false;

    if (decoder == NULL || symbol == NULL || level > 1U)
    {
        return false;
    }
    if (decoder->have_level == 0U)
    {
        decoder->current_level = level;
        decoder->run_start_us  = timestamp_us;
        decoder->have_level    = 1U;
        return false;
    }

    if (decoder->have_pending != 0U)
    {
        if (level == decoder->pending_level)
        {
            return false; /* repeated edge, same level */
        }
        if (timestamp_us - decoder->pending_us < B3_CODE39_MIN_RUN_US)
        {
            /* Flipped straight back: the pending change was a spike.
             * Drop it; the element in progress simply continues. */
            decoder->have_pending = 0U;
            return false;
        }
        decoded = commit_pending(decoder, symbol);
    }

    if (level == decoder->current_level)
    {
        return decoded;
    }
    decoder->pending_level = level;
    decoder->pending_us    = timestamp_us;
    decoder->have_pending  = 1U;
    return decoded;
}

bool
b3_code39_poll (B3_CODE39_DECODER * decoder, uint32_t now_us, char * symbol)
{
    bool decoded = false;

    if (decoder == NULL || symbol == NULL || decoder->have_level == 0U)
    {
        return false;
    }
    if (decoder->have_pending != 0U &&
        now_us - decoder->pending_us >= B3_CODE39_MIN_RUN_US)
    {
        decoded = commit_pending(decoder, symbol);
    }
    if (decoder->have_pending == 0U && decoder->count != 0U &&
        now_us - decoder->run_start_us >= B3_CODE39_QUIET_US)
    {
        if (finish_scan(decoder, symbol))
        {
            decoded = true;
        }
    }
    return decoded;
}

/*** end of file ***/

