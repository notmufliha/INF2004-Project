#ifndef B3_LINE_H
#define B3_LINE_H
#include "../app.h"
#include <stdint.h>

/* Line-only integration extracted from this project's B3 implementation at
 * commit 912c369. GP26=ADC0 left, GP27=ADC1 centre; low ADC means black.
 * The teaching consumer and GP28 barcode ownership remain independent. */
#define B3_LINE_THRESHOLD 2048u
#define B3_LINE_UPDATE_MS 20u
#define B3_LINE_F_DETECTED 0x01u
#define B3_LINE_F_LOST 0x02u
#define B3_LINE_F_JUNCTION 0x04u
typedef struct {
    uint16_t ir_left;
    uint16_t ir_centre;
    uint8_t line_flags;
} B3_STATE;
ER b3_line_init(void);
ER b3_line_start(void);
/* E_OBJ until a complete successful ADC pair, and immediately after failure.
 * Callers must check the timestamp as well as the return value. */
ER b3_get_timed_state(B3_STATE *state, uint32_t *sample_ms);
#endif
