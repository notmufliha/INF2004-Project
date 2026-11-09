/**
 * @file b3_line_barcode.h
 * @brief Buddy 3 interface consumed by the Vehicle Controller and B1.
 */
#ifndef B3_LINE_BARCODE_H
#define B3_LINE_BARCODE_H

#include <stdint.h>
#include "../app.h"
#include "cfg_line.h"

#define B3_LINE_UPDATE_MS (20U)
#define B3_NAV_QUEUE_LENGTH (8U)
#define B3_BARCODE_COUNT (26U)
#define B3_LINE_F_DETECTED (0x01U)
#define B3_LINE_F_LOST (0x02U)
#define B3_LINE_F_JUNCTION (0x04U)

typedef enum {
	B3_NAV_NONE = 0,
	B3_NAV_LEFT,
	B3_NAV_RIGHT,
	B3_NAV_STRAIGHT,
	B3_NAV_UTURN
} B3_NAV_COMMAND;

typedef struct {
	unsigned int ir_left : 12;
	unsigned int ir_centre : 12;
	unsigned int ir_right : 1;
	unsigned int line_flags : 3;
	unsigned int last_barcode : 5;
	unsigned int last_nav_command : 3;
} B3_STATE;

typedef char b3_state_layout_must_be_8_bytes
	[(sizeof(B3_STATE) == 8U) ? 1 : -1];

IMPORT ID mbf_nav_cmd;
IMPORT ID mtx_b3_state;
IMPORT ER b3_init(void);
IMPORT ER b3_start(void);
IMPORT ER b3_get_state(B3_STATE *state);
IMPORT ER b3_get_timed_state(B3_STATE *state, uint32_t *sample_ms);
IMPORT void b3_publish_barcode(uint8_t letter);

#endif
