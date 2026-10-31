/**
 * @file b3_line_barcode.h
 * @brief Buddy 3 IR line sampling and GP28 barcode edge capture.
 */
#ifndef B3_LINE_BARCODE_H
#define B3_LINE_BARCODE_H

#include <stdint.h>

#include "../app.h"

/* WARNING: Validate the 20 ms period against sensor response and car speed. */
#define B3_LINE_UPDATE_MS (20U)
#define B3_NAV_QUEUE_LENGTH (8U)

#define B3_LINE_F_DETECTED (0x01U)
#define B3_LINE_F_LOST (0x02U)
#define B3_LINE_F_JUNCTION (0x04U)

typedef enum {
	B3_BARCODE_NONE = 0,
	B3_BARCODE_A,
	B3_BARCODE_B,
	B3_BARCODE_C,
	B3_BARCODE_D
} B3_BARCODE;

typedef enum {
	B3_NAV_NONE = 0,
	B3_NAV_LEFT,
	B3_NAV_RIGHT,
	B3_NAV_STRAIGHT,
	B3_NAV_UTURN
} B3_NAV_COMMAND;

/**
 * @brief Packed current readings and most recently decoded navigation data.
 *
 * The packed attribute is supported by the RP2040 GNU C toolchain used by
 * this project. Its bit-fields occupy two 32-bit allocation units (8 bytes);
 * allocation order is compiler-specific, so this is not a wire-format type.
 */
typedef struct __attribute__((__packed__)) {
	unsigned int ir_left : 12;
	unsigned int ir_centre : 12;
	unsigned int ir_right : 1;
	unsigned int line_flags : 3;
	unsigned int last_barcode : 3;
	unsigned int last_nav_command : 3;
} B3_STATE;

_Static_assert(sizeof(B3_STATE) == 8U,
	       "B3_STATE must occupy eight bytes on the RP2040 toolchain");

/* Message buffer owned by B3; each message is one B3_NAV_COMMAND byte. */
IMPORT ID mbf_nav_cmd;
IMPORT ID mtx_b3_state;

/**
 * @brief Create the ADC handle, navigation queue, and state mutex.
 * @return E_OK on success or the first μT-Kernel error.
 */
IMPORT ER b3_init(void);

/**
 * @brief Configure GP28 and start the line-sampling and barcode tasks.
 * @return E_OK on success or an initialization/task-start error.
 *
 * Call from core 0 after b3_init(). The navigation consumer must receive
 * one uint8_t message from mbf_nav_cmd for each decoded command.
 */
IMPORT ER b3_start(void);

/**
 * @brief Copy the packed B3 state while holding its mutex.
 * @param state Destination for the snapshot.
 * @return E_OK on success, E_PAR for a null pointer, otherwise a kernel error.
 */
IMPORT ER b3_get_state(B3_STATE *state);

/**
 * @brief Sample the GP26/GP27 ADC channels and publish line telemetry.
 * @param stacd μT-Kernel task start code (unused).
 * @param exinf μT-Kernel extended information (unused).
 */
IMPORT void b3_line_task(INT stacd, void *exinf);

/**
 * @brief Drain timestamped GP28 edges and decode pulse widths outside the ISR.
 * @param stacd μT-Kernel task start code (unused).
 * @param exinf μT-Kernel extended information (unused).
 */
IMPORT void b3_barcode_task(INT stacd, void *exinf);

#endif /* B3_LINE_BARCODE_H */
