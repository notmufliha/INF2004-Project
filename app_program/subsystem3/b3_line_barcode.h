/**
 * @file b3_line_barcode.h
 * @brief Buddy 3 IR line sampling and GP28 barcode edge capture.
 */
#ifndef B3_LINE_BARCODE_H
#define B3_LINE_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

/* WARNING: Validate the 20 ms period against sensor response and car speed. */
#define B3_LINE_UPDATE_MS (20U)
/* Track tuning: measured black is 100..200, white is about 1000, and a
 * lifted sensor reads 4000+. Raw 12-bit ADC is unchanged for telemetry.
 * Transition from fully black at 200 to fully white at 1000.
 * Change polarity if measured
 * black readings are HIGHER than white readings.
 *
 * Backward compatibility: the V2 comparison notes refer to
 * B3_CAL_LOW_IS_BLACK; keep that alias working without changing the
 * canonical macro used by the current implementation. */
#define B3_LINE_RAW_LOW_IS_BLACK (1U)
#ifndef B3_CAL_LOW_IS_BLACK
#define B3_CAL_LOW_IS_BLACK B3_LINE_RAW_LOW_IS_BLACK
#endif
#define B3_LINE_RAW_BLACK_MAX (200U)
#define B3_LINE_RAW_WHITE_MIN (1000U)
/* Only zero is invalid/off-line. Do not discard the measured black range. */
#define B3_LINE_RAW_LOST_BELOW (1U)

#define B3_NAV_QUEUE_LENGTH (8U)

#define B3_LINE_F_DETECTED (0x01U)
#define B3_LINE_F_LOST (0x02U)
#define B3_LINE_F_JUNCTION (0x04U)
/* Line values given to the controller (B3_STATE ir_left / ir_centre) are
 * normalized to 0 = off-line and 1000 = black. The dashboard shows raw ADC. */
/* Pololu QTRSensors readLine() thresholds: a sensor is "on the line" above
 * 200, and values of 50 or less are noise left out of the position. */
#define B3_LINE_ACTIVE_MIN (201U)
#define B3_LINE_NOISE_MAX (50U)
#define B3_LINE_ACTIVE_MAX (1000U)
#define B3_LINE_LEFT_ACTIVE(norm) \
	((uint16_t)(norm) >= B3_LINE_ACTIVE_MIN && \
	 (uint16_t)(norm) <= B3_LINE_ACTIVE_MAX)
#define B3_LINE_CENTRE_ACTIVE(norm) \
	((uint16_t)(norm) >= B3_LINE_ACTIVE_MIN && \
	 (uint16_t)(norm) <= B3_LINE_ACTIVE_MAX)
/* Line position (Pololu weighted average): 0 = under the robot's LEFT
 * sensor (ADC "centre"), 1000 = under its RIGHT sensor (ADC "left"),
 * B3_LINE_POS_CENTRE = centred.  Off the line it holds the last side. */
#define B3_LINE_POS_CENTRE (500)
/* Steering for THIS robot's sensor layout: the line is wider than the
 * sensor gap, so both sensors read black when centred and the only
 * steering signal appears when one of them reaches a line edge.  The car
 * then has just the sensor gap left before the line is lost, so it must
 * correct hard and at once.  Steering is the team's original, tested
 * mapping on the normalised difference (right - left): a 40-count deadband,
 * rising to full steer at a 400-count difference (one sensor ~40% onto the
 * white).  The Pololu weighted position, 500 * (R - L) / (R + L) for two
 * sensors, gave only ~270 permille with one sensor half off the line,
 * against 800 here: the car ran straight off at curves.
 * CALIBRATE on the course: lower FULL_SCALE (stronger) if curves are
 * missed, raise it (gentler) if the car weaves on straights. */
#define B3_LINE_STEER_DEADBAND (40)
#define B3_LINE_STEER_ERROR_FULL_SCALE (400)
/* 800 permille: inner wheel at 0.2x, outer at 1.8x (before the 5000 cps
 * outer-wheel limit), so sharp curves can be followed. */
#define B3_LINE_STEER_MAX (800)
/* A junction (cross line) puts BOTH sensors on solid black, reading about
 * equally (normalised).  Merely both "on the line" is normal following, not
 * a junction; treating it as one forced the car straight on every curve. */
#define B3_LINE_JUNCTION_MIN (800U)
#define B3_LINE_JUNCTION_BALANCE (150)
#define B3_LINE_IS_JUNCTION(left, centre) \
	((uint16_t)(left) >= B3_LINE_JUNCTION_MIN && \
	 (uint16_t)(centre) >= B3_LINE_JUNCTION_MIN && \
	 (uint16_t)(left) <= B3_LINE_ACTIVE_MAX && \
	 (uint16_t)(centre) <= B3_LINE_ACTIVE_MAX && \
	 ((int)(left) - (int)(centre)) <= B3_LINE_JUNCTION_BALANCE && \
	 ((int)(centre) - (int)(left)) <= B3_LINE_JUNCTION_BALANCE)

typedef enum {
	B3_BARCODE_NONE = 0,
	B3_BARCODE_A,
	B3_BARCODE_B,
	B3_BARCODE_C,
	B3_BARCODE_D,
	B3_BARCODE_Z
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
 * this project. Its packed 34-bit payload occupies 5 bytes on this toolchain;
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

_Static_assert(sizeof(B3_STATE) == 5U,
	       "B3_STATE must occupy five bytes on the RP2040 toolchain");

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
/* Timestamp is updated only after both ADC reads succeed. E_OBJ until then. */
IMPORT ER b3_get_timed_state(B3_STATE *state, uint32_t *sample_ms);
/* Hold the slow scan speed through wide bars at the minimum wheel speed.
 * Keep this aligned with B3_CODE39_IDLE_US. */
#define B3_BARCODE_ACTIVE_US (1000000U)
IMPORT bool b3_barcode_active(void);

/**
 * @brief Sample the GP26/GP27 ADC channels and publish line telemetry.
 * @param stacd μT-Kernel task start code (unused).
 * @param exinf μT-Kernel extended information (unused).
 */
IMPORT void b3_line_task(INT stacd, void *exinf);

/**
 * @brief Drain timestamped GP28 edges and decode Code 39 outside the ISR.
 * @param stacd μT-Kernel task start code (unused).
 * @param exinf μT-Kernel extended information (unused).
 */
IMPORT void b3_barcode_task(INT stacd, void *exinf);

#endif /* B3_LINE_BARCODE_H */
