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
/* ADC conversions averaged per line-sensor reading (noise). */
#define B3_ADC_OVERSAMPLE (8U)
/* Per-channel calibration from the 2026-11-20 course screenshots.
 * Higher ADC means tape. GP26 is physically right, GP27 left.
 * Centred on the spread-sensor line: GP26=195, GP27=142 (floor).
 * Both on a junction: GP26=355, GP27=842 (tape).
 * Small floor margins reject noise without reducing the tape endpoint.
 * Dashboard values remain raw; control uses the independent 0..1000 scales.
 */
#define B3_LINE_RAW_LOW_IS_BLACK (0U)
/* Final-course mats (run 2026-11-24 17:58): GP26 floor 260..340 (mostly
 * 280..300), tape 380..430 (mostly ~400); GP27 floor 160..280, tape
 * 1000+. The earlier 210/355 made the bare mat read 50..90 % "on the line"
 * under GP26, so the car steered hard right everywhere. White is set at the
 * TOP of the floor band so the mat reads 0; black at 375 still counts
 * earlier logs' 361..400 tape as a junction (>= 800 from ~365). Re-check
 * [LINE-RAW] if the sensors are moved or the mats change. */
/* Current stationary captures: floor GP26=205 / GP27=144,
 * black GP26=360 / GP27=539. Small floor margins reject noise.
 * These endpoints supersede the older course values above. */
#define B3_LINE_GP26_WHITE (220U)
#define B3_LINE_GP26_BLACK (360U)
#define B3_LINE_GP27_WHITE (160U)
#define B3_LINE_GP27_BLACK (539U)
/* 1: white follows each sensor's own floor (b3_floor_update), so the
 * same sensitivity holds on every mat.  Tuned on the 24 Nov runs (17:58,
 * 18:12, 18:38): GP26 floor 190..320 / tape 355..425; GP27 floor 125..225
 * / tape edge 300..700, fully on 1000+.  0: the fixed endpoints above. */
#ifndef B3_LINE_FLOOR_TRACK
#define B3_LINE_FLOOR_TRACK (0) /* use current measured endpoints */
#endif
/* init, lo, hi, band, tape, margin, black_min, span */
#define B3_LINE_GP26_FLOOR_CFG { 250U, 150U, 330U, 45U, 340U, 25U, 375U, 90U }
#define B3_LINE_GP27_FLOOR_CFG { 170U,  90U, 280U, 70U, 450U, 40U, 700U, 400U }
/* 1: once a sensor has seen both floor and tape (slide the car across the
 * line once after power-up), normalise with its own learned white/black
 * instead of the fixed endpoints above.  Needed whenever the sensors are
 * moved: GP26's fixed span is only ~100 counts, so a few counts of change
 * in its floor reading made it look "on the line" all the time.  Readings
 * darker than black + 2 * (black - white) + 100 (car lifted) are never learned. */
#ifndef B3_LINE_LEARN_RANGE
#define B3_LINE_LEARN_RANGE (0) /* off: fixed endpoints */
#endif
/* Zero is off-line; all other values use the measured channel endpoints. */
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
 * mapping on the normalised difference (right - left): a 60-count deadband,
 * rising to full steer at a 650-count difference (one sensor ~65% onto the
 * white).  The Pololu weighted position, 500 * (R - L) / (R + L) for two
 * sensors, gave only ~270 permille with one sensor half off the line,
 * against 800 here: the car ran straight off at curves.
 * CALIBRATE on the course: lower FULL_SCALE (stronger) if curves are
 * missed, raise it (gentler) if the car weaves on straights. */
#define B3_LINE_STEER_DEADBAND (60)
#define B3_LINE_STEER_ERROR_FULL_SCALE (650)
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

/* Barcode letter code, as on the wire (v2): 0 = none, A..Z = 1..26. */
typedef enum {
	B3_BARCODE_NONE = 0,
	B3_BARCODE_A = 1,
	B3_BARCODE_Z = 26
} B3_BARCODE;
#define B3_BARCODE_CODE(letter) ((uint8_t)((letter) - 'A' + 1))

/* Action for a barcode (values = comm_nav_cmd_t, 4 bits on the wire).
 * LEFT..UTURN, STOP, SPIN and FINISH wait for the next crossing; SLOW and
 * NORMAL change the following speed as soon as the barcode is read. */
typedef enum {
	B3_NAV_NONE = 0,
	B3_NAV_LEFT,
	B3_NAV_RIGHT,
	B3_NAV_STRAIGHT,
	B3_NAV_UTURN,
	B3_NAV_STOP,    /* stop 3 s at the crossing, then go straight */
	B3_NAV_SLOW,    /* half following speed until NORMAL          */
	B3_NAV_NORMAL,  /* normal following speed                     */
	B3_NAV_SPIN,    /* 360-degree spin at the crossing, straight  */
	B3_NAV_FINISH,  /* stop at the crossing and end the run       */
	B3_NAV_COUNT
} B3_NAV_COMMAND;

/**
 * @brief Packed current readings and most recently decoded navigation data.
 *
 * The packed attribute is supported by the RP2040 GNU C toolchain used by
 * this project. Its packed 37-bit payload occupies 5 bytes on this toolchain;
 * allocation order is compiler-specific, so this is not a wire-format type.
 */
typedef struct __attribute__((__packed__)) {
	unsigned int ir_left : 12;
	unsigned int ir_centre : 12;
	unsigned int ir_right : 1;
	unsigned int line_flags : 3;
	unsigned int last_barcode : 5;
	unsigned int last_nav_command : 4;
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

/** Latest raw ADC readings of the two line sensors (GP26, GP27). */
IMPORT void b3_line_raw(uint16_t *gp26, uint16_t *gp27);

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
/* True while the barcode sensor is crossing bars (an edge within the last
 * B3_BARCODE_ACTIVE_US); the controller then holds a slow, steady speed so
 * bar widths are measured cleanly. */
#define B3_BARCODE_ACTIVE_US (400000U)
IMPORT bool b3_barcode_active(void);
/* Count of rejected barcode scans (bars crossed, no valid *X*); it only
 * increases.  The controller backs up and re-reads when it changes. */
IMPORT uint16_t b3_barcode_rejects(void);

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
