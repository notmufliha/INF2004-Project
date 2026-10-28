/*
 *	error_codes.h
 *	Shared 1-byte error codes for comm_report_error() (Week 8 Tribunal:
 *	"error codes instead of text").
 *
 *	    bit  7 6 5 | 4 3 2 1 0
 *	         subsys | error number
 *
 *	3-bit subsystem (0 = system / Vehicle Controller, 1..5 = buddy) and
 *	a 5-bit error number (0..31) inside that subsystem's block.  These
 *	are #define constants, so they cost no RAM on the Pico.  The text for
 *	each code lives only in the laptop decoder (tools/s1_wire.py): keep
 *	ERROR_NAMES there in step with this file.
 *
 *	Each buddy owns their block and adds codes at the end of it; never
 *	renumber a code that has been used in a test log.
 */

#ifndef ERROR_CODES_H
#define ERROR_CODES_H

#define ERR_CODE(sub, num)	((UB)((((sub) & 0x07U) << 5) | ((num) & 0x1FU)))
#define ERR_SUBSYS(code)	(((code) >> 5) & 0x07U)
#define ERR_NUMBER(code)	((code) & 0x1FU)

/* 0: system / Vehicle Controller */
#define ERR_SYS_WATCHDOG	ERR_CODE(0, 1)	/* DRIVE refresh missed      */

/* 1: Buddy 1, WiFi & telemetry.  detail = count where it applies */
#define ERR_S1_TX_DROPPED	ERR_CODE(1, 1)	/* outgoing queue was full   */
#define ERR_S1_CMD_BUSY		ERR_CODE(1, 2)	/* VC command queue full     */
#define ERR_S1_TEST		ERR_CODE(1, 31)	/* used by the S1 self-tests */

/* 2: Buddy 2, motion control */
#define ERR_S2_MOTOR_STALL	ERR_CODE(2, 1)
#define ERR_S2_ENCODER		ERR_CODE(2, 2)
#define ERR_S2_LOOP_OVERRUN	ERR_CODE(2, 3)	/* 10 ms period missed       */
#define ERR_S2_MOTION_TIMEOUT ERR_CODE(2, 4)
#define ERR_S2_STOP_TIMEOUT   ERR_CODE(2, 5)
#define ERR_S2_COMMAND        ERR_CODE(2, 6)
#define ERR_S2_DRIVE_WATCHDOG ERR_CODE(2, 7)

/* 3: Buddy 3, line following and barcodes */
#define ERR_S3_LINE_LOST	ERR_CODE(3, 1)
#define ERR_S3_BAD_BARCODE	ERR_CODE(3, 2)

/* 4: Buddy 4, IMU */
#define ERR_S4_IMU_FAULT	ERR_CODE(4, 1)	/* readings suppressed       */

/* 5: Buddy 5, obstacles.  Number = S5Error (s5_logic.h), so B5 can
 *    report ERR_CODE(5, c.error) directly. */
#define ERR_S5(s5_error)	ERR_CODE(5, (s5_error))

#endif /* ERROR_CODES_H */
