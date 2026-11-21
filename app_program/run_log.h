/**
 * @file run_log.h
 * @brief Run recorder: recent driving history, dumped over USB as CSV.
 *
 * While the car runs, the controller stores one record per 20 ms tick
 * (sensors, flags, state, steering, speed, junction and barcode state),
 * and B3/VC add event records (barcode read/rejected, junction, turn,
 * line lost, barcode-sensor black runs).  The car does NOT need the USB
 * cable while driving: afterwards plug it in and run
 * tools/capture_run_log.ps1, which opens the port (DTR) and saves the dump
 * to logs/.  Commit and push that file for analysis.
 */
#ifndef RUN_LOG_H
#define RUN_LOG_H

#include <stdbool.h>
#include <stdint.h>

#ifndef RUN_LOG_ENABLE
#define RUN_LOG_ENABLE 1
#endif
#define RUN_LOG_RECORDS 1200u /* ~16-24 s depending on diagnostic/event rate */

typedef enum {
    RUN_EV_NONE = 0,
    RUN_EV_START,          /* controller started a run                 */
    RUN_EV_STOP,           /* run ended (STOP, safe stop, FINISH...)   */
    RUN_EV_BARCODE_READ,   /* arg = letter                             */
    RUN_EV_BARCODE_REJECT, /* arg = B3_CODE39_REJECT reason            */
    RUN_EV_GP28_RUN,       /* arg = mm the barcode sensor was off floor */
    RUN_EV_JUNCTION,       /* junction latched                         */
    RUN_EV_NAV_START,      /* arg = navigation command (turn) started   */
    RUN_EV_NAV_DONE,       /* arg = navigation command                 */
    RUN_EV_NAV_WAITING,    /* arg = curve_steer: a GP28 tape with a turn
                            * pending did not count as a junction     */
    RUN_EV_LINE_LOST,      /* line search started                      */
    RUN_EV_RESCAN,         /* arg = mm reversed for a barcode re-read  */
    RUN_EV_LINE_AGE,       /* arg = age of the sensor pair in ms       */
    RUN_EV_LOOP_TIME,      /* arg = controller start-to-start ms       */
    RUN_EV_MEAS_LEFT,      /* arg = signed measured left counts/s     */
    RUN_EV_MEAS_RIGHT,     /* arg = signed measured right counts/s    */
    RUN_EV_MOTION_AGE,     /* arg = age of the wheel-speed snapshot ms */
    RUN_EV_COUNT
} RUN_EVENT;

/* Bits of RUN_TICK.bits */
#define RUN_BIT_GP28       0x0001u
#define RUN_BIT_DETECTED   0x0002u
#define RUN_BIT_LOST       0x0004u
#define RUN_BIT_JFLAG      0x0008u /* B3 both-black JUNCTION flag     */
#define RUN_BIT_TAPE       0x0010u /* GP28 crossing-tape evidence     */
#define RUN_BIT_JLATCH     0x0020u /* junction latched                */
#define RUN_BIT_BARCODE    0x0040u /* barcode bars being crossed      */
#define RUN_BIT_FLOOR      0x0080u /* GP28 floor level learned        */
#define RUN_BIT_STRADDLE   0x0100u /* spread sensor layout            */

typedef struct {
    uint32_t t_ms;
    uint32_t travel_mm;
    uint16_t raw26, raw27;  /* raw ADC (B3)                     */
    uint16_t n26, n27;      /* normalised 0..1000 (controller)  */
    int16_t steer, speed;   /* last DRIVE sent                  */
    int16_t curve;          /* recent hardest steer (curve_steer) */
    uint16_t bits;
    uint8_t state, pending, nav, event;
    uint16_t arg;
} RUN_TICK;

#if RUN_LOG_ENABLE
void run_log_raw(uint16_t raw26, uint16_t raw27);
void run_log_tick(RUN_TICK *tick);  /* fills raw26/raw27 itself */
void run_log_event(uint32_t now_ms, uint8_t event, uint16_t arg);
void run_log_control(uint32_t now_ms, uint32_t line_age, uint32_t loop_ms,
                     int16_t left_cps, int16_t right_cps, uint32_t motion_age);
int  run_log_start(void);           /* starts the dump task       */
#else
static inline void run_log_raw(uint16_t a, uint16_t b) { (void)a; (void)b; }
static inline void run_log_tick(RUN_TICK *t) { (void)t; }
static inline void run_log_event(uint32_t n, uint8_t e, uint16_t a)
{ (void)n; (void)e; (void)a; }
static inline void run_log_control(uint32_t n, uint32_t l, uint32_t t,
                                  int16_t a, int16_t b, uint32_t m)
{ (void)n; (void)l; (void)t; (void)a; (void)b; (void)m; }
static inline int run_log_start(void) { return 0; }
#endif

#endif /* RUN_LOG_H */
