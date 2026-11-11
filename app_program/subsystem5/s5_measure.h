#ifndef S5_MEASURE_H
#define S5_MEASURE_H
#include <stdbool.h>
#include <stdint.h>
/* True for echoes that should not be used to authorize driving or scanning.
 * This does not convert an ignored echo into a safe distance. */
bool s5_echo_suppressed_cm(float cm);
/* Returns true only if 3 of 5 non-near-field echoes agree (with one bounded retry). A false return
 * is UNKNOWN and must never be used as permission to move forward. */
bool s5_measure_stable_cm(float *out_cm);
/* Forward policy may accept repeated no-target results. */
bool s5_measure_front_cm(float *out_cm);
/* Same consensus for scan rays; configured no-target rays are assumed clear. */
bool s5_measure_scan_cm(float *out_cm);
/* Repeated STOP chatter protection. 'locked' means a TEMPORARY hold,
 * never permission to ignore an echo. After the hold, re-arm requires
 * multiple consecutive, valid, agreeing stationary clearance batches. */
typedef struct {
    uint32_t window_start_ms;
    uint32_t hold_start_ms;
    uint32_t last_clear_ms;
    uint8_t stop_count;
    uint8_t clear_batches;
    bool locked;
} S5GuardStopHistory;

void s5_guard_stop_history_init(S5GuardStopHistory *history);
/* Returns true if a temporary hold is active or has just started. */
bool s5_guard_stop_history_record(S5GuardStopHistory *history,
                                  uint32_t now_ms);
/* Reset the recovery streak on any invalid, close or ambiguous batch. */
void s5_guard_stop_history_unclear(S5GuardStopHistory *history);
/* Returns true only when a hold has JUST been safely released. */
bool s5_guard_stop_history_clear(S5GuardStopHistory *history,
                                 uint32_t now_ms);
#endif
