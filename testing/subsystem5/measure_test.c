/* Exercise real B5 consensus, including open space and sensor faults. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#define APP_H
#define IMPORT extern
typedef int INT;
typedef unsigned char UB;
#include "../../app_program/subsystem5/subsystem5.h"
#include "../../app_program/subsystem5/drv_ultrasonic.h"
static S5EchoReading readings[10];
static unsigned count, cursor;
static void tm_printf(UB *format, ...) { (void)format; }
static void tk_dly_tsk(unsigned ms) { (void)ms; }
S5EchoReading s5_ultrasonic_read_detailed(void)
{
    return cursor < count ? readings[cursor++] : (S5EchoReading){S5_ECHO_STUCK_HIGH, 0u, 0.0f};
}
S5EchoReading s5_ultrasonic_read_front(void)
{
    S5EchoReading r = s5_ultrasonic_read_detailed();
#if ROBOT_NO_TARGET_FORWARD
    if (r.status == S5_ECHO_NO_RISE || r.status == S5_ECHO_OUT_OF_RANGE) {
        r.status = S5_ECHO_OPEN_SPACE; r.cm = 400.0f;
    }
#endif
    return r;
}
const char *s5_echo_status_name(S5EchoStatus status) { (void)status; return "test"; }
#include "../../app_program/subsystem5/s5_measure.c"
static void load(S5EchoStatus status, float cm)
{
    count = 10u; cursor = 0u;
    for (unsigned i = 0u; i < count; ++i) readings[i] = (S5EchoReading){status, 0u, cm};
}
int main(void)
{
    float cm;
    load(S5_ECHO_OK, 120.0f);
    assert(s5_measure_stable_cm(&cm) && cm == 120.0f);
    load(S5_ECHO_NO_RISE, 0.0f);
    assert(!s5_measure_stable_cm(&cm)); /* Strict measurements never invent ranges. */
    load(S5_ECHO_NO_RISE, 0.0f);
    assert(s5_measure_front_cm(&cm) == (ROBOT_NO_TARGET_FORWARD != 0));
    load(S5_ECHO_OUT_OF_RANGE, 500.0f);
    assert(s5_measure_front_cm(&cm) == (ROBOT_NO_TARGET_FORWARD != 0));
    load(S5_ECHO_NO_RISE, 0.0f);
    assert(s5_measure_scan_cm(&cm) == (ROBOT_NO_TARGET_SCAN && ROBOT_NO_TARGET_FORWARD));
    for (int status = S5_ECHO_STUCK_HIGH; status <= S5_ECHO_TOO_SHORT; ++status) {
        if (status == S5_ECHO_NO_RISE) continue;
        load((S5EchoStatus)status, 0.0f);
        assert(!s5_measure_front_cm(&cm));
        load((S5EchoStatus)status, 0.0f);
        assert(!s5_measure_scan_cm(&cm));
    }
    load(S5_ECHO_NO_RISE, 0.0f);
    readings[0] = (S5EchoReading){S5_ECHO_OK, 150u, 2.5f};
    assert(s5_measure_front_cm(&cm) && cm == 2.5f); /* Close dominates assumed clear. */
    load(S5_ECHO_NO_RISE, 0.0f);
    readings[0] = (S5EchoReading){S5_ECHO_OK, 150u, 2.5f};
    readings[5] = readings[0];
    assert(!s5_measure_scan_cm(&cm)); /* Close dissent vetoes scan clearance. */
    assert(!s5_measure_front_cm(NULL));
    puts("PASS B5 consensus: open space, strict policy, close priority and hardware faults");
    return 0;
}
