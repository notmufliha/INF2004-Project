#!/usr/bin/env python3
"""B5 host tests for the STOP/FORWARD chatter fix; no Pico or motor needed."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
src = root / 'app_program/subsystem5/s5_measure.c'
task = (root / 'app_program/subsystem5/task_obstacle.c').read_text()
config = (root / 'app_program/subsystem5/subsystem5.h').read_text()

# Test the exact B5 guard-history implementation in s5_measure.c, with
# only the hardware-facing includes and callbacks mocked out.
code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#define SUBSYSTEM5_H
#define ROBOT_CONFIG_H
#define ROBOT_STOP_CM 35.0f
#define ROBOT_IGNORE_ECHO_AT_OR_BELOW_CM 5.0f
#define ROBOT_SUPPRESS_REPEATING_ECHO_BANDS 0
#define ROBOT_ECHO_AGREEMENT_FRACTION 0.15f
#define ROBOT_ECHO_AGREEMENT_MIN_CM 3.0f
#define ROBOT_ECHO_SAMPLES 5u
#define ROBOT_ECHO_REQUIRED_AGREEING 3u
#define ROBOT_ECHO_BATCH_ATTEMPTS 2u
#define ROBOT_ECHO_BATCH_RETRY_PAUSE_MS 120u
#define ROBOT_ECHO_GAP_AFTER_FAILURE_MS 95u
#define S5_ECHO_GAP_MS 65u
#define S5_GUARD_STOP_LIMIT 3u
#define S5_GUARD_STOP_WINDOW_MS 15000u
#define S5_GUARD_COOLDOWN_MS 8000u
#define S5_GUARD_REARM_CLEAR_BATCHES 5u
#define S5_GUARD_REARM_MAX_GAP_MS 2500u
#include "DRIVER_HEADER"
static void tk_dly_tsk(unsigned delay) { (void)delay; }
static S5EchoReading g_readings[10];
static unsigned g_index;
S5EchoReading s5_ultrasonic_read_detailed(void) { return g_readings[g_index++ % 10]; }
S5EchoReading s5_ultrasonic_read_front(void) {
    S5EchoReading r = s5_ultrasonic_read_detailed();
    if (r.status == S5_ECHO_NO_RISE) { r.status = S5_ECHO_OPEN_SPACE; r.cm = 400.0f; }
    return r;
}
#include "MEASURE_SOURCE"
int main(void)
{
    S5GuardStopHistory h;
    float cm;
    unsigned i;
    s5_guard_stop_history_init(&h);
    assert(!h.locked && h.stop_count == 0u);
    assert(!s5_guard_stop_history_record(&h, 100u));
    assert(!s5_guard_stop_history_record(&h, 1000u));
    assert(s5_guard_stop_history_record(&h, 2000u));
    assert(h.locked && h.stop_count == 3u);
    /* A clear reading alone cannot bypass the cooldown. */
    for (i = 0; i < 5; ++i)
        assert(!s5_guard_stop_history_clear(&h, 2500u + i * 400u));
    assert(h.locked);
    /* A failure breaks the clear streak. */
    s5_guard_stop_history_unclear(&h);
    assert(h.clear_batches == 0u);
    /* 5 consecutive clear batches AFTER the hold may safely re-arm. */
    for (i = 0; i < 4; ++i)
        assert(!s5_guard_stop_history_clear(&h, 10000u + i * 400u));
    assert(h.locked);
    assert(s5_guard_stop_history_clear(&h, 11600u));
    assert(!h.locked && h.stop_count == 0u);
    assert(!s5_guard_stop_history_clear(&h, 12000u));
    /* No rearm when clear samples are separated by too long a gap. */
    s5_guard_stop_history_init(&h);
    assert(!s5_guard_stop_history_record(&h, 100u));
    assert(!s5_guard_stop_history_record(&h, 200u));
    assert(s5_guard_stop_history_record(&h, 300u));
    assert(!s5_guard_stop_history_clear(&h, 9000u));
    assert(!s5_guard_stop_history_clear(&h, 12000u));
    assert(h.clear_batches == 1u);
    /* A new stop window starts after the old 15-second window expires. */
    s5_guard_stop_history_init(&h);
    assert(!s5_guard_stop_history_record(&h, 100u));
    assert(!s5_guard_stop_history_record(&h, 16000u));
    assert(h.stop_count == 1u);
    assert(!s5_guard_stop_history_record(&h, 16500u));
    assert(s5_guard_stop_history_record(&h, 17000u));
    /* Unsigned millisecond arithmetic works across wraparound. */
    s5_guard_stop_history_init(&h);
    assert(!s5_guard_stop_history_record(&h, 0xffffff00u));
    assert(!s5_guard_stop_history_record(&h, 100u));
    assert(s5_guard_stop_history_record(&h, 1000u));
    assert(!s5_guard_stop_history_clear(&h, 5000u));
    for (i = 0; i < 4; ++i)
        assert(!s5_guard_stop_history_clear(&h, 9000u + i * 400u));
    assert(s5_guard_stop_history_clear(&h, 10600u));
    assert(s5_guard_stop_history_record(NULL, 1000u)); /* fail closed */
    assert(!s5_guard_stop_history_clear(NULL, 1000u));
    /* Stable clear consensus remains available for rearming while stopped. */
    for (i = 0; i < 10; ++i) {
        g_readings[i].status = S5_ECHO_OK;
        g_readings[i].cm = 310.0f + (float)(i % 3);
    }
    g_index = 0;
    assert(s5_measure_stable_cm(&cm));
    assert(cm > 309.0f && cm < 313.0f);
    /* No echo is NOT clearance. */
    for (i = 0; i < 10; ++i) g_readings[i].status = S5_ECHO_NO_FALL;
    g_index = 0;
    assert(!s5_measure_stable_cm(&cm));
    /* Open-space forward policy never affects side-scan measurements. */
    for (i = 0; i < 10; ++i) g_readings[i].status = S5_ECHO_NO_RISE;
    g_index = 0;
    assert(s5_measure_front_cm(&cm) && cm == 400.0f);
    g_index = 0;
    assert(!s5_measure_stable_cm(&cm));
    /* A single real close echo overrides all assumed-open samples. */
    g_readings[2].status = S5_ECHO_OK; g_readings[2].cm = 20.0f;
    g_index = 0;
    assert(s5_measure_front_cm(&cm) && cm == 20.0f);
    for (i = 0; i < 10; ++i) g_readings[i].status = S5_ECHO_NO_FALL;
    g_index = 0;
    assert(!s5_measure_front_cm(&cm));
    puts("B5 timed guard hold + safe recovery + stopped consensus: PASS");
    return 0;
}
'''
code = code.replace('DRIVER_HEADER', str(root / 'app_program/subsystem5/drv_ultrasonic.h'))
code = code.replace('MEASURE_SOURCE', str(src))

# Ensure the running path no longer calls the multi-ping blocking routine.
start = task.index('        if (driving) {', task.index('static void stationary_profile_mode'))
end = task.index('        valid = s5_measure_front_cm(&cm);', start)
running = task[start:end]
assert 's5_ultrasonic_read_front()' in running
assert 'b2_request_forward();' in running
assert 's5_demo_clear_stop();' in running
assert 's5_demo_clear_publish();' in running
assert 'tk_dly_tsk(S5_FRONT_POLL_MS);' in running
assert 'continue;' in running
assert 's5_measure_stable_cm' not in running
assert 's5_guard_stop_history_record' in running
assert 's5_guard_stop_history_clear' in task
assert 's5_guard_stop_history_unclear' in task
assert '#define S5_GUARD_STOP_LIMIT 3u' in config
assert '#define S5_GUARD_COOLDOWN_MS 8000u' in config
assert '#define S5_GUARD_REARM_CLEAR_BATCHES 5u' in config
assert '#define S5_MOTION_BRIDGE_ENABLED 0' in config

with tempfile.TemporaryDirectory(prefix='b5_guard_') as tmp:
    source = Path(tmp) / 'guard_test.c'
    binary = Path(tmp) / 'guard_test'
    source.write_text(code)
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('B5 non-blocking moving-guard source checks: PASS')
