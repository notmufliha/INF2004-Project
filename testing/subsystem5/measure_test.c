/*
 * Host test for app_program/subsystem5/s5_measure.c (no Pico needed).
 * The real consensus code runs against a scripted ultrasonic driver.
 *
 *   S5-M01  a valid nearby echo is reported as its own distance
 *   S5-M02  "nothing within range" pings (NO_FALL / OUT_OF_RANGE) give
 *           a clear far reading, so the car may drive in open space
 *   S5-M03  sensor faults (NO_RISE, STUCK_HIGH, glitch) never grant drive
 *   S5-M04  a mix of real echoes and faults still needs 3 agreeing pings
 *
 * Build/run: sh testing/subsystem5/run_measure_tests.sh
 */
#include <stdio.h>
#include "subsystem5.h"
#include "s5_measure.h"
#include "robot_config.h"

static S5EchoReading script[16];
static unsigned script_len;
static unsigned script_pos;
static unsigned failures;

S5EchoReading s5_ultrasonic_read_detailed(void)
{
    S5EchoReading none = { S5_ECHO_NO_RISE, 0u, 0.0f };

    if (script_pos < script_len) {
        return script[script_pos++];
    }
    return none;
}

ER tk_dly_tsk(RELTIM dlytim)
{
    (void)dlytim;
    return E_OK;
}

static void load(const S5EchoStatus *status, const float *cm, unsigned n)
{
    unsigned i;

    for (i = 0u; i < n; ++i) {
        script[i].status = status[i];
        script[i].cm = cm[i];
        script[i].pulse_us = 0u;
    }
    script_len = n;
    script_pos = 0u;
}

static void check(const char *id, int condition)
{
    printf("%s %s\n", condition ? "PASS" : "FAIL", id);
    if (!condition) {
        ++failures;
    }
}

int main(void)
{
    float cm = -1.0f;
    bool ok;

    {
        const S5EchoStatus s[5] = { S5_ECHO_OK, S5_ECHO_OK, S5_ECHO_OK,
                                    S5_ECHO_OK, S5_ECHO_OK };
        const float d[5] = { 120.0f, 121.0f, 119.5f, 120.5f, 120.0f };
        load(s, d, 5u);
        ok = s5_measure_stable_cm(&cm);
        check("S5-M01 valid echo distance", ok && cm > 119.0f && cm < 122.0f);
    }
    {
        const S5EchoStatus s[5] = { S5_ECHO_NO_FALL, S5_ECHO_NO_FALL,
                                    S5_ECHO_OUT_OF_RANGE, S5_ECHO_NO_FALL,
                                    S5_ECHO_NO_FALL };
        const float d[5] = { 0.0f, 0.0f, 450.0f, 0.0f, 0.0f };
        load(s, d, 5u);
        ok = s5_measure_stable_cm(&cm);
#if ROBOT_NO_ECHO_MEANS_CLEAR
        check("S5-M02 no echo in range is clear",
              ok && cm >= ROBOT_RELEASE_CM && cm <= ROBOT_FAR_CLEAR_CM);
#else
        check("S5-M02 no echo in range fails closed", !ok);
#endif
    }
    {
        const S5EchoStatus s[5] = { S5_ECHO_NO_RISE, S5_ECHO_STUCK_HIGH,
                                    S5_ECHO_TOO_SHORT, S5_ECHO_NO_RISE,
                                    S5_ECHO_NO_RISE };
        const float d[5] = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f };
        load(s, d, 5u);
        ok = s5_measure_stable_cm(&cm);
        check("S5-M03 sensor faults never grant drive", !ok);
    }
    {
        const S5EchoStatus s[5] = { S5_ECHO_OK, S5_ECHO_NO_RISE, S5_ECHO_OK,
                                    S5_ECHO_STUCK_HIGH, S5_ECHO_NO_RISE };
        const float d[5] = { 30.0f, 0.0f, 30.5f, 0.0f, 0.0f };
        load(s, d, 5u);
        ok = s5_measure_stable_cm(&cm);
        check("S5-M04 two echoes plus faults are not enough", !ok);
    }
    {
        float out = -1.0f;
        check("S5-M05 near-field echo suppressed",
              !s5_echo_usable_cm(S5_ECHO_OK, 4.0f, &out));
        check("S5-M06 close echo kept for STOP",
              s5_echo_usable_cm(S5_ECHO_OK, 20.0f, &out) && out == 20.0f);
        check("S5-M07 no response is a fault",
              !s5_echo_usable_cm(S5_ECHO_NO_RISE, 0.0f, &out));
    }

    printf("s5_measure: %u failed\n", failures);
    return failures == 0u ? 0 : 1;
}
