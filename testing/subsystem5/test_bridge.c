/* Host-only bridge contract test; no RP2040 or motor power required. */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* Suppress the real μT-Kernel B2 header while testing with B2 fakes. */
#define B2_MOTOR_H
static unsigned g_stop_calls;
static bool g_stopped;
void b2_request_stop(void) { ++g_stop_calls; g_stopped = true; }
bool b2_motor_is_stopped(void) { return g_stopped; }

#include "../../app_program/subsystem5/s5_bridge.c"

static unsigned g_requests;
static unsigned g_vc_stops;
static S5Command g_last_command;
static bool g_ready;
static bool g_motion_done;
static bool g_line_found;
static bool vc_ready(void) { return g_ready; }
static bool vc_request(S5Command cmd, const S5Profile *p)
{
    (void)p;
    ++g_requests;
    g_last_command = cmd;
    return cmd != S5_CMD_REVERSE; /* No rear clearance: reverse rejected. */
}
static bool vc_motion_done(void) { return g_motion_done; }
static bool vc_line_found(void) { return g_line_found; }
static void vc_stop(void) { ++g_vc_stops; }

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    exit(1); \
} } while (0)

int main(void)
{
    S5Profile profile = {0};
    S5BridgeDriver missing = {0};
    const S5BridgeDriver vc = {
        vc_ready, vc_request, vc_motion_done, vc_line_found, vc_stop
    };
    CHECK(!s5_bridge_ready());
    CHECK(!s5_bridge_motion_complete());
    CHECK(!s5_bridge_line_found());
    CHECK(!s5_bridge_request(S5_CMD_GO_LEFT, &profile));
    CHECK(g_requests == 0u);
    CHECK(!s5_bridge_install(NULL));
    CHECK(!s5_bridge_install(&missing));
    CHECK(s5_bridge_request(S5_CMD_STOP, &profile));
    CHECK(g_stop_calls == 1u);
    CHECK(s5_bridge_stopped());
    s5_bridge_emergency_stop();
    CHECK(g_stop_calls == 2u);
    CHECK(g_vc_stops == 0u);
    CHECK(s5_bridge_install(&vc));
    CHECK(!s5_bridge_install(&vc));
    CHECK(!s5_bridge_ready());
    CHECK(!s5_bridge_request(S5_CMD_GO_RIGHT, &profile));
    CHECK(g_requests == 0u);
    g_ready = true;
    CHECK(s5_bridge_ready());
    CHECK(s5_bridge_request(S5_CMD_GO_LEFT, &profile));
    CHECK(g_last_command == S5_CMD_GO_LEFT);
    CHECK(g_requests == 1u);
    CHECK(!s5_bridge_request(S5_CMD_REVERSE, &profile));
    CHECK(g_requests == 2u);
    CHECK(!s5_bridge_motion_complete());
    g_motion_done = true;
    CHECK(s5_bridge_motion_complete());
    CHECK(!s5_bridge_line_found());
    g_line_found = true;
    CHECK(s5_bridge_line_found());
    CHECK(s5_bridge_request(S5_CMD_STOP, &profile));
    CHECK(g_stop_calls == 3u);
    CHECK(g_requests == 3u);
    s5_bridge_emergency_stop();
    CHECK(g_stop_calls == 4u);
    CHECK(g_vc_stops == 1u);
    g_ready = false;
    CHECK(!s5_bridge_motion_complete());
    CHECK(!s5_bridge_line_found());
    puts("PASS B5 bridge: fail-closed, VC handoff, B2 STOP, motion/line status");
    return 0;
}
