#include "subsystem5.h"
#include "../robot_config.h"
#include "../subsystem2/b2_motor.h"
#include "s5_logic.h"
#include "s5_sim.h"
#include "s5_bridge.h"
#include "drv_ultrasonic.h"
#include "s5_measure.h"
#include "drv_servo.h"
#include "obstacle_scan.h"

#if S5_SIMULATION
#include "s5_pico_demo.h"
#endif

#if !S5_SIMULATION && S5_HARDWARE_ENABLED
#if ROBOT_VERBOSE_SENSOR_LOG || S5_MOTION_BRIDGE_ENABLED
static void print_scan(const char *label, const S5Scan *s)
{
    int i;
    for (i = 0; i < S5_SCAN_COUNT; ++i) {
        const S5ScanPoint *p = &s->points[i];
        tm_printf((UB*)"[S5-%s] angle=%d valid=%d cm_x100=%d\n",
                  label, (int)p->angle_deg, (int)p->valid,
                  (int)(p->distance_cm * 100.0f));
    }
}
#endif
static void print_profile(const S5Profile *p)
{
    tm_printf((UB*)"[S5-PROFILE] valid=%d width_cm_x100=%d nearest_cm_x100=%d bearing_deg=%d center_x_cm_x100=%d center_y_cm_x100=%d left_cm_x100=%d right_cm_x100=%d required_cm_x100=%d side=%d\n",
              (int)p->valid, (int)(p->estimated_width_cm * 100.0f),
              (int)(p->nearest_cm * 100.0f), (int)p->nearest_angle_deg,
              (int)(p->center_x_cm * 100.0f), (int)(p->center_y_cm * 100.0f),
              (int)(p->left_clearance_cm * 100.0f),
              (int)(p->right_clearance_cm * 100.0f),
              (int)(p->required_lateral_clearance_cm * 100.0f),
              (int)p->bypass_side);
}

#if S5_MOTION_BRIDGE_ENABLED
static void emergency_halt(const char *reason)
{
    tm_printf((UB*)"[S5-FAULT] %s; requesting emergency stop\n", reason);
    s5_bridge_emergency_stop();
    s5_servo_angle(90u);
    for (;;) tk_dly_tsk(1000);
}

#endif /* S5_MOTION_BRIDGE_ENABLED */


#if !S5_MOTION_BRIDGE_ENABLED
/* Week 6 hardware demonstration. Real sensor + servo; no wheel movement.
 * The real B2/B3 tasks must replace the bridge before robot movement.
 * Logs use cm_x100 to avoid floating-point printf in the RTOS console. */
static void stationary_profile_mode(void)
{
    unsigned samples = 0u, close_hits = 0u, clear_hits = 0u;
    bool sensor_unavailable = false;
    bool obstacle_latched = false;
    bool scanned_this_obstacle = false;
    bool driving = false;
    bool startup_scan_pending = (S5_FORCE_STARTUP_SCAN != 0);
    tm_printf((UB*)"[S5] REAL ROBOT: clear -> forward; close -> STOP -> coarse/fine/profile\n");
    tm_printf((UB*)"[S5] SENSOR=REAL SERVO=REAL MOTORS=%s LINE=UNAVAILABLE\n",
              ROBOT_ENABLE_DRIVE ? "GUARDED FORWARD/STOP" : "DISABLED");
    tm_printf((UB*)"[S5] STOP below %d cm; RESUME after %d clear samples >= %d cm\n",
              (int)ROBOT_STOP_CM, (int)ROBOT_CLEAR_CONFIRM,
              (int)ROBOT_RELEASE_CM);
    b2_request_stop();
    tm_printf((UB*)"[S5] GPIO TRIG=16 ECHO=17 SERVO=12; confirm 3.3V-safe ECHO\n");
#if S5_STARTUP_SERVO_TEST
    tm_printf((UB*)"[S5-SERVO] SELF TEST 90 -> 80 -> 100 -> 90\n");
    s5_servo_angle(90u); tk_dly_tsk(500);
    s5_servo_angle(80u); tk_dly_tsk(650);
    s5_servo_angle(100u); tk_dly_tsk(650);
    s5_servo_angle(90u); tk_dly_tsk(650);
    tm_printf((UB*)"[S5-SERVO] SELF TEST END\n");
#endif
    /* On USB cold boots the servo and sensor may not yet be stable.
     * Keep both motors stopped while the supply settles, then throw
     * away a few first echoes. None of these readings authorizes drive. */
    /* Wait for power/servo stabilization without raw sensor diagnostics. */
    b2_request_stop();
    tk_dly_tsk(ROBOT_SENSOR_STARTUP_SETTLE_MS);
    {
        unsigned warmup;
        for (warmup = 0u; warmup < ROBOT_SENSOR_WARMUP_PINGS; ++warmup) {
            float discard = 0.0f;
            (void)s5_ultrasonic_read_cm(&discard);
            tk_dly_tsk(S5_ECHO_GAP_MS);
        }
    }
    tm_printf((UB*)"[S5] ready; waiting for confirmed clear path\n");
    for (;;) {
        float cm = 0.0f;
        bool valid;
        bool scan_needed = false;
        /* A five-ping consensus is too slow to be the only guard while
         * moving. Stop immediately on a single unknown/near/close echo,
         * then use consensus to decide whether the path can be released.
         * False-positive stops are safer than delayed collision stops. */
        if (driving) {
            S5EchoReading guard = s5_ultrasonic_read_detailed();
            if (guard.status != S5_ECHO_OK ||
                s5_echo_suppressed_cm(guard.cm) ||
                guard.cm < ROBOT_STOP_CM) {
                b2_request_stop();
                driving = false;
                tm_printf((UB*)"[S5-GUARD] immediate STOP status=%s cm_x100=%d; verifying next echoes\n",
                          s5_echo_status_name(guard.status),
                          (int)(guard.cm * 100.0f));
            }
            tk_dly_tsk(S5_ECHO_GAP_MS);
        }
        valid = s5_measure_stable_cm(&cm);
        ++samples;
        if (!valid) {
            close_hits = 0u;
            clear_hits = 0u;
            b2_request_stop();
            if (driving) tm_printf((UB*)"[S5-STOP] invalid ultrasonic echo: motors disabled\n");
            driving = false;
            if (!sensor_unavailable) {
                tm_printf((UB*)"[S5-STOP] no reliable distance; motors stopped until clear path confirmed\n");
                sensor_unavailable = true;
            }
        } else {
            sensor_unavailable = false;
            if (cm < ROBOT_STOP_CM) {
                /* One close sample must stop the wheels. Three are needed to scan. */
                b2_request_stop();
                if (driving) {
                    tm_printf((UB*)"[S5-STOP] object at %d cm (< %d cm); PWM=0 requested\n",
                              (int)cm, (int)ROBOT_STOP_CM);
                }
                driving = false;
                clear_hits = 0u;
                if (close_hits < S5_CONFIRM_HITS) ++close_hits;
                if (close_hits >= S5_CONFIRM_HITS) {
                    obstacle_latched = true;
                    if (!scanned_this_obstacle) scan_needed = true;
                }
#if ROBOT_VERBOSE_SENSOR_LOG
                tm_printf((UB*)"[S5-DETECT] sample=%d cm_x100=%d CLOSE=%d/%d state=STOP\n",
                          (int)samples, (int)(cm * 100.0f),
                          (int)close_hits, (int)S5_CONFIRM_HITS);
#endif
            } else if (cm >= ROBOT_RELEASE_CM) {
                close_hits = 0u;
                if (clear_hits < ROBOT_CLEAR_CONFIRM) ++clear_hits;
                if (clear_hits >= ROBOT_CLEAR_CONFIRM) {
                    if (obstacle_latched)
                        tm_printf((UB*)"[S5-RECOVER] path clear again; obstacle latch released\n");
                    obstacle_latched = false;
                    scanned_this_obstacle = false;
                    b2_request_forward(); /* Heartbeat every valid clear sample. */
                    if (!driving) {
                        tm_printf((UB*)"[S5-DRIVE] valid clear=%d cm; FORWARD requested (verify B2 FORWARD and wheel movement)\n",
                                  (int)cm);
                        driving = true;
                    }
                } else {
                    b2_request_stop();
                    driving = false;
                }
#if ROBOT_VERBOSE_SENSOR_LOG
                if (clear_hits <= ROBOT_CLEAR_CONFIRM || samples % 10u == 0u)
                    tm_printf((UB*)"[S5-CLEAR] sample=%d cm_x100=%d clear=%d/%d state=%s\n",
                              (int)samples, (int)(cm * 100.0f),
                              (int)clear_hits, (int)ROBOT_CLEAR_CONFIRM,
                              driving ? "FORWARD_REQUESTED" : "WAITING");
#endif
            } else {
                /* 35..45cm hysteresis band is not confirmed safe. */
                close_hits = 0u;
                clear_hits = 0u;
                b2_request_stop();
                if (driving) tm_printf((UB*)"[S5-STOP] distance in uncertain band; PWM=0 requested\n");
                driving = false;
#if ROBOT_VERBOSE_SENSOR_LOG
                tm_printf((UB*)"[S5-DETECT] sample=%d cm_x100=%d AMBIGUOUS state=STOP\n",
                          (int)samples, (int)(cm * 100.0f));
#endif
            }
        }
        if (obstacle_latched) {
            b2_request_stop();
            driving = false;
        }
        if (scan_needed || startup_scan_pending || S5_CONTINUOUS_SWEEP_TEST) {
            S5Scan coarse = {0}, fine = {0};
            S5Profile profile;
            S5Side side;
            SYSTIM started, ended;
            unsigned elapsed;
            startup_scan_pending = false;
            scanned_this_obstacle = true;
            close_hits = 0u;
            b2_request_stop();
            driving = false;
            tk_dly_tsk(ROBOT_STOP_SETTLE_MS);
            if (!b2_motor_is_stopped()) {
                tm_printf((UB*)"[S5-FAULT] motor PWM stop not acknowledged; scan aborted\n");
                tk_dly_tsk(S5_FRONT_POLL_MS);
                continue;
            }
            tk_get_tim(&started);
            tm_printf((UB*)"[S5-STOP] confirmed close obstacle; motor PWM=0 acknowledged\n");
            tm_printf((UB*)"[S5-COARSE] sweep=70,80,90,100,110 (real servo)\n");
            s5_coarse_scan(&coarse);
#if ROBOT_VERBOSE_SENSOR_LOG
            print_scan("COARSE", &coarse);
#endif
            side = s5_choose_side(&coarse);
            if (coarse.nearest_index < 0) {
                tm_printf((UB*)"[S5-FAULT] no valid coarse readings; cannot plan bypass\n");
            } else {
                uint16_t center = coarse.points[coarse.nearest_index].angle_deg;
                tm_printf((UB*)"[S5-FINE] center_deg=%d sweep=+/-10 deg in 5 deg steps (safe mount)\n", (int)center);
                s5_fine_scan(&fine, center);
#if ROBOT_VERBOSE_SENSOR_LOG
                print_scan("FINE", &fine);
#endif
                profile = s5_estimate_profile(&fine, side);
                if (profile.valid) {
                    int k;
                    float required = profile.required_lateral_clearance_cm;
                    float l = 0.0f, r = 0.0f;
                    bool left_ok = true, right_ok = true;
                    for (k = 0; k < 2; ++k)
                        if (!coarse.points[k].valid || coarse.points[k].distance_cm < S5_CLEAR_CM)
                            left_ok = false;
                    for (k = 3; k < 5; ++k)
                        if (!coarse.points[k].valid || coarse.points[k].distance_cm < S5_CLEAR_CM)
                            right_ok = false;
                    if (left_ok) l = coarse.points[0].distance_cm < coarse.points[1].distance_cm ?
                                     coarse.points[0].distance_cm : coarse.points[1].distance_cm;
                    if (right_ok) r = coarse.points[3].distance_cm < coarse.points[4].distance_cm ?
                                      coarse.points[3].distance_cm : coarse.points[4].distance_cm;
                    profile.left_clearance_cm = l;
                    profile.right_clearance_cm = r;
                    if (side == S5_SIDE_LEFT && l < required)
                        side = r >= required ? S5_SIDE_RIGHT : S5_SIDE_NONE;
                    else if (side == S5_SIDE_RIGHT && r < required)
                        side = l >= required ? S5_SIDE_LEFT : S5_SIDE_NONE;
                    profile.bypass_side = side;
                } else {
                    side = S5_SIDE_NONE;
                    tm_printf((UB*)"[S5-FAULT] width not bracketed by valid clear rays; no safe plan\n");
                }
                print_profile(&profile);
                tm_printf((UB*)"[S5-PLAN] direction=%s width_valid=%d action=%s\n",
                          side == S5_SIDE_LEFT ? "LEFT" : side == S5_SIDE_RIGHT ? "RIGHT" : "NONE",
                          (int)profile.valid,
                          side == S5_SIDE_NONE ? "STOP/REASSESS" : "BYPASS_REQUEST_ONLY");
                tm_printf((UB*)"[S5-RECOVER] physical bypass NOT enabled: no calibrated encoder/IR bridge\n");
            }
            tk_get_tim(&ended);
            elapsed = (unsigned)(ended.lo - started.lo);
            tm_printf((UB*)"[S5-METRIC] scan_profile_ms=%d (physical bypass not timed)\n", (int)elapsed);
            tm_printf((UB*)"[S5] scan complete; no repeat until 3 clear readings; motors remain stopped\n");
        }
        tk_dly_tsk(S5_FRONT_POLL_MS);
    }
}
#endif /* !S5_MOTION_BRIDGE_ENABLED */
#if S5_MOTION_BRIDGE_ENABLED
/* Actual controller: synchronous scans are converted into ready events on
 * the next tick; motion/line events must come from real team bridge. */
static void live_controller_mode(void)
{
    S5Controller c;
    S5Scan coarse = {0}, fine = {0};
    bool coarse_ready = false, fine_ready = false;
    unsigned invalid_motion_echo = 0;
    bool stop_requested = false;
    if (!s5_bridge_ready()) emergency_halt("Buddy 2/3 motion bridge unavailable");
    s5_controller_init(&c);
    tm_printf((UB*)"[S5] LIVE CONTROLLER GP16=TRIG GP17=ECHO GP12=SERVO\n");
    tm_printf((UB*)"[S5] B2 owns motors, B3 owns IR line, navigation arbitrates commands\n");
    for (;;) {
        S5Input in = {0};
        S5Command cmd;
        float cm = 0.0f;
        /* Sensor always faces forward outside scans. Monitoring continues
         * during movement. Repeated invalid readings fail closed. */
        in.front_valid = s5_ultrasonic_read_cm(&cm);
        in.front_cm = cm;
        if (c.state == S5_AVOID || c.state == S5_RECOVER || c.state == S5_REVERSE) {
            if (!in.front_valid) {
                if (++invalid_motion_echo >= S5_FRONT_INVALID_LIMIT)
                    emergency_halt("echo invalid during motion");
            } else {
                invalid_motion_echo = 0;
                if (cm < 10.0f) emergency_halt("very close obstacle during motion");
            }
        } else invalid_motion_echo = 0;
        in.stopped = s5_bridge_stopped();
        in.motion_complete = s5_bridge_motion_complete();
        in.line_found = s5_bridge_line_found();
        in.coarse_ready = coarse_ready;
        in.fine_ready = fine_ready;
        if (coarse_ready) in.coarse = coarse;
        if (fine_ready) in.fine = fine;
        coarse_ready = fine_ready = false;
        cmd = s5_controller_step(&c, &in);
        if (cmd != S5_CMD_NONE) {
            tm_printf((UB*)"[S5] state=%s command=%s error=%s\n",
                      s5_state_name(c.state), s5_command_name(cmd),
                      s5_error_name(c.error));
        }
        switch (cmd) {
        case S5_CMD_NONE: break;
        case S5_CMD_SCAN_COARSE:
            s5_coarse_scan(&coarse);
            print_scan("COARSE", &coarse);
            coarse_ready = true;
            break;
        case S5_CMD_SCAN_FINE:
            s5_fine_scan(&fine, c.fine_center_deg);
            print_scan("FINE", &fine);
            fine_ready = true;
            break;
        case S5_CMD_HALT:
            emergency_halt(s5_error_name(c.error));
            break;
        case S5_CMD_STOP:
            if (!stop_requested) {
                if (!s5_bridge_request(cmd, &c.profile))
                    emergency_halt("stop request rejected");
                stop_requested = true;
            }
            break;
        case S5_CMD_GO_LEFT:
        case S5_CMD_GO_RIGHT:
            print_profile(&c.profile);
            /* fall through */
        default:
            if (!s5_bridge_request(cmd, &c.profile))
                emergency_halt("motion or line command rejected");
            if (cmd == S5_CMD_RESUME) {
                /* Resume is a completed handoff, not a permanent DONE state. */
                s5_controller_init(&c);
                stop_requested = false;
            }
            break;
        }
        tk_dly_tsk(100);
    }
}
#endif /* S5_MOTION_BRIDGE_ENABLED */
#endif /* hardware mode */

EXPORT void s5_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
#if ROBOT_WHEELS_UP_MOTOR_TEST
    /* Bench-only boot motor proof: do not start ultrasonic/servo routines
     * or flood USB logs while B2 drives raised wheels for a short test.
     * The B2 test always stops and does not run autonomously. */
    tm_printf((UB*)"[S5] Bench motor boot test: sensor/servo tasks parked; wheels MUST be raised\n");
    for (;;) tk_dly_tsk(1000);
#else
#if S5_SIMULATION
    s5_pico_demo_run_forever();
#elif S5_HARDWARE_ENABLED && S5_SERVO_ENABLED
    tm_printf((UB*)"[FW-ID] 2026-10-21 PWM-ECHO-CAPTURE-V1: hardware pulse timing, obstacle safety ON\n");
    tm_printf((UB*)"[S5-BOOT] Buddy 5 task running, hardware profile GP16/17/12\n");
    s5_ultrasonic_init();
    tm_printf((UB*)"[S5-BOOT] ultrasonic pins initialized\n");
    s5_servo_init();
    tm_printf((UB*)"[S5-BOOT] servo PWM initialized at 50Hz\n");
    tk_dly_tsk(600);
#if S5_MOTION_BRIDGE_ENABLED
    live_controller_mode();
#else
    stationary_profile_mode();
#endif
#else
    tm_printf((UB*)"[S5] hardware/servo disabled\n");
    for (;;) tk_dly_tsk(1000);
#endif
#endif /* ROBOT_WHEELS_UP_MOTOR_TEST */
}
