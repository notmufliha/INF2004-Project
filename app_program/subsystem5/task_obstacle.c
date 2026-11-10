#include "subsystem5.h"
#include "../robot_config.h"
#include "../subsystem1/subsystem1.h"   /* B1 telemetry + shared fault mask */
#include "../subsystem2/b2_motor.h"
#include "../subsystem2/motion_command.h"
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif
#include "s5_logic.h"
#include "s5_sim.h"
#include "s5_bridge.h"
#include "drv_ultrasonic.h"
#include "s5_measure.h"
#include "drv_servo.h"
#include "obstacle_scan.h"
#include "s5_navigation.h"
#include "s5_mission.h"

/* The B1 wire format uses 9/7-bit obstacle fields.  B5's approved
 * application ranges are 0..400 cm (distance/clearance) and 0..100 cm
 * (width).  Invalid data uses B1's reserved all-ones sentinel. */
#define S5_WIRE_DISTANCE_MAX_CM (400.0f)
#define S5_WIRE_WIDTH_MAX_CM    (100.0f)
#define S5_FRONT_ANGLE_DETAIL  (45u) /* 90 degrees / 2, Tribunal A1 */

#if !S5_SIMULATION && S5_HARDWARE_ENABLED
static UH s5_wire_cm(float cm, float maximum_cm)
{
    /* NaN, infinities and negative distances are not measurements.
     * Finite distances beyond B5's range saturate rather than wrap. */
    if (!__builtin_isfinite(cm) || (cm < 0.0f))
    {
        return COMM_CM_INVALID;
    }

    if (cm > maximum_cm)
    {
        cm = maximum_cm;
    }

    return (UH)(cm + 0.5f);
}

static void s5_report_profile(const S5Profile *profile, S5Side side)
{
    UH distance_cm = COMM_CM_INVALID;
    UH width_cm = COMM_CM_INVALID;
    UH left_cm = COMM_CM_INVALID;
    UH right_cm = COMM_CM_INVALID;
    AVOID_ACTION action = AVOID_STOP;

    if ((profile != NULL) && profile->valid)
    {
        distance_cm = s5_wire_cm(profile->nearest_cm,
                                 S5_WIRE_DISTANCE_MAX_CM);
        width_cm = s5_wire_cm(profile->estimated_width_cm,
                              S5_WIRE_WIDTH_MAX_CM);
        left_cm = s5_wire_cm(profile->left_clearance_cm,
                             S5_WIRE_DISTANCE_MAX_CM);
        right_cm = s5_wire_cm(profile->right_clearance_cm,
                              S5_WIRE_DISTANCE_MAX_CM);

        if (side == S5_SIDE_LEFT)
        {
            action = AVOID_LEFT;
        }
        else if (side == S5_SIDE_RIGHT)
        {
            action = AVOID_RIGHT;
        }

        /* Never publish a partly valid profile or authorise bypass
         * when any field is malformed. The receiver sees one explicit
         * invalid STOP event, not misleading partial clearance data. */
        if ((distance_cm == COMM_CM_INVALID) ||
            (width_cm == COMM_CM_INVALID) ||
            (left_cm == COMM_CM_INVALID) ||
            (right_cm == COMM_CM_INVALID))
        {
            distance_cm = COMM_CM_INVALID;
            width_cm = COMM_CM_INVALID;
            left_cm = COMM_CM_INVALID;
            right_cm = COMM_CM_INVALID;
            action = AVOID_STOP;
        }
    }

    (void)comm_report_obstacle(distance_cm, width_cm, left_cm, right_cm,
                               action);
}
#endif /* !S5_SIMULATION && S5_HARDWARE_ENABLED */

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
    s5_mission_publish(S5_MISSION_FAULT);
    tm_printf((UB*)"[S5-FAULT] %s; requesting emergency stop\n", reason);
    s5_bridge_emergency_stop();
    s5_servo_angle(90u);

    for (;;)
    {
        tk_dly_tsk(1000);
    }
}

#endif /* S5_MOTION_BRIDGE_ENABLED */


#if S5_DEMO_B2_FORWARD_COMMANDS && !S5_DEMO_SENSOR_ONLY && !S5_MOTION_BRIDGE_ENABLED
/* This is a *guarded forward-only demo adapter*, not an obstacle-bypass VC.
 * B2 remains the sole PWM owner. No reverse, turn or line reacquisition.
 * The previous firmware only called b2_request_forward(), which is a safety
 * permission and does NOT create a MOTION_CMD_DRIVE. A separate task is
 * needed because B2's DRIVE command watchdog is 100 ms, while ultrasonic
 * sampling/scan operations can take much longer. */
#if ROBOT_MOTION_TEST != 0
#error "B5 forward-only demo requires ROBOT_MOTION_TEST=0 (disable B2 one-shot motion test)"
#endif
/* Shared between the sensor task and the B2 command-refresh task.
 * Use the same T-Kernel SMP spinlock API as Buddy 2; no libatomic dependency
 * on RP2040 Cortex-M0+. Never call B2 while holding this lock. */
static uint32_t s5_demo_last_clear_ms;
static bool s5_demo_clear_permission;
static bool s5_demo_drive_fault;
#if TK_SUPPORT_SMP
static T_SPLOCK s5_demo_lock;
#endif

static void s5_demo_lock_enter(UINT *irq)
{
#if TK_SUPPORT_SMP
    (void)ISpinLock(&s5_demo_lock, irq);
#else
    DI(*irq);
#endif
}
static void s5_demo_lock_leave(UINT irq)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&s5_demo_lock, irq);
#else
    EI(irq);
#endif
}
static void s5_demo_state_init(void)
{
#if TK_SUPPORT_SMP
    (void)InitSpinLock(&s5_demo_lock);
#endif
    s5_demo_last_clear_ms = 0u;
    s5_demo_clear_permission = false;
    s5_demo_drive_fault = false;
}
static bool s5_demo_fault_latched(void)
{
    UINT irq;
    bool fault;
    s5_demo_lock_enter(&irq);
    fault = s5_demo_drive_fault;
    s5_demo_lock_leave(irq);
    return fault;
}
static void s5_demo_state_read(uint32_t *last_clear_ms, bool *permitted)
{
    UINT irq;
    s5_demo_lock_enter(&irq);
    *last_clear_ms = s5_demo_last_clear_ms;
    *permitted = s5_demo_clear_permission;
    s5_demo_lock_leave(irq);
}
static void s5_demo_clear_stop(void)
{
    UINT irq;
    s5_demo_lock_enter(&irq);
    s5_demo_clear_permission = false;
    s5_demo_lock_leave(irq);
    b2_request_stop();
}
static void s5_demo_clear_publish(void)
{
    SYSTIM time;
    UINT irq;
    if (tk_get_tim(&time) != E_OK)
    {
        s5_demo_clear_stop();
        return;
    }
    s5_demo_lock_enter(&irq);
    if (!s5_demo_drive_fault)
    {
        s5_demo_last_clear_ms = time.lo;
        s5_demo_clear_permission = true;
    }
    s5_demo_lock_leave(irq);
}

static bool s5_demo_can_refresh(uint32_t now_ms, uint32_t last_clear_ms,
                                bool clear_permission, bool b2_allowed)
{
    return clear_permission && b2_allowed &&
           (uint32_t)(now_ms - last_clear_ms) <= S5_DEMO_CLEAR_MAX_AGE_MS;
}

static void s5_demo_drive_fault_stop(const char *reason)
{
    UINT irq;
    /* A PID/encoder/watchdog failure must not be automatically rearmed. */
    s5_demo_lock_enter(&irq);
    s5_demo_drive_fault = true;
    s5_demo_clear_permission = false;
    s5_demo_lock_leave(irq);
    b2_request_stop();
    tm_printf((UB*)"[S5-B2-FAULT] %s; motor drive locked until reset\n", reason);
}

static void s5_demo_drive_task(INT stacd, void *exinf)
{
    bool issued = false;
    uint32_t last_status_log_ms = 0u;
    (void)stacd;
    (void)exinf;
    for (;;)
    {
        SYSTIM time;
        motion_status_snapshot_t status;
        uint32_t last_clear;
        bool permitted;
        ER er;
        motion_command_t command = {0};
        if (s5_demo_fault_latched())
        {
            tk_dly_tsk(S5_DEMO_DRIVE_REFRESH_MS);
            continue;
        }
        if (tk_get_tim(&time) != E_OK)
        {
            s5_demo_drive_fault_stop("system clock unavailable");
            continue;
        }
        s5_demo_state_read(&last_clear, &permitted);
        if (!s5_demo_can_refresh(time.lo, last_clear, permitted,
                                 b2_safety_motion_allowed()))
        {
            if (permitted && (uint32_t)(time.lo - last_clear) >
                             S5_DEMO_CLEAR_MAX_AGE_MS)
            {
                /* A delayed ultrasonic task is a recoverable clearance
                 * timeout, NOT a permanent B2/encoder failure. Revoke
                 * permission and require stationary revalidation. */
                s5_demo_clear_stop();
                tm_printf((UB*)"[S5-B2] front clearance expired; STOP and recheck (not permanently locked)\n");
            }
            issued = false;
            tk_dly_tsk(S5_DEMO_DRIVE_REFRESH_MS);
            continue;
        }
        if (motion_get_status(&status) != E_OK ||
            (uint32_t)(time.lo - status.sample_time_ms) >
                S5_DEMO_B2_STATUS_MAX_AGE_MS)
        {
            s5_demo_drive_fault_stop("B2 controller status unavailable/stale");
            continue;
        }
        if (status.status == MOTION_STATUS_FAULT ||
            status.status == MOTION_STATUS_TIMEOUT)
        {
            tm_printf((UB*)"[S5-B2-STATUS] status=%d fault=%d L_cps=%d R_cps=%d\n",
                      (int)status.status, (int)status.fault,
                      (int)status.left_speed_cps, (int)status.right_speed_cps);
            s5_demo_drive_fault_stop("B2 controller/encoder fault or timeout");
            continue;
        }
        command.type = MOTION_CMD_DRIVE;
        command.direction = MOTION_DIR_FORWARD;
        command.target_speed_cps = S5_DEMO_SPEED_CPS;
        command.safety_generation = b2_safety_stop_generation();
        er = motion_send_command(&command);
        if (er == E_OK)
        {
            if (!issued)
                tm_printf((UB*)"[S5-B2] guarded DRIVE command accepted speed=%d cps; B2 owns PWM\n",
                          (int)S5_DEMO_SPEED_CPS);
            issued = true;
            /* An accepted command only means queued, not that the motors
             * physically turned. Periodic feedback distinguishes sensor
             * recovery from B2 encoder/motor-power problems. */
            if ((uint32_t)(time.lo - last_status_log_ms) >= 1000u)
            {
                last_status_log_ms = time.lo;
                tm_printf((UB*)"[S5-B2-STATUS] status=%d fault=%d L_cps=%d R_cps=%d\n",
                          (int)status.status, (int)status.fault,
                          (int)status.left_speed_cps, (int)status.right_speed_cps);
            }
        }
        else if (er != E_OBJ && er != E_TMOUT)
        {
            s5_demo_drive_fault_stop("B2 rejected DRIVE command");
        }
        /* B2 PID's DRIVE watchdog is 100 ms. Refresh well before that;
         * B5 clearance and B2's own safety gate are checked each cycle. */
        tk_dly_tsk(S5_DEMO_DRIVE_REFRESH_MS);
    }
}
#else
static void s5_demo_clear_stop(void) { b2_request_stop(); }
static void s5_demo_clear_publish(void) { }
#endif

#if !S5_MOTION_BRIDGE_ENABLED
/* Real sensor + servo. In optional forward-only demo mode, a B5 helper
 * submits guarded straight DRIVE commands to B2's public command API.
 * This does not implement autonomous bypass or B3 line recovery. */
static void stationary_profile_mode(void)
{
    unsigned samples = 0u, close_hits = 0u, clear_hits = 0u;
    unsigned moving_samples = 0u;
    bool sensor_unavailable = false;
    bool obstacle_latched = false;
    bool scanned_this_obstacle = false;
    bool driving = false;
    S5GuardStopHistory guard_history;
    bool startup_scan_pending = (S5_FORCE_STARTUP_SCAN != 0);
    tm_printf((UB*)"[S5-FW] B5-GUARDED-DRIVE-2026-11-04-V3; bounded trigger + timed sensor recovery + echo safety\n");
    tm_printf((UB*)"[S5] REAL ROBOT: clear -> forward; close -> STOP -> coarse/fine/profile\n");
    tm_printf((UB*)"[S5] SENSOR=REAL SERVO=REAL MOTORS=%s LINE=UNAVAILABLE\n",
              S5_DEMO_SENSOR_ONLY ? "DEMO STOP-ONLY" :
              (ROBOT_ENABLE_DRIVE ? "GUARDED FORWARD/STOP" : "DISABLED"));
    tm_printf((UB*)"[S5] STOP below %d cm; RESUME after %d clear samples >= %d cm\n",
              (int)ROBOT_STOP_CM, (int)ROBOT_CLEAR_CONFIRM,
              (int)ROBOT_RELEASE_CM);
#if S5_DEMO_B2_FORWARD_COMMANDS && !S5_DEMO_SENSOR_ONLY
    s5_demo_state_init();
#endif
    s5_demo_clear_stop();
    s5_guard_stop_history_init(&guard_history);
#if S5_DEMO_B2_FORWARD_COMMANDS && !S5_DEMO_SENSOR_ONLY
    {
        ID demo_tid = app_start_task((FP)s5_demo_drive_task,
                                     S5_PRI_MAIN, TP_PRC2,
                                     "s5_demo_drive_task");
        if (demo_tid < E_OK)
        {
            tm_printf((UB*)"[S5-B2-FAULT] cannot start guarded DRIVE task; STOP locked\n");
            for (;;) { s5_demo_clear_stop(); tk_dly_tsk(1000); }
        }
    }
#endif
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
    s5_demo_clear_stop();
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
#if ROBOT_NO_TARGET_FORWARD
    tm_printf((UB*)"[S5-POLICY] no target -> OPEN_SPACE_ASSUMED; not a measured distance; disconnected sensor may also allow drive\n");
#endif
    for (;;) {
        float cm = 0.0f;
        bool valid;
        bool scan_needed = false;
        /* While moving, perform ONE bounded forward ping per tick, not a
         * blocking 5-ping consensus. A successful clear ping refreshes
         * B2's 1200-ms safety heartbeat. This prevents false watchdog
         * expiries caused by the old slow 5/10-ping sampling loop.
         * Any invalid, suppressed, close or uncertain ping still causes
         * an immediate STOP; it is never treated as a clear path. */
        if (driving) {
#if S5_DEMO_B2_FORWARD_COMMANDS && !S5_DEMO_SENSOR_ONLY
            uint32_t last_clear;
            bool permitted;
            s5_demo_state_read(&last_clear, &permitted);
            (void)last_clear;
            /* The independent B2 refresh task may have revoked clearance
             * because the B5 task was delayed. Never re-arm from just one
             * moving ping: go back to stopped multi-echo consensus. */
            if (!permitted)
            {
                s5_demo_clear_stop();
                driving = false;
                clear_hits = 0u;
                tk_dly_tsk(S5_FRONT_POLL_MS);
                continue;
            }
            if (s5_demo_fault_latched())
            {
                s5_demo_clear_stop();
                driving = false;
                tk_dly_tsk(S5_FRONT_POLL_MS);
                continue;
            }
#endif
            S5EchoReading guard = s5_ultrasonic_read_front();
#if ROBOT_VERBOSE_SENSOR_LOG
            if ((moving_samples++ % 5u) == 0u)
                tm_printf((UB*)"[S5-FRONT] status=%s pulse_us=%d cm_x100=%d state=FORWARD\n",
                          s5_echo_status_name(guard.status),
                          (int)guard.pulse_us, (int)(guard.cm * 100.0f));
#else
            (void)moving_samples;
#endif
            bool guard_valid = (guard.status == S5_ECHO_OK ||
                                guard.status == S5_ECHO_OPEN_SPACE) &&
                               !s5_echo_suppressed_cm(guard.cm) &&
                               (s5_wire_cm(guard.cm,
                                           S5_WIRE_DISTANCE_MAX_CM) !=
                                COMM_CM_INVALID);
            /* True hysteresis: restart needs 45 cm; while already moving,
             * valid distances above 35 cm keep the current drive interval. */
            if (guard_valid && (guard.cm > ROBOT_STOP_CM)) {
                comm_set_obstacle_dist(s5_wire_cm(guard.cm,
                                                 S5_WIRE_DISTANCE_MAX_CM));
                fault_clear(FAULT_NO_ECHO);
                sensor_unavailable = false;
                close_hits = 0u;
                clear_hits = ROBOT_CLEAR_CONFIRM;
                /* B2 grants clearance only after its STOP generation
                 * has been acknowledged. Refresh on each valid ping. */
                b2_request_forward();
                if (b2_safety_motion_allowed()) s5_demo_clear_publish();
                if (!b2_safety_motion_allowed()) {
                    s5_demo_clear_stop();
                    driving = false;
                    tm_printf((UB*)"[S5-STOP] B2 motion clearance revoked; waiting for confirmed clear path\n");
                }
                tk_dly_tsk(S5_FRONT_POLL_MS);
                continue;
            }
            /* A missing or close echo cannot prove the path is clear.
             * Do not continue moving based on an old 310-cm reading. */
            s5_demo_clear_stop();
            driving = false;
            clear_hits = 0u;
            close_hits = (guard_valid && guard.cm <= ROBOT_STOP_CM) ? 1u : 0u;
            if (guard_valid) {
                comm_set_obstacle_dist(s5_wire_cm(guard.cm,
                                                 S5_WIRE_DISTANCE_MAX_CM));
                fault_clear(FAULT_NO_ECHO);
                sensor_unavailable = false;
            } else {
                comm_set_obstacle_dist(COMM_CM_INVALID);
                fault_raise(FAULT_NO_ECHO);
                if (!sensor_unavailable) {
                    (void)comm_report_error(ERR_S5_NO_ECHO,
                                            S5_FRONT_ANGLE_DETAIL);
                    sensor_unavailable = true;
                }
            }
            {
                SYSTIM stopped_at;
                tk_get_tim(&stopped_at);
                if (s5_guard_stop_history_record(&guard_history,
                                                 stopped_at.lo)) {
                    tm_printf((UB*)"[S5-HOLD] repeated unsafe/invalid readings: STOP for at least %d ms; require %d stable clear batches to re-arm\n",
                              (int)S5_GUARD_COOLDOWN_MS,
                              (int)S5_GUARD_REARM_CLEAR_BATCHES);
                }
            }
            tm_printf((UB*)"[S5-GUARD] STOP status=%s pulse_us=%d cm_x100=%d; rechecking while stationary\n",
                      s5_echo_status_name(guard.status),
                      (int)guard.pulse_us, (int)(guard.cm * 100.0f));
            tk_dly_tsk(S5_ECHO_GAP_MS);
            continue;
        }
        /* Only while stopped: use the slower multi-ping consensus to
         * confirm a clear path before permitting a new drive interval. */
        valid = s5_measure_front_cm(&cm);
        /* Defensive guard: a malformed successful measurement must
         * never clear NO_ECHO or release the motors. */
        valid = valid &&
                (s5_wire_cm(cm, S5_WIRE_DISTANCE_MAX_CM) != COMM_CM_INVALID);
        ++samples;
        if (!driving)
            tm_printf((UB*)"[S5-CHECK] stable=%d cm_x100=%d clear=%d/%d close=%d/%d locked=%d\n",
                      (int)valid, (int)(cm * 100.0f),
                      (int)clear_hits, (int)ROBOT_CLEAR_CONFIRM,
                      (int)close_hits, (int)S5_CONFIRM_HITS,
                      (int)guard_history.locked);
        /* B1 telemetry (non-blocking): front distance in cm, and the
         * shared no-echo fault bit */
        comm_set_obstacle_dist(valid ?
                               s5_wire_cm(cm, S5_WIRE_DISTANCE_MAX_CM) :
                               COMM_CM_INVALID);
        if (valid) {
            fault_clear(FAULT_NO_ECHO);
        } else {
            fault_raise(FAULT_NO_ECHO);
        }
        /* Recovery is never inferred from a timeout or a single echo.
         * Each accepted stationary sample already represents 3 agreeing
         * echoes from a batch of 5 (or its bounded retry). */
        if (valid && cm >= ROBOT_RELEASE_CM)
        {
            SYSTIM recovered_at;
            if (tk_get_tim(&recovered_at) == E_OK &&
                s5_guard_stop_history_clear(&guard_history,
                                             recovered_at.lo))
            {
                tm_printf((UB*)"[S5-REARM] echo recovered: cooldown and %d clear batches passed; forward can resume\n",
                          (int)S5_GUARD_REARM_CLEAR_BATCHES);
            }
        }
        else
        {
            s5_guard_stop_history_unclear(&guard_history);
        }
        if (!valid) {
            close_hits = 0u;
            clear_hits = 0u;
            s5_demo_clear_stop();
            if (driving) tm_printf((UB*)"[S5-STOP] invalid ultrasonic echo: motors disabled\n");
            driving = false;
            if (!sensor_unavailable)
            {
                /* Send the event only on the transition to no-echo;
                 * repeated failures still keep the shared fault raised. */
                (void)comm_report_error(ERR_S5_NO_ECHO,
                                        S5_FRONT_ANGLE_DETAIL);
                tm_printf((UB*)"[S5-STOP] no reliable distance; motors stopped until clear path confirmed\n");
                sensor_unavailable = true;
            }
        } else {
            sensor_unavailable = false;
            if (cm <= ROBOT_STOP_CM) {
                /* One close sample must stop the wheels. Three are needed to scan. */
                s5_demo_clear_stop();
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
#if S5_DEMO_SENSOR_ONLY
                    /* Sensor-only demo: never release B2 STOP. */
                    s5_demo_clear_stop();
                    driving = false;
#else
                    if (guard_history.locked
#if S5_DEMO_B2_FORWARD_COMMANDS && !S5_DEMO_SENSOR_ONLY
                        || s5_demo_fault_latched()
#endif
                        ) {
                        /* A burst of bad echoes requires a timed hold and
                         * sustained stationary verification, not a reboot. */
                        s5_demo_clear_stop();
                        driving = false;
                    } else {
                        b2_request_forward();
                        if (b2_safety_motion_allowed()) {
                            s5_demo_clear_publish();
                            tm_printf((UB*)"[S5-DRIVE] confirmed clear=%d cm; B2 permission granted\n",
                                      (int)cm);
                            driving = true;
                        } else {
                            s5_demo_clear_stop();
                            driving = false;
                        }
                    }
#endif
                } else {
                    s5_demo_clear_stop();
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
                s5_demo_clear_stop();
                if (driving) tm_printf((UB*)"[S5-STOP] distance in uncertain band; PWM=0 requested\n");
                driving = false;
#if ROBOT_VERBOSE_SENSOR_LOG
                tm_printf((UB*)"[S5-DETECT] sample=%d cm_x100=%d AMBIGUOUS state=STOP\n",
                          (int)samples, (int)(cm * 100.0f));
#endif
            }
        }
        if (obstacle_latched) {
            s5_demo_clear_stop();
            driving = false;
        }
        /* Scan only with PWM stopped and STOP acknowledged, including in
         * fixed-duty forward/STOP mode. Never drive from a side reading. */
        if (scan_needed || startup_scan_pending || S5_CONTINUOUS_SWEEP_TEST) {
            S5Scan coarse = {0}, fine = {0};
            S5Profile profile = {0};
            S5Side side = S5_SIDE_NONE;
            SYSTIM started, ended;
            unsigned elapsed;
            startup_scan_pending = false;
            close_hits = 0u;
            s5_demo_clear_stop();
            driving = false;
            tk_dly_tsk(ROBOT_STOP_SETTLE_MS);
            if (!b2_motor_is_stopped()) {
                (void)comm_report_error(ERR_S5_STOP_ACK, 0u);
                tm_printf((UB*)"[S5-FAULT] motor PWM stop not acknowledged; scan aborted\n");
                tk_dly_tsk(S5_FRONT_POLL_MS);
                continue;
            }
            scanned_this_obstacle = true;
            clear_hits = 0u;
            tk_get_tim(&started);
            tm_printf((UB*)"[S5-STOP] confirmed close obstacle; motor PWM=0 acknowledged\n");
            tm_printf((UB*)"[S5-COARSE] sweep=70,80,90,100,110 (real servo)\n");
            s5_coarse_scan(&coarse);
#if ROBOT_VERBOSE_SENSOR_LOG
            print_scan("COARSE", &coarse);
#endif
            side = s5_choose_side(&coarse);
            if (coarse.nearest_index < 0) {
                /* No profile can be inferred; still send an explicit
                 * invalid STOP event instead of silently omitting it. */
                (void)comm_report_error(ERR_S5_NO_ECHO,
                                        S5_FRONT_ANGLE_DETAIL);
                tm_printf((UB*)"[S5-FAULT] no valid coarse readings; cannot plan bypass\n");
            } else {
                uint16_t center = coarse.points[coarse.nearest_index].angle_deg;
                tm_printf((UB*)"[S5-FINE] center_deg=%d sweep=+/-10 deg in 5 deg steps (safe mount)\n", (int)center);
                s5_fine_scan(&fine, center);
#if ROBOT_VERBOSE_SENSOR_LOG
                print_scan("FINE", &fine);
#endif
                profile = s5_profile_from_scans(&coarse, &fine, side);
                if (profile.valid) {
                    side = s5_plan_bypass(&profile, &coarse);
                    tm_printf((UB*)"[S5-SIZE] class=%d (1=small 2=large 3=oversized) width_cm_x100=%d\n",
                              (int)profile.size, (int)(profile.estimated_width_cm * 100.0f));
                } else {
                    side = S5_SIDE_NONE;
                    tm_printf((UB*)"[S5-FAULT] width not bracketed by valid clear rays; no safe plan\n");
                }
                print_profile(&profile);
                /* One scan event is emitted below by s5_report_profile().
                 * That helper enforces B1's field ranges and STOP policy. */
                tm_printf((UB*)"[S5-PLAN] direction=%s width_valid=%d action=%s\n",
                          side == S5_SIDE_LEFT ? "LEFT" : side == S5_SIDE_RIGHT ? "RIGHT" : "NONE",
                          (int)profile.valid,
                          side == S5_SIDE_NONE ? "STOP/REASSESS" : "BYPASS_REQUEST_ONLY");
                tm_printf((UB*)"[S5-RECOVER] physical bypass NOT enabled: no calibrated encoder/IR bridge\n");
            }
            if (!profile.valid || (side == S5_SIDE_NONE))
            {
                /* Existing team code: S5_ERR_NO_CLEAR_ROUTE = 4.
                 * Do not repurpose the team's 0xA2 STOP_TIMEOUT code. */
                (void)comm_report_error(ERR_S5_NO_CLEAR_ROUTE, 0u);
            }
            s5_report_profile(&profile, side);
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
    bool stop_requested = false;
    bool sensor_unavailable = false;
    uint32_t last_log_ms = 0u;
    if (!s5_bridge_ready()) emergency_halt("Buddy 2/3 motion bridge unavailable");
    s5_controller_init(&c);
    tm_printf((UB*)"[S5] LIVE CONTROLLER GP16=TRIG GP17=ECHO GP12=SERVO\n");
    tm_printf((UB*)"[S5] B2 owns motors, B3 owns IR line, navigation arbitrates commands\n");
    for (;;) {
        S5Input in = {0};
        S5Command cmd;
        float cm = 0.0f;
        SYSTIM cycle_start = {0}, cycle_end;
        S5EchoReading reading;
        if (!s5_mission_enabled()) {
            s5_navigation_pause();
            s5_controller_init(&c);
            coarse_ready = fine_ready = stop_requested = false;
            s5_mission_publish(S5_MISSION_FOLLOW);
            tk_dly_tsk(S5_ECHO_GAP_MS);
            continue;
        }
        if (tk_get_tim(&cycle_start) != E_OK) emergency_halt("system clock unavailable");
        /* Sensor always faces forward outside scans. Monitoring continues
         * during movement. Repeated invalid readings fail closed. */
        /* Apply the same configured no-target policy in every heading.
         * Stuck-high, malformed and incomplete pulses remain faults. */
        reading = s5_ultrasonic_read_front();
        cm = reading.cm;
        in.front_valid = reading.status == S5_ECHO_OK ||
                         reading.status == S5_ECHO_OPEN_SPACE;
        in.front_valid = in.front_valid &&
                         (s5_wire_cm(cm, S5_WIRE_DISTANCE_MAX_CM) !=
                          COMM_CM_INVALID);
        in.front_cm = in.front_valid ? cm : 0.0f;
        comm_set_obstacle_dist(in.front_valid ?
                               s5_wire_cm(cm, S5_WIRE_DISTANCE_MAX_CM) :
                               COMM_CM_INVALID);
        if (cycle_start.lo - last_log_ms >= 1000u) {
            last_log_ms = cycle_start.lo;
            tm_printf((UB*)"[S5-LIVE] state=%s echo=%s cm_x100=%d nav_error=%s\n",
                      s5_state_name(c.state), s5_echo_status_name(reading.status),
                      (int)(cm * 100.0f), s5_error_name(s5_navigation_error()));
        }
        if (in.front_valid)
        {
            fault_clear(FAULT_NO_ECHO);
            sensor_unavailable = false;
        }
        else
        {
            fault_raise(FAULT_NO_ECHO);
            if (!sensor_unavailable)
            {
                (void)comm_report_error(ERR_S5_NO_ECHO,
                                        S5_FRONT_ANGLE_DETAIL);
                sensor_unavailable = true;
            }
        }
        s5_navigation_tick(in.front_valid, cm, c.state == S5_FOLLOW);
        if (s5_navigation_error() != S5_ERR_NONE) {
            (void)comm_report_error(ERR_S5(s5_navigation_error()), 0u);
            emergency_halt(s5_error_name(s5_navigation_error()));
        }
        /* Establish the original line before starting an avoidance cycle.
         * A robot placed off the line waits, even with an obstacle ahead. */
        if (c.state == S5_FOLLOW && !s5_navigation_has_line_reference()) {
            tk_dly_tsk(S5_ECHO_GAP_MS);
            continue;
        }
        in.stopped = s5_navigation_stopped();
        in.motion_complete = s5_bridge_motion_complete();
        in.line_found = s5_bridge_line_found();
        in.coarse_ready = coarse_ready;
        in.fine_ready = fine_ready;
        if (coarse_ready) in.coarse = coarse;
        if (fine_ready) in.fine = fine;
        coarse_ready = fine_ready = false;
        cmd = s5_controller_step(&c, &in);
        s5_mission_publish(c.state == S5_FOLLOW ? S5_MISSION_FOLLOW : S5_MISSION_BUSY);
        if (cmd != S5_CMD_NONE) {
            tm_printf((UB*)"[S5] state=%s command=%s error=%s\n",
                      s5_state_name(c.state), s5_command_name(cmd),
                      s5_error_name(c.error));
        }
        switch (cmd) {
        case S5_CMD_NONE: break;
        case S5_CMD_SCAN_COARSE:
            if (!s5_navigation_stopped()) emergency_halt("scan requires encoder-confirmed STOP");
            s5_coarse_scan(&coarse);
            print_scan("COARSE", &coarse);
            coarse_ready = true;
            break;
        case S5_CMD_SCAN_FINE:
            if (!s5_navigation_stopped()) emergency_halt("scan requires encoder-confirmed STOP");
            s5_fine_scan(&fine, c.fine_center_deg);
            print_scan("FINE", &fine);
            fine_ready = true;
            break;
        case S5_CMD_HALT:
            if ((c.error == S5_ERR_NO_CLEAR_ROUTE) ||
                (c.error == S5_ERR_FINE_SCAN) ||
                (c.error == S5_ERR_INSUFFICIENT_CLEARANCE))
            {
                s5_report_profile(&c.profile, S5_SIDE_NONE);
            }
            if (c.error != S5_ERR_NONE)
            {
                (void)comm_report_error(ERR_S5(c.error), 0u);
            }
            emergency_halt(s5_error_name(c.error));
            break;
        case S5_CMD_STOP:
            if (!stop_requested) {
                if (!s5_bridge_request(cmd, &c.profile)) {
                    (void)comm_report_error(ERR_S5_STOP_ACK, 0u);
                    emergency_halt("stop request rejected");
                }
                stop_requested = true;
            }
            break;
        case S5_CMD_GO_LEFT:
        case S5_CMD_GO_RIGHT:
            print_profile(&c.profile);
            s5_report_profile(&c.profile,
                              cmd == S5_CMD_GO_LEFT ? S5_SIDE_LEFT :
                              S5_SIDE_RIGHT);
            /* fall through */
        default:
            if (!s5_bridge_request(cmd, &c.profile)) {
                (void)comm_report_error(ERR_S5_NO_CLEAR_ROUTE, (UH)cmd);
                emergency_halt("motion or line command rejected");
            }
            if (cmd == S5_CMD_RESUME) {
                /* Resume is a completed handoff, not a permanent DONE state. */
                s5_controller_init(&c);
                stop_requested = false;
                s5_mission_publish(S5_MISSION_FOLLOW);
            }
            break;
        }
        /* 65 ms between trigger starts, rather than echo time PLUS 65 ms.
         * A 38 ms no-target pulse previously exceeded B2's 100 ms DRIVE
         * watchdog. Scans run stationary and retain their longer pacing. */
        if (tk_get_tim(&cycle_end) == E_OK &&
            cycle_end.lo - cycle_start.lo < S5_ECHO_GAP_MS)
            tk_dly_tsk(S5_ECHO_GAP_MS - (cycle_end.lo - cycle_start.lo));
        else
            tk_dly_tsk(1);
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
    tm_printf((UB*)"[FW-ID] 2026-11-06 B5-SIZE-AVOID-RECOVER: autonomous=%d\n", (int)ROBOT_ENABLE_BYPASS);
    tm_printf((UB*)"[S5-BOOT] Buddy 5 task running, hardware profile GP16/17/12\n");
    if (!s5_ultrasonic_init()) {
        s5_mission_publish(S5_MISSION_FAULT);
        b2_request_stop();
        (void)comm_report_error(ERR_S5_NO_ECHO, S5_FRONT_ANGLE_DETAIL);
        tm_printf((UB*)"[S5-FAULT] ultrasonic initialization failed; motors stopped\n");
        for (;;) tk_dly_tsk(1000);
    }
    tm_printf((UB*)"[S5-BOOT] ultrasonic pins initialized\n");
    if (!s5_servo_init()) {
        s5_mission_publish(S5_MISSION_FAULT);
        b2_request_stop();
        (void)comm_report_error(ERR_S5_FINE_SCAN, 0u);
        tm_printf((UB*)"[S5-FAULT] servo initialization failed; motors stopped\n");
        for (;;) tk_dly_tsk(1000);
    }
    tm_printf((UB*)"[S5-BOOT] servo PWM initialized at 50Hz\n");
    tk_dly_tsk(600);
#if S5_MOTION_BRIDGE_ENABLED
    /* Warm-up never releases the initial B2 STOP latch. Startup echoes are
     * discarded; the controller then requires fresh clearance to drive. */
    tk_dly_tsk(ROBOT_SENSOR_STARTUP_SETTLE_MS);
    for (unsigned i = 0u; i < ROBOT_SENSOR_WARMUP_PINGS; ++i) {
        (void)s5_ultrasonic_read_detailed();
        tk_dly_tsk(S5_ECHO_GAP_MS);
    }
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
