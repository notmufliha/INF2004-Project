/* s5_box.c - obstacle handling, "box" bypass (ROBOT_BOX_BYPASS).
 *
 * Following: the front sonar slows the car below S5B_SLOW_CM and stops it
 * at S5B_STOP_CM (two readings within S5B_AGREE_CM).  On a ramp (VC flag)
 * the stop distance is S5B_RAMP_STOP_CM: the beam sees the ramp face.
 *
 * Stopped: creep to S5B_SCAN_CM, scan (servo 70..110 deg), and plan:
 *   - nothing near any more            -> resume following
 *   - one narrow echo (< 28 mm wide)   -> a ramp glint: resume
 *   - lane on the side needing the smaller sideways shift:
 *       shift = robot half width + margin + obstacle edge offset
 *       forward leg = obstacle distance + assumed depth + tail
 *     too far (lateral > 30 cm, forward > 80 cm) -> stop (no path)
 * Bypass: turn 90 out, lateral leg, turn 90 back, pass leg, turn 45 in,
 * drive toward the line (expected after lateral x sqrt 2); accept the
 * first tape crossing 50..120 % of that with a 15..75 mm wide black run,
 * advance 60 mm, turn 45 back to the original heading, follow (slow for
 * 150 mm, VC).  20 s from the end of the pass leg to the line, or a side
 * echo under 6 cm, stops the car.
 */
#include "subsystem5.h"
#include "../robot_config.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/b2_motor.h"
#include "../subsystem2/motion_command.h"
#include "../subsystem3/b3_line_barcode.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "obstacle_scan.h"
#include "s5_mission.h"
#include "s5_box.h"
#include "s5_logic.h"

#if ROBOT_BOX_BYPASS && !S5_SIMULATION && S5_HARDWARE_ENABLED

#define S5B_SLOW_CM 60.0f
#define S5B_STOP_CM                                                            \
    40.1f /* 100 clearance + 60 offset + braking + 150 early                   \
           */
#define S5B_RAMP_STOP_CM 25.1f
#define S5B_AGREE_CM 3.0f
#define S5B_SCAN_CM 25.0f
#define S5B_CREEP_TOL_CM 3.0f
#define S5B_NEAR_CM 65.0f     /* scan point counts as obstacle */
#define S5B_CORE_BAND_CM 8.0f /* points this far behind the nearest */
#define S5B_HALF_W_CM 7.5f    /* robot half width */
#define S5B_BOX_MARGIN_CM 3.0f
#define S5B_DEPTH_CM 15.0f /* assumed obstacle depth */
#define S5B_TAIL_CM 10.0f  /* axle to rear + 20 mm */
#define S5B_LEG_MIN_CM 2.0f
#define S5B_LAT_MAX_CM 30.0f
#define S5B_FWD_MAX_CM 80.0f
#define S5B_WIDTH_MIN_CM 10.0f
#define S5B_WIDTH_MAX_CM 20.0f
#define S5B_GLINT_CM 2.8f
#define S5B_SIDE_CLOSE_CM 6.0f
#define S5B_REJOIN_DEG 45
#define S5B_REJOIN_WIDTH_CM 4.5f
#define S5B_REJOIN_TOL_CM 3.0f
#define S5B_REJOIN_LO_PCT 50
#define S5B_REJOIN_HI_PCT 120
#define S5B_ADVANCE_CM 6.0f
#define S5B_FR9_MS 20000u
#define S5B_MOVE_CPS 1786 /* 150 mm/s */
#define S5B_TURN_CPS 1786
#define S5B_CREEP_CPS 1190 /* 100 mm/s */
#define S5B_MOVE_TIMEOUT_MS 15000u
#define S5B_LINE_BLACK 700u
#define S5B_NO_ECHO_LIMIT 3u

typedef enum
{
    BX_FOLLOW,
    BX_STOPPING,
    BX_CREEP,
    BX_SCAN,
    BX_TURN_OUT,
    BX_LEG_OUT,
    BX_TURN_PAR,
    BX_LEG_PASS,
    BX_TURN_IN,
    BX_REJOIN,
    BX_ADVANCE,
    BX_HEADING,
    BX_FAULT
} BoxState;

static BoxState g_state;
static uint32_t g_move_id, g_move_ms, g_fr9_ms, g_state_ms;
static bool     g_left; /* bypass on the left */
static float    g_lat_cm, g_fwd_cm, g_e_cm;
static float    g_prev_cm;
static bool     g_prev_valid;
static unsigned g_clear_hits, g_bad_echo, g_startup_hits;
static bool     g_black, g_crept;
static float    g_black_start_cm;

static uint32_t
now_ms (void)
{
    SYSTIM t;
    return tk_get_tim(&t) == E_OK ? t.lo : 0u;
}

static UH
wire_cm (float cm)
{
    if (!__builtin_isfinite(cm) || cm < 0.0f)
    {
        return COMM_CM_INVALID;
    }
    if (cm > 400.0f)
    {
        cm = 400.0f;
    }
    return (UH)(cm + 0.5f);
}

static bool
stopped (void)
{
    motion_status_snapshot_t st;
    return motion_get_status(&st) == E_OK && st.physically_stopped &&
        b2_motor_is_stopped();
}

static void
halt (void)
{
    motion_command_t stop = {0};
    b2_request_stop();
    stop.type = MOTION_CMD_STOP;
    (void)motion_send_command(&stop);
    g_move_id = 0u;
}

static void
go (BoxState s)
{
    g_state    = s;
    g_state_ms = now_ms();
}

static void
fail (const char * why)
{
    halt();
    tm_printf((UB *)"[S5-BOX] STOP: %s\n", why);
    (void)comm_report_error(ERR_S5_NO_CLEAR_ROUTE, 0u);
    comm_set_robot_state(ROBOT_SAFE_STOP);
    s5_mission_publish(S5_MISSION_FAULT);
    go(BX_FAULT);
}

static void
resume (const char * why)
{
    tm_printf((UB *)"[S5-BOX] resume line following: %s\n", why);
    g_clear_hits = 0u;
    g_prev_valid = false;
    g_crept      = false;
    go(BX_FOLLOW);
    s5_mission_publish(S5_MISSION_FOLLOW);
}

/* Submit a move (cm > 0 forward) or a turn (deg != 0, + = right). */
static bool
submit (float cm, int deg, INT cps)
{
    ER err;
    b2_request_forward();
    if (!b2_safety_motion_allowed())
    {
        return false; /* STOP not acked yet */
    }
    if (deg != 0)
    {
        err = motion_turn_degrees(deg < 0 ? -deg : deg,
            cps,
            deg < 0 ? MOTION_TURN_LEFT : MOTION_TURN_RIGHT,
            &g_move_id);
    }
    else
    {
        err = motion_move_cm(cm, cps, MOTION_DIR_FORWARD, &g_move_id);
    }
    if (err != E_OK || g_move_id == 0u)
    {
        return false;
    }
    g_move_ms = now_ms();
    tm_printf((UB *)"[S5-BOX] state=%d move cm_x10=%d turn=%d\n",
        (int)g_state,
        (int)(cm * 10.0f),
        deg);
    return true;
}

/* 1 = finished OK, 0 = running, -1 = failed. */
static int
move_done (void)
{
    motion_status_snapshot_t res;
    if (g_move_id == 0u)
    {
        return 0;
    }
    b2_request_forward(); /* clearance heartbeat during the move */
    if (motion_get_result(g_move_id, &res) == E_OK)
    {
        g_move_id = 0u;
        return res.status == MOTION_STATUS_COMPLETED ? 1 : -1;
    }
    if (now_ms() - g_move_ms > S5B_MOVE_TIMEOUT_MS)
    {
        return -1;
    }
    return 0;
}

static float
progress_cm (void)
{
    motion_status_snapshot_t st;
    if (motion_get_status(&st) != E_OK)
    {
        return 0.0f;
    }
    return (st.left_distance_mm + st.right_distance_mm) / 20.0f;
}

/* Plan from the coarse scan.  Bearings: 70 = left .. 110 = right. */
static void
plan (const S5Scan * scan, bool on_ramp)
{
    float nearest = 1e9f, xl = 1e9f, xr = -1e9f, y_near = 0.0f;
    int   hits = 0, first = -1, last = -1;
    bool  clip_l, clip_r;
    float width, shift_l, shift_r;

    for (int i = 0; i < S5_SCAN_COUNT; ++i)
    {
        const S5ScanPoint * p = &scan->points[i];
        if (p->valid && p->distance_cm > 0.0f && p->distance_cm < nearest)
        {
            nearest = p->distance_cm;
        }
    }
    if (nearest > S5B_NEAR_CM)
    {
        resume("nothing within 65 cm after the scan");
        return;
    }
    for (int i = 0; i < S5_SCAN_COUNT; ++i)
    {
        const S5ScanPoint * p = &scan->points[i];
        float               th, x, y;
        if (!p->valid || p->distance_cm > nearest + S5B_CORE_BAND_CM ||
            p->distance_cm > S5B_NEAR_CM)
        {
            continue;
        }
        th = ((float)p->angle_deg - 90.0f) * 3.14159265f / 180.0f;
        x  = p->distance_cm * __builtin_sinf(th); /* + right */
        y  = p->distance_cm * __builtin_cosf(th);
        if (x < xl)
        {
            xl = x;
        }
        if (x > xr)
        {
            xr = x;
        }
        if (p->distance_cm <= nearest)
        {
            y_near = y;
        }
        if (first < 0)
        {
            first = i;
        }
        last = i;
        hits++;
    }
    width = xr - xl;
    tm_printf((UB *)"[S5-BOX] scan: nearest_cm_x10=%d hits=%d width_cm_x10=%d "
                    "ramp=%d\n",
        (int)(nearest * 10.0f),
        hits,
        (int)(width * 10.0f),
        (int)on_ramp);
    if (hits == 1 && width < S5B_GLINT_CM)
    {
        resume("single narrow echo: ramp glint, not an obstacle");
        return;
    }
    /* An edge ray still on the obstacle: the true edge is beyond it. */
    clip_l = first == 0;
    clip_r = last == S5_SCAN_COUNT - 1;
    if (width < S5B_WIDTH_MIN_CM)
    {
        float grow = S5B_WIDTH_MIN_CM - width;
        if (clip_l && !clip_r)
        {
            xl -= grow;
        }
        else if (clip_r && !clip_l)
        {
            xr += grow;
        }
        else
        {
            xl -= grow / 2.0f;
            xr += grow / 2.0f;
        }
    }
    if (clip_l && xr - xl < S5B_WIDTH_MAX_CM)
    {
        xl = xr - S5B_WIDTH_MAX_CM;
    }
    if (clip_r && xr - xl < S5B_WIDTH_MAX_CM)
    {
        xr = xl + S5B_WIDTH_MAX_CM;
    }
    shift_l = S5B_HALF_W_CM + S5B_BOX_MARGIN_CM - xl; /* xl < 0 on left */
    shift_r = S5B_HALF_W_CM + S5B_BOX_MARGIN_CM + xr;
    if (shift_l < 0.0f)
    {
        shift_l = 0.0f;
    }
    if (shift_r < 0.0f)
    {
        shift_r = 0.0f;
    }
    g_left   = shift_l <= shift_r;
    g_lat_cm = g_left ? shift_l : shift_r;
    if (g_lat_cm < S5B_LEG_MIN_CM)
    {
        g_lat_cm = S5B_LEG_MIN_CM;
    }
    g_fwd_cm = y_near + S5B_DEPTH_CM + S5B_TAIL_CM;
    tm_printf((UB *)"[S5-BOX] plan: side=%s lateral_cm_x10=%d "
                    "forward_cm_x10=%d (edges x10 %d..%d)\n",
        g_left ? "LEFT" : "RIGHT",
        (int)(g_lat_cm * 10.0f),
        (int)(g_fwd_cm * 10.0f),
        (int)(xl * 10.0f),
        (int)(xr * 10.0f));
    if (g_lat_cm > S5B_LAT_MAX_CM || g_fwd_cm > S5B_FWD_MAX_CM)
    {
        fail("no path: bypass legs over the limit");
        return;
    }
    comm_set_robot_state(ROBOT_OBSTACLE_AVOID);
    go(BX_TURN_OUT);
}

static bool
line_black (void)
{
    B3_STATE line;
    uint32_t sample = 0u;
    if (b3_get_timed_state(&line, &sample) != E_OK)
    {
        return false;
    }
    return line.ir_left >= S5B_LINE_BLACK || line.ir_centre >= S5B_LINE_BLACK;
}

static void
bypass_step (float front_cm, bool front_valid)
{
    int d;

    if (front_valid && front_cm > 0.0f && front_cm < S5B_SIDE_CLOSE_CM &&
        g_state != BX_REJOIN && g_state != BX_ADVANCE)
    {
        fail("echo under 6 cm during the bypass");
        return;
    }
    if (g_fr9_ms != 0u && now_ms() - g_fr9_ms > S5B_FR9_MS)
    {
        fail("line not rejoined within 20 s of the pass leg");
        return;
    }
    if (g_move_id == 0u)
    {
        /* Start this state's motion. */
        bool ok = true;
        switch (g_state)
        {
            case BX_TURN_OUT:
                ok = submit(0.0f, g_left ? -90 : 90, S5B_TURN_CPS);
            break;
            case BX_LEG_OUT:
                ok = submit(g_lat_cm, 0, S5B_MOVE_CPS);
            break;
            case BX_TURN_PAR:
                ok = submit(0.0f, g_left ? 90 : -90, S5B_TURN_CPS);
            break;
            case BX_LEG_PASS:
                ok = submit(g_fwd_cm, 0, S5B_MOVE_CPS);
            break;
            case BX_TURN_IN:
                ok = submit(0.0f,
                    g_left ? S5B_REJOIN_DEG : -S5B_REJOIN_DEG,
                    S5B_TURN_CPS);
            break;
            case BX_REJOIN:
                g_e_cm = g_lat_cm * 1.41421356f;
                tm_printf((UB *)"[S5-BOX] rejoin: line expected at %d mm "
                                "(accept %d..%d)\n",
                    (int)(g_e_cm * 10.0f),
                    (int)(g_e_cm * S5B_REJOIN_LO_PCT / 10.0f),
                    (int)(g_e_cm * S5B_REJOIN_HI_PCT / 10.0f));
                g_black = false;
                ok      = submit(
                    g_e_cm * S5B_REJOIN_HI_PCT / 100.0f, 0, S5B_MOVE_CPS);
            break;
            case BX_ADVANCE:
                ok = submit(S5B_ADVANCE_CM, 0, S5B_MOVE_CPS);
            break;
            case BX_HEADING:
                ok = submit(0.0f,
                    g_left ? -S5B_REJOIN_DEG : S5B_REJOIN_DEG,
                    S5B_TURN_CPS);
            break;
            default:
            break;
        }
        if (!ok && now_ms() - g_state_ms > 2000u)
        {
            fail("bypass move rejected");
        }
        return;
    }
    if (g_state == BX_REJOIN)
    {
        /* Measure the black run while driving toward the line. */
        bool  black = line_black();
        float done  = progress_cm();
        if (black && !g_black)
        {
            g_black          = true;
            g_black_start_cm = done;
        }
        else if (!black && g_black)
        {
            float width = done - g_black_start_cm;
            g_black     = false;
            tm_printf((UB *)"[S5-BOX] rejoin: tape at %d mm, %d mm wide\n",
                (int)(g_black_start_cm * 10.0f),
                (int)(width * 10.0f));
            if (g_black_start_cm >= g_e_cm * S5B_REJOIN_LO_PCT / 100.0f &&
                width + S5B_REJOIN_TOL_CM >= S5B_REJOIN_WIDTH_CM &&
                width <= S5B_REJOIN_WIDTH_CM + S5B_REJOIN_TOL_CM)
            {
                tm_printf((UB *)"[S5-BOX] rejoin accepted\n");
                halt();
                go(BX_ADVANCE);
                return;
            }
        }
    }
    d = move_done();
    if (d == 0)
    {
        return;
    }
    if (d < 0)
    {
        fail("bypass move failed");
        return;
    }
    switch (g_state)
    {
        case BX_TURN_OUT:
            go(BX_LEG_OUT);
        break;
        case BX_LEG_OUT:
            go(BX_TURN_PAR);
        break;
        case BX_TURN_PAR:
            go(BX_LEG_PASS);
        break;
        case BX_LEG_PASS:
            g_fr9_ms = now_ms() | 1u;
            go(BX_TURN_IN);
        break;
        case BX_TURN_IN:
            go(BX_REJOIN);
        break;
        case BX_REJOIN:
            fail("drove the whole rejoin window, no line");
        break;
        case BX_ADVANCE:
            go(BX_HEADING);
        break;
        case BX_HEADING:
            tm_printf(
                (UB *)"[S5-BOX] bypass done in %d ms after the pass leg\n",
                (int)(now_ms() - g_fr9_ms));
            g_fr9_ms = 0u;
            resume("bypass complete");
        break;
        default:
        break;
    }
}

void
s5_box_mode (void)
{
    tm_printf((UB *)"[S5-BOX] obstacle mode: slow %d cm, stop %d cm (ramp %d), "
                    "scan at %d cm, box bypass\n",
        (int)S5B_SLOW_CM,
        (int)S5B_STOP_CM,
        (int)S5B_RAMP_STOP_CM,
        (int)S5B_SCAN_CM);
    g_state = BX_FOLLOW;
    s5_mission_publish(S5_MISSION_FOLLOW);
    for (;;)
    {
        uint32_t      t0      = now_ms();
        S5EchoReading r       = s5_ultrasonic_read_front();
        bool          open    = r.status == S5_ECHO_OPEN_SPACE;
        bool          valid   = r.status == S5_ECHO_OK || open;
        float         cm      = open ? 1000.0f : r.cm;
        bool          ramp    = s5_mission_ramp();
        float         stop_cm = ramp ? S5B_RAMP_STOP_CM : S5B_STOP_CM;

        s5_mission_front(valid, open ? 999.0f : cm);
        comm_set_obstacle_dist(open ? COMM_FRONT_OPEN_SPACE_CM
                : valid             ? wire_cm(cm)
                                    : COMM_CM_INVALID);
        if (valid)
        {
            g_bad_echo = 0u;
            fault_clear(FAULT_NO_ECHO);
        }
        else if (g_bad_echo < S5B_NO_ECHO_LIMIT)
        {
            g_bad_echo++;
        }
        if (g_startup_hits < ROBOT_CLEAR_CONFIRM)
        {
            g_startup_hits = valid ? g_startup_hits + 1u : 0u;
            b2_request_stop();
            tk_dly_tsk(S5_ECHO_GAP_MS);
            continue;
        }
        if (!s5_mission_enabled())
        {
            if (g_state != BX_FOLLOW || g_move_id != 0u)
            {
                halt();
            }
            g_state      = BX_FOLLOW;
            g_fr9_ms     = 0u;
            g_clear_hits = 0u;
            g_crept      = false;
            s5_mission_publish(S5_MISSION_FOLLOW);
            tk_dly_tsk(S5_ECHO_GAP_MS);
            continue;
        }
        if (g_bad_echo >= S5B_NO_ECHO_LIMIT && g_state == BX_FOLLOW)
        {
            fault_raise(FAULT_NO_ECHO);
            b2_request_stop();
            g_clear_hits = 0u;
        }

        switch (g_state)
        {
            case BX_FOLLOW:
                {
                    bool near  = valid && !open && cm <= stop_cm;
                    bool agree = near && g_prev_valid &&
                        (cm - g_prev_cm < S5B_AGREE_CM &&
                            g_prev_cm - cm < S5B_AGREE_CM);
                    g_prev_valid = near;
                    g_prev_cm    = cm;
                    if (agree)
                    {
                        halt();
                        g_clear_hits = 0u;
                        tm_printf((UB *)"[S5-BOX] obstacle at %d mm: stop\n",
                            (int)(cm * 10.0f));
                        comm_set_robot_state(ROBOT_OBSTACLE_SCAN);
                        s5_mission_publish(S5_MISSION_BUSY);
                        go(BX_STOPPING);
            break;
                    }
                    if (valid && !near)
                    {
                        if (g_clear_hits < ROBOT_CLEAR_CONFIRM)
                        {
                            g_clear_hits++;
                        }
                        if (g_clear_hits >= ROBOT_CLEAR_CONFIRM)
                        {
                            b2_request_forward();
                        }
                    }
                    else if (!valid && g_bad_echo < S5B_NO_ECHO_LIMIT &&
                        g_clear_hits >= ROBOT_CLEAR_CONFIRM)
                    {
                        b2_request_forward(); /* one dropped echo: keep going */
                    }
            break;
                }
            case BX_STOPPING:
                if (!stopped())
                {
                    if (now_ms() - g_state_ms > 3000u)
                    {
                        fail("did not stop");
                    }
            break;
                }
                if (!g_crept && valid && !open &&
                    cm > S5B_SCAN_CM + S5B_CREEP_TOL_CM)
                {
                    g_crept = true;
                    tm_printf((UB *)"[S5-BOX] creep %d mm closer to scan\n",
                        (int)((cm - S5B_SCAN_CM) * 10.0f));
                    if (submit(cm - S5B_SCAN_CM, 0, S5B_CREEP_CPS))
                    {
                        go(BX_CREEP);
                    }
                    else
                    {
                        g_crept = false;
                    }
                }
                else
                {
                    go(BX_SCAN);
                }
            break;
            case BX_CREEP:
                {
                    int d = move_done();
                    if (valid && !open && cm <= S5B_SCAN_CM)
                    {
                        halt();
                        d = 1;
                    }
                    if (d != 0)
                    {
                        go(BX_STOPPING); /* stopped: then scan */
                    }
                    if (d != 0)
                    {
                        g_crept = true;
                    }
            break;
                }
            case BX_SCAN:
                {
                    S5Scan scan;
                    if (!stopped())
                    {
            break;
                    }
                    b2_request_stop();
                    s5_coarse_scan(&scan);
                    plan(&scan, ramp);
            break;
                }
            case BX_FAULT:
                b2_request_stop();
            break;
            default:
                bypass_step(cm, valid && !open);
            break;
        }
        {
            uint32_t used = now_ms() - t0;
            tk_dly_tsk(used < S5_ECHO_GAP_MS ? S5_ECHO_GAP_MS - used : 1);
        }
    }
}
#endif

/*** end of file ***/

