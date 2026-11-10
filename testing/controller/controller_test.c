#include "mock_platform.h"
#include "../../app_program/vehicle_controller.c"
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <setjmp.h>
#include <string.h>
#include "../../app_program/subsystem3/b3_line_cal.c"

static motion_command_t sent;
static UINT drives, stops, resets;
static ER send_error;
static motion_status_snapshot_t result;
static bool result_ready;
static ROBOT_STATE published;
static bool permission = true;
static S5MissionState obstacle_state = S5_MISSION_FOLLOW;
S5MissionState s5_mission_state(void) { return obstacle_state; }
void s5_mission_enable(bool allow) { (void)allow; }
static uint32_t generation = 1u;
static VC_INPUT input;
static REMOTE_CMD remote_queue[8];
static UINT remote_head, remote_count, task_ticks;
static uint32_t clock_ms;
static bool stale_line, stale_imu, delayed_startup;
static uint32_t snapshot_delay;
static bool calibrated_startup;
static B3_LINE_CAL startup_right_cal, startup_left_cal;
static UINT first_calibrated_drive_tick;
static jmp_buf task_done;
ID mbf_remote_cmd = 1, mbf_nav_cmd = 2;

INT tm_printf(UB *format, ...) { (void)format; return 0; }
void comm_set_robot_state(ROBOT_STATE state) { published = state; }
ER subsystem4_reset_peak(void) { ++resets; return E_OK; }
void b2_request_stop(void) { permission = false; ++generation; }
bool b2_safety_motion_allowed(void) { return permission; }
uint32_t b2_safety_stop_generation(void) { return generation; }
UB fault_get(void) { return 0u; }
ER motion_speed_from_percent(INT percent, INT *speed) { *speed = percent * 50; return E_OK; }
ER motion_send_command(const motion_command_t *command)
{
    if (send_error != E_OK) return send_error;
    sent = *command;
    if (command->type == MOTION_CMD_DRIVE) {
        /* Every DRIVE: no wheel in the stall zone, outer within B2's limit. */
        INT s = command->steering_permille;
        INT left = command->target_speed_cps * (1000 + s) / 1000;
        INT right = command->target_speed_cps * (1000 - s) / 1000;
        assert(left == 0 || left >= VC_MIN_WHEEL_CPS - 1);
        assert(right == 0 || right >= VC_MIN_WHEEL_CPS - 1);
        assert(left <= 5000 && right <= 5000 && (left > 0 || right > 0));
        if (calibrated_startup && drives == 0u) first_calibrated_drive_tick = task_ticks;
        ++drives;
    }
    if (command->type == MOTION_CMD_STOP) ++stops;
    return E_OK;
}
ER motion_move_cm(float cm, INT speed, motion_direction_t dir, uint32_t *id)
{
    assert(cm == 10.0f && speed > 0 && dir == MOTION_DIR_FORWARD);
    *id = 42u; return E_OK;
}
ER motion_turn_degrees(INT angle, INT speed, motion_turn_direction_t dir, uint32_t *id)
{
    assert((angle == 90 || angle == 180) && speed > 0);
    sent.type = MOTION_CMD_TURN;
    sent.target_angle_deg = angle;
    sent.turn_direction = dir;
    *id = 42u; return E_OK;
}
ER motion_get_result(uint32_t id, motion_status_snapshot_t *snapshot)
{
    assert(id == 42u);
    if (!result_ready) return E_OBJ;
    *snapshot = result; return E_OK;
}
ER motion_get_status(motion_status_snapshot_t *snapshot)
{
    *snapshot = input.motion;
    snapshot->sample_time_ms = clock_ms;
    return E_OK;
}
ER tk_get_tim(SYSTIM *time) { time->lo = clock_ms; return E_OK; }
ER tk_dly_tsk(INT delay)
{
    clock_ms += (uint32_t)delay;
    permission = true; /* Simulate fresh B5 authorization after STOP. */
    if (++task_ticks == (calibrated_startup ? 8u : 4u)) longjmp(task_done, 1);
    return E_OK;
}
INT tk_rcv_mbf(ID id, void *data, TMO timeout)
{
    (void)timeout;
    if (id != mbf_remote_cmd || remote_head == remote_count) return E_TMOUT;
    memcpy(data, &remote_queue[remote_head++], sizeof(REMOTE_CMD));
    return sizeof(REMOTE_CMD);
}
bool b3_barcode_active(void) { return false; }
static UINT recal_requests;
void b3_request_recalibration(void) { ++recal_requests; }
ER b3_get_timed_state(B3_STATE *state, uint32_t *sample_ms)
{
    clock_ms += snapshot_delay;
    *state = (B3_STATE){0};
    state->line_flags = LINE_F_DETECTED;
    if (calibrated_startup) {
        /* Real low-output calibration feeds the real autostart adapter. */
        state->ir_left = b3_line_cal_update(&startup_right_cal, task_ticks < 3u ? 140u : 260u);
        state->ir_centre = b3_line_cal_update(&startup_left_cal, task_ticks < 3u ? 120u : 220u);
        state->line_flags = B3_LINE_LEFT_ACTIVE(state->ir_left) ||
            B3_LINE_CENTRE_ACTIVE(state->ir_centre) ? LINE_F_DETECTED : LINE_F_LOST;
    }
    *sample_ms = (stale_line || (delayed_startup && task_ticks < 2u)) ? clock_ms - 101u : clock_ms;
    return E_OK;
}
ER imu_vc_receive(imu_notice_t *notice, TMO timeout)
{
    static uint32_t last_tick = UINT32_MAX;
    (void)timeout;
    if (last_tick == clock_ms) return E_TMOUT;
    last_tick = clock_ms;
    *notice = (imu_notice_t){0};
    notice->snapshot.b_valid = true;
    notice->snapshot.timestamp_ms = stale_imu ? clock_ms - 1501u : clock_ms;
    return E_OK;
}
ID app_start_task(FP entry, INT priority, UW core, const char *name)
{
    (void)entry; (void)name;
    assert(priority == 6 && core == TP_PRC1);
    return 10;
}

static VEHICLE_CONTROLLER fresh(void)
{
    VEHICLE_CONTROLLER c;
    send_error = E_OK; result_ready = false; drives = stops = 0u;
    input = (VC_INPUT){0};
    input.now_ms = 100u; input.clearance = input.line_valid = input.imu_valid = true;
    input.motion_valid = true;
    input.motion.physically_stopped = true;
    input.line_flags = LINE_F_DETECTED;
    input.left = 900u; input.centre = 300u;
    input.safety_generation = generation;
    vehicle_controller_init(&c);
    return c;
}
static void command(VEHICLE_CONTROLLER *c, UINT type, UINT arg)
{
    REMOTE_CMD remote = {0}; remote.type = type; remote.arg = arg;
    vehicle_controller_command(c, &remote);
}
static void step(VEHICLE_CONTROLLER *c, UINT elapsed, UB nav)
{
    input.now_ms += elapsed;
    vehicle_controller_step(c, &input, nav);
}
static void settle(VEHICLE_CONTROLLER *c, UINT ticks)
{
    for (UINT i = 0u; i < ticks; ++i) step(c, 20u, 0u);
}
/* Wheel targets B2 derives from the last DRIVE: v(1+s) and v(1-s). */
static INT left_wheel(void)
{
    return sent.target_speed_cps * (1000 + sent.steering_permille) / 1000;
}
static INT right_wheel(void)
{
    return sent.target_speed_cps * (1000 - sent.steering_permille) / 1000;
}
static bool wheel_ok(INT wheel)
{
    /* Never in the stall zone: stopped, or at least the sustainable minimum. */
    return wheel == 0 || wheel >= VC_MIN_WHEEL_CPS - 1;
}
static void confirm_junction(VEHICLE_CONTROLLER *c)
{
    input.line_flags = LINE_F_DETECTED | LINE_F_JUNCTION;
    step(c, 20u, 0u);
    step(c, 20u, 0u);
    step(c, 20u, 0u);
}

int main(void)
{
    VEHICLE_CONTROLLER c = fresh();
    step(&c, 20u, 0u); assert(c.state == ROBOT_IDLE && drives == 0u);
    command(&c, RCMD_START, 0u);
    for (UINT i = 0u; i < 50u; ++i) step(&c, 20u, 0u);
    assert(drives == 50u && c.state == ROBOT_LINE_FOLLOW);
    step(&c, 1u, 0u); assert(drives == 50u);
    command(&c, RCMD_PAUSE, 0u); step(&c, 20u, 0u);
    assert(c.paused && drives == 50u && c.state == ROBOT_IDLE);
    command(&c, RCMD_RESUME, 0u); step(&c, 20u, 0u); assert(drives == 51u);
    command(&c, RCMD_ESTOP, 0u); command(&c, RCMD_RESUME, 0u);
    step(&c, 20u, 0u); assert(c.estop && drives == 51u);
    command(&c, RCMD_START, 0u); step(&c, 20u, 0u); assert(drives == 52u);
    command(&c, RCMD_STOP, 0u); command(&c, RCMD_RESUME, 0u);
    assert(!c.running);
    command(&c, RCMD_MODE, MODE_TEST); command(&c, RCMD_START, 0u);
    assert(!c.running && c.mode == MODE_TEST);
    command(&c, RCMD_MODE, MODE_LINE_ONLY); command(&c, RCMD_START, 0u);
    step(&c, 20u, B3_NAV_LEFT); assert(c.navigation == 0u);
    command(&c, RCMD_SPEED, 100u); input.left = 900u; input.centre = 30u;
    settle(&c, 30u);
    assert(sent.steering_permille == B3_LINE_STEER_MAX &&
        left_wheel() <= 5000 && wheel_ok(right_wheel()));
    input.left = 30u; input.centre = 900u;
    step(&c, 20u, 0u); /* steering slews, it does not jump */
    assert(sent.steering_permille == B3_LINE_STEER_MAX - VC_STEER_SLEW_PER_TICK);
    settle(&c, 30u);
    assert(sent.steering_permille == -B3_LINE_STEER_MAX &&
        right_wheel() <= 5000 && wheel_ok(left_wheel()));
    input.left = 700u; input.centre = 600u;
    settle(&c, 30u);
    assert(sent.steering_permille > 0);
    input.left = 700u; input.centre = 700u;
    settle(&c, 30u);
    assert(sent.steering_permille == 0);
    /* Pololu thresholds: on the line above 200. */
    assert(!B3_LINE_LEFT_ACTIVE(200u) && !B3_LINE_CENTRE_ACTIVE(200u));
    assert(B3_LINE_LEFT_ACTIVE(201u) && B3_LINE_CENTRE_ACTIVE(201u));
    /* Pololu position: weighted average, noise (<= 50) ignored, and the
     * last side held (0 or 1000) once neither sensor sees the line. */
    {
        VEHICLE_CONTROLLER p = {0};
        VC_INPUT v = {0};
        v.left = 1000u; v.centre = 1000u;
        assert(line_position(&p, &v) == B3_LINE_POS_CENTRE);
        v.left = 600u; v.centre = 200u;
        assert(line_position(&p, &v) == 750);
        v.left = 900u; v.centre = 50u;
        assert(line_position(&p, &v) == 1000);
        v.left = 40u; v.centre = 100u;  /* lost: line was to the right */
        assert(line_position(&p, &v) == 1000 && p.line_pos == 1000);
        v.left = 0u; v.centre = 700u;
        assert(line_position(&p, &v) == 0);
        v.left = 0u; v.centre = 0u;     /* lost: line was to the left */
        assert(line_position(&p, &v) == 0 && p.last_line_search_direction == -1);
        /* Hysteresis: centred (both on black) keeps the last clear side;
         * it used to flip to "right" on every centred sample. */
        v.left = 1000u; v.centre = 1000u;
        assert(line_position(&p, &v) == B3_LINE_POS_CENTRE &&
            p.last_line_search_direction == -1);
        v.left = 560u; v.centre = 440u; /* slightly right, inside the band */
        (void)line_position(&p, &v);
        assert(p.last_line_search_direction == -1);
        v.left = 0u; v.centre = 0u;     /* lost after centring: still left */
        assert(line_position(&p, &v) == 0);
        v.left = 1000u; v.centre = 300u; /* clearly right */
        (void)line_position(&p, &v);
        assert(p.last_line_search_direction == 1);
    }
    assert(B3_LINE_LEFT_ACTIVE(1000u) && !B3_LINE_LEFT_ACTIVE(1001u));
    /* Edge steering for this robot (both sensors on black when centred):
     * nothing while both read black, full correction as soon as one
     * sensor is ~40% onto the white, in the right direction. */
    {
        VEHICLE_CONTROLLER p = {0};
        VC_INPUT v = {0};
        v.left = 1000u; v.centre = 1000u;          /* centred */
        assert(line_follow_steering(&p, &v) == 0);
        v.left = 1000u; v.centre = 970u;           /* noise: deadband */
        assert(line_follow_steering(&p, &v) == 0);
        v.left = 1000u; v.centre = 780u;           /* left sensor at edge */
        assert(line_follow_steering(&p, &v) == 400);
        v.left = 1000u; v.centre = 500u;           /* left half off: full */
        assert(line_follow_steering(&p, &v) == B3_LINE_STEER_MAX);
        assert(p.last_line_search_direction == 1); /* line is to the right */
        v.left = 500u; v.centre = 1000u;           /* right half off */
        assert(line_follow_steering(&p, &v) == -B3_LINE_STEER_MAX);
        assert(p.last_line_search_direction == -1);
        v.left = 0u; v.centre = 1000u;
        assert(line_follow_steering(&p, &v) == -B3_LINE_STEER_MAX);
    }
    input.hump = true; step(&c, 20u, 0u);
    assert(c.state == ROBOT_HUMP && sent.target_speed_cps <= 2500);
    command(&c, RCMD_SPEED, 0u); step(&c, 20u, 0u);
    assert(c.state == ROBOT_IDLE);
    command(&c, RCMD_RESET_PEAK, 0u); assert(resets == 1u);

    c = fresh(); command(&c, RCMD_START, 0u); step(&c, 20u, 0u);
    input.clearance = false; step(&c, 20u, 0u);
    assert(c.state == ROBOT_OBSTACLE_SCAN && c.running);
    input.clearance = true; step(&c, 20u, 0u); assert(c.state == ROBOT_LINE_FOLLOW);
    input.faults = FAULT_WIFI_LOSS; step(&c, 20u, 0u); assert(c.running);
    input.left = 900u; input.centre = 100u;
    step(&c, 20u, 0u);
    assert(c.last_line_search_direction == 1);
    input.line_flags = LINE_F_LOST; step(&c, 20u, 0u);
    assert(c.state == ROBOT_LINE_RECOVERY && sent.type == MOTION_CMD_DRIVE &&
        sent.steering_permille >= B3_LINE_STEER_MAX && right_wheel() == 0 &&
        sent.target_speed_cps < 2500);
    step(&c, VC_LINE_SEARCH_SWITCH_MS, 0u); /* still turning back right */
    assert(sent.steering_permille >= B3_LINE_STEER_MAX);
    step(&c, VC_LINE_REACQUIRE_MS - VC_LINE_SEARCH_SWITCH_MS, 0u);
    /* not found there: sweep the other way */
    assert(sent.steering_permille == -VC_LINE_SEARCH_STEER);
    input.line_flags = LINE_F_DETECTED;
    step(&c, 20u, 0u);
    assert(c.state == ROBOT_LINE_FOLLOW && c.running);
    /* Hard turn back right for VC_LINE_REACQUIRE_MS, then widening
     * sweeps: 600 ms left, 1200 ms right, 1800 ms left. */
    input.line_flags = LINE_F_LOST;
    step(&c, 20u, 0u);
    assert(c.state == ROBOT_LINE_RECOVERY &&
        sent.steering_permille >= B3_LINE_STEER_MAX);
    step(&c, VC_LINE_REACQUIRE_MS, 0u);
    assert(sent.steering_permille == -VC_LINE_SEARCH_STEER);
    step(&c, VC_LINE_SEARCH_SWITCH_MS, 0u);
    assert(sent.steering_permille == VC_LINE_SEARCH_STEER);
    step(&c, VC_LINE_SEARCH_SWITCH_MS, 0u); /* 1200 ms segment: still right */
    assert(sent.steering_permille == VC_LINE_SEARCH_STEER);
    step(&c, VC_LINE_SEARCH_SWITCH_MS, 0u); /* then back left */
    assert(sent.steering_permille == -VC_LINE_SEARCH_STEER);
    step(&c, VC_RECOVERY_MS, 0u); /* old 3 s limit no longer disarms */
    assert(c.state == ROBOT_LINE_RECOVERY && c.running);
    /* Search exhausted: stopped but still armed. */
    step(&c, VC_LINE_SEARCH_MS, 0u);
    assert(c.state == ROBOT_LINE_RECOVERY && c.running &&
        sent.type == MOTION_CMD_STOP);
    UINT held_drives = drives;
    step(&c, 1000u, 0u); step(&c, 1000u, 0u);
    assert(drives == held_drives && c.running);
    /* Line back under the car: follows again without a new START. */
    input.line_flags = LINE_F_DETECTED;
    step(&c, 20u, 0u);
    assert(c.state == ROBOT_LINE_FOLLOW && c.running &&
        sent.type == MOTION_CMD_DRIVE);
    /* Dashboard STOP while holding still disarms. */
    input.line_flags = LINE_F_LOST;
    step(&c, 20u, 0u); step(&c, VC_LINE_SEARCH_MS, 0u);
    command(&c, RCMD_STOP, 0u); step(&c, 20u, 0u);
    assert(!c.running);

    c = fresh(); command(&c, RCMD_START, 0u);
    input.line_flags = LINE_F_LOST;
    step(&c, 20u, 0u);
    input.clearance = false; step(&c, 20u, 0u);
    assert(c.state == ROBOT_OBSTACLE_SCAN && c.running && stops > 0u);

    c = fresh(); command(&c, RCMD_START, 0u);
    input.left = 100u; input.centre = 900u;
    step(&c, 20u, 0u);
    assert(c.last_line_search_direction == -1);
    input.line_flags = LINE_F_LOST;
    step(&c, 20u, 0u);
    assert(c.state == ROBOT_LINE_RECOVERY &&
        sent.steering_permille <= -B3_LINE_STEER_MAX && left_wheel() == 0);

    /* Curve with BOTH sensors on the line (no junction flag): the car
     * must steer, not drive straight, and slow down for the curve. */
    c = fresh(); command(&c, RCMD_START, 0u);
    input.line_flags = LINE_F_DETECTED;
    input.left = 700u; input.centre = 700u;
    step(&c, 20u, 0u); /* acceleration from standstill is ramped */
    assert(sent.target_speed_cps == VC_MIN_WHEEL_CPS);
    settle(&c, 30u);
    INT straight_speed = sent.target_speed_cps;
    assert(sent.steering_permille == 0 && straight_speed == (INT)VC_DEFAULT_SPEED_PERCENT * 50);
    /* The wheel-minimum cases below are defined at 20% (dashboard SPEED). */
    command(&c, RCMD_SPEED, 20u); settle(&c, 30u);
    straight_speed = sent.target_speed_cps;
    assert(sent.steering_permille == 0 && straight_speed == 1000);
    /* Sharp curve at 20%: inner wheel would be far below the minimum, so it
     * stops (pivot) instead of stalling; outer keeps turning; slower. */
    input.left = 900u; input.centre = 30u; settle(&c, 10u);
    assert(sent.steering_permille > 0 && right_wheel() == 0 &&
        left_wheel() > 0 && sent.target_speed_cps <= straight_speed * 6 / 10);
    /* Moderate curve: inner wheel raised to the sustainable minimum. */
    input.left = 840u; input.centre = 500u; settle(&c, 10u);
    assert(sent.steering_permille > 0 && wheel_ok(right_wheel()) &&
        right_wheel() > 0 && left_wheel() > right_wheel());
    /* Crossing a barcode: steady slow speed, then normal again. */
    input.left = 700u; input.centre = 700u; settle(&c, 60u);
    assert(sent.target_speed_cps == straight_speed);
    input.barcode_active = true; step(&c, 20u, 0u);
    assert(sent.target_speed_cps == VC_BARCODE_CPS);
    settle(&c, 10u);
    assert(sent.target_speed_cps == VC_BARCODE_CPS);
    input.barcode_active = false; settle(&c, 30u);
    assert(sent.target_speed_cps == straight_speed);
    input.left = 900u; input.centre = 30u; settle(&c, 10u);
    /* Look-ahead hold: back on centre, speed recovers only gradually. */
    input.left = 700u; input.centre = 700u; step(&c, 20u, 0u);
    assert(sent.target_speed_cps < straight_speed * 9 / 10);
    settle(&c, 40u);
    assert(sent.steering_permille == 0 &&
        sent.target_speed_cps == straight_speed);
    /* B3 junction rule: both strongly black and balanced only. */
    assert(B3_LINE_IS_JUNCTION(950u, 900u));
    assert(!B3_LINE_IS_JUNCTION(900u, 300u));
    assert(!B3_LINE_IS_JUNCTION(1000u, B3_LINE_JUNCTION_MIN - 1u));
    assert(!B3_LINE_IS_JUNCTION(1000u, 820u)); /* unbalanced */

    c = fresh(); command(&c, RCMD_START, 0u);
    input.left = 900u; input.centre = 900u;
    input.line_flags = LINE_F_DETECTED | LINE_F_JUNCTION;
    step(&c, 20u, 0u);
    assert(!c.junction_latched && sent.steering_permille == 0 &&
        sent.target_speed_cps < 1000);
    step(&c, 20u, 0u);
    step(&c, 20u, 0u);
    assert(c.junction_latched && sent.steering_permille == 0);
    input.line_flags = LINE_F_DETECTED;
    step(&c, 20u, 0u);
    step(&c, 20u, 0u);
    assert(c.junction_latched);
    step(&c, 20u, 0u);
    assert(!c.junction_latched);

    for (UINT failure = 0u; failure < 6u; ++failure) {
        c = fresh(); command(&c, RCMD_START, 0u); step(&c, 20u, 0u);
        if (failure == 0u) input.line_valid = false;
        if (failure == 1u) input.imu_valid = false;
        if (failure == 2u) input.hazard = true;
        if (failure == 3u) input.motion_valid = false;
        if (failure == 4u) input.motion.status = MOTION_STATUS_TIMEOUT;
        if (failure == 5u) input.faults = FAULT_ENCODER;
        step(&c, 20u, 0u); assert(!c.running && c.state == ROBOT_SAFE_STOP);
    }
    c = fresh(); command(&c, RCMD_START, 0u); step(&c, 20u, 0u);
    send_error = E_OBJ; step(&c, 20u, 0u); assert(c.running);
    step(&c, 40u, 0u); assert(!c.running);

    for (UB nav = B3_NAV_LEFT; nav <= B3_NAV_UTURN; ++nav) {
        c = fresh(); command(&c, RCMD_START, 0u); step(&c, 20u, 0u);
        step(&c, 20u, nav);
        assert(c.navigation == 0u && c.pending_navigation == nav);
        confirm_junction(&c);
        assert(c.state == ROBOT_NAV_COMMAND && c.navigation == nav &&
            c.navigation_id == 0u);
        input.clearance = false; step(&c, 20u, 0u); assert(c.navigation == nav);
        input.clearance = true; step(&c, 20u, 0u); assert(c.navigation_id == 42u);
        result = (motion_status_snapshot_t){0}; result.status = MOTION_STATUS_COMPLETED;
        result.physically_stopped = true; result_ready = true;
        step(&c, 20u, 0u); step(&c, 20u, 0u); assert(c.state == ROBOT_LINE_FOLLOW);
    }
    c = fresh(); command(&c, RCMD_START, 0u);
    step(&c, 20u, B3_NAV_RIGHT);
    confirm_junction(&c);
    step(&c, 20u, 0u);
    assert(sent.turn_direction == MOTION_TURN_RIGHT);
    result = (motion_status_snapshot_t){0};
    result.status = MOTION_STATUS_CANCELLED; result_ready = true;
    step(&c, 20u, 0u); assert(!c.running && c.state == ROBOT_SAFE_STOP);
    c = fresh(); command(&c, RCMD_START, 0u);
    step(&c, 20u, B3_NAV_UTURN);
    confirm_junction(&c);
    input.clearance = false;
    step(&c, VC_RECOVERY_MS, 0u); assert(!c.running);
    c = fresh(); command(&c, RCMD_START, 0u);
    input.line_flags = LINE_F_LOST; step(&c, 20u, 0u);
    input.line_flags = LINE_F_DETECTED; step(&c, 20u, 0u);
    assert(c.state == ROBOT_LINE_FOLLOW && c.running);
    c = fresh(); command(&c, RCMD_START, 0u);
    input.now_ms = UINT32_MAX - 10u; step(&c, 0u, 0u); step(&c, 20u, 0u);
    assert(drives == 2u);

    /* Avoidance owns motor commands until the line/heading handoff. */
    c = fresh(); command(&c, RCMD_START, 0u); step(&c, 20u, 0u);
    {
        UINT prior_drives = drives, prior_stops = stops;
        input.obstacle_active = true;
        input.clearance = false;
        input.line_flags = LINE_F_LOST;
        for (UINT i = 0u; i < 10u; ++i) step(&c, 20u, B3_NAV_RIGHT);
        assert(drives == prior_drives && stops == prior_stops && c.running);
        assert(c.navigation == 0u && c.pending_navigation == B3_NAV_NONE);
        input.obstacle_active = false;
        input.clearance = true;
        input.line_flags = LINE_F_DETECTED;
        step(&c, 20u, 0u);
        assert(drives == prior_drives + 1u && c.running);
    }
    for (UINT failure = 0u; failure < 6u; ++failure) {
        c = fresh(); command(&c, RCMD_START, 0u); step(&c, 20u, 0u);
        input.obstacle_active = true;
        step(&c, 20u, 0u);
        permission = true;
        if (failure == 0u) input.hazard = true;
        if (failure == 1u) input.obstacle_fault = true;
        if (failure == 2u) command(&c, RCMD_STOP, 0u);
        if (failure == 3u) command(&c, RCMD_ESTOP, 0u);
        if (failure == 4u) command(&c, RCMD_PAUSE, 0u);
        if (failure == 5u) command(&c, RCMD_SPEED, 0u);
        step(&c, 20u, 0u);
        assert(!permission);
    }

    /* Execute the real task adapter: stop dominates queued START; stale
     * producer timestamps never authorize movement, even with fresh reads. */
    for (UINT scenario = 0u; scenario < 5u; ++scenario) {
        c = fresh(); (void)c;
        clock_ms = 1000u + scenario * 1000u;
        task_ticks = remote_head = 0u; remote_count = scenario == 1u ? 2u : 1u;
        remote_queue[0] = (REMOTE_CMD){0}; remote_queue[0].type = scenario == 1u ? RCMD_ESTOP : RCMD_START;
        remote_queue[1] = (REMOTE_CMD){0}; remote_queue[1].type = RCMD_START;
        stale_line = scenario == 2u; stale_imu = scenario == 3u;
        snapshot_delay = scenario == 4u ? 5u : 0u;
        permission = true;
        if (setjmp(task_done) == 0) controller_task(0, NULL);
        assert(scenario == 0u || scenario == 4u ? drives > 0u : drives == 0u);
    }
    snapshot_delay = 0u;
    /* Autostart survives delayed first samples, without a remote START.
     * An explicit STOP or a bad IMU still prevents boot motion. */
    for (UINT scenario = 0u; scenario < 3u; ++scenario) {
        c = fresh(); (void)c;
        clock_ms = 10000u + scenario * 1000u;
        task_ticks = remote_head = 0u;
        remote_count = scenario == 1u ? 1u : 0u;
        remote_queue[0] = (REMOTE_CMD){0}; remote_queue[0].type = RCMD_STOP;
        stale_line = false; stale_imu = scenario == 2u; delayed_startup = true;
        permission = true;
        if (setjmp(task_done) == 0) controller_task(0, NULL);
        assert(scenario == 0u ? drives > 0u : drives == 0u);
    }
    c = fresh(); (void)c;
    clock_ms = 20000u;
    task_ticks = remote_head = remote_count = 0u;
    stale_line = stale_imu = delayed_startup = false;
    calibrated_startup = true;
    first_calibrated_drive_tick = UINT32_MAX;
    b3_line_cal_init(&startup_right_cal);
    b3_line_cal_init(&startup_left_cal);
    permission = true;
    if (setjmp(task_done) == 0) controller_task(0, NULL);
    assert(drives > 0u && first_calibrated_drive_tick == 5u);
    calibrated_startup = false;
    puts("PASS low-output calibration feeds autostart on third black sample without extra delay");
    assert(vehicle_controller_start() == E_OK);
    /* Small corrections retain their radius even at the startup speed. */
    for (INT steering = -200; steering <= 200; steering += 25) {
        motion_command_t gentle = {0};
        gentle.target_speed_cps = 100;
        gentle.steering_permille = steering;
        apply_wheel_minimum(&gentle);
        assert(gentle.steering_permille == steering);
        assert(gentle.target_speed_cps * (1000 - (steering < 0 ? -steering : steering)) /
            1000 >= VC_MIN_WHEEL_CPS);
    }
    /* Every START restarts B3 calibration (car lifted after a safe-stop
     * must not keep an in-air black level), including a repeat START. */
    c = fresh(); recal_requests = 0u;
    command(&c, RCMD_START, 0u); assert(recal_requests == 1u);
    command(&c, RCMD_STOP, 0u); command(&c, RCMD_START, 0u);
    assert(recal_requests == 2u && c.running);
    command(&c, RCMD_MODE, MODE_TEST); command(&c, RCMD_START, 0u);
    assert(recal_requests == 2u); /* TEST mode START is refused */
    /* Default speed: gentle search sweeps keep both wheels turning. */
    c = fresh(); command(&c, RCMD_START, 0u);
    input.left = 900u; input.centre = 100u; input.line_flags = LINE_F_DETECTED;
    settle(&c, 30u);
    input.line_flags = LINE_F_LOST; step(&c, 20u, 0u);
    step(&c, VC_LINE_REACQUIRE_MS, 0u);
    assert(c.state == ROBOT_LINE_RECOVERY &&
        sent.steering_permille == -VC_LINE_SEARCH_STEER &&
        wheel_ok(left_wheel()) && left_wheel() > 0 && right_wheel() > 0);
    /* LOST at START (car off the line, or B3 just restarted a bad
     * calibration): wait in place, never drive a search for a line not
     * yet seen; follow once seen; only then does losing it search. */
    c = fresh();
    input.line_flags = LINE_F_LOST; input.left = input.centre = 0u;
    command(&c, RCMD_START, 0u);
    {
        UINT before = drives;
        for (UINT i = 0u; i < 100u; ++i) step(&c, 20u, 0u);
        assert(drives == before && c.running && c.state == ROBOT_LINE_RECOVERY);
        input.line_flags = LINE_F_DETECTED; input.left = input.centre = 900u;
        step(&c, 20u, 0u);
        assert(drives == before + 1u && c.state == ROBOT_LINE_FOLLOW);
        input.line_flags = LINE_F_LOST; input.left = input.centre = 0u;
        step(&c, 20u, 0u);
        assert(drives == before + 2u && c.state == ROBOT_LINE_RECOVERY &&
            sent.type == MOTION_CMD_DRIVE);
    }
    puts("PASS: controller commands, curves, B5 handoff, boot readiness, hazards and task adapter");
    return 0;
}
