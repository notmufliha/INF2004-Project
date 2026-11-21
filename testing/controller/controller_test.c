#include "mock_platform.h"
/* The older tests below are for the close-together layout; the spread
 * layout is tested by switching VEHICLE_CONTROLLER.line_straddle on. */
#define VC_LINE_STRADDLE 0
#define VC_JUNCTION_APPROACH_MM 0u /* older tests: turn at once */
#define RUN_LOG_ENABLE 0
#include "../../app_program/vehicle_controller.c"
#include "../../app_program/vc_follow.c"
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
static bool boot_handshake, boot_mission_enabled;
S5MissionState s5_mission_state(void) { return obstacle_state; }
void s5_mission_enable(bool allow) { boot_mission_enabled = allow; }
bool s5_mission_approach_slow(void) { return false; }
static uint32_t generation = 1u;
static VC_INPUT input;
static REMOTE_CMD remote_queue[8];
static UINT remote_head, remote_count, task_ticks;
static uint32_t clock_ms;
static bool stale_line, stale_imu, delayed_startup;
static bool force_no_line;
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
static uint32_t cleared_faults;
void fault_clear(uint32_t bits) { cleared_faults |= bits; }
ER motion_speed_from_percent(INT percent, INT *speed) { *speed = percent * 50; return E_OK; }
ER motion_send_command(const motion_command_t *command)
{
    if (send_error != E_OK) return send_error;
    sent = *command;
    if (command->type == MOTION_CMD_DRIVE) {
        if (boot_handshake) assert(task_ticks >= 4u);
        /* Every DRIVE: no wheel in the stall zone, outer within B2's limit. */
        INT s = command->steering_permille;
        INT left = command->target_speed_cps * (1000 + s) / 1000;
        INT right = command->target_speed_cps * (1000 - s) / 1000;
        /* Spread following may run the inner wheel down to B2's move
         * minimum (VC_STRADDLE_MIN_INNER_CPS); never in the stall zone. */
        /* V4 deliberately starts below the steady wheel floor for its
         * first acceleration step after a stopped barcode rewind. */
        bool starting_v4 = VC_FOLLOW_V4 &&
            command->target_speed_cps <= VC_SPEED_RISE_PER_TICK;
        assert(left == 0 || left >= VC_STRADDLE_MIN_INNER_CPS - 1 || starting_v4);
        assert(right == 0 || right >= VC_STRADDLE_MIN_INNER_CPS - 1 || starting_v4);
        assert(left <= 5000 && right <= 5000 && (left > 0 || right > 0));
        if (calibrated_startup && drives == 0u) first_calibrated_drive_tick = task_ticks;
        ++drives;
    }
    if (command->type == MOTION_CMD_STOP) ++stops;
    return E_OK;
}
static float reversed_cm;
ER motion_move_cm(float cm, INT speed, motion_direction_t dir, uint32_t *id)
{
    if (dir == MOTION_DIR_REVERSE) { /* barcode re-read */
        assert(cm > 0.0f && cm <= 40.0f && speed > 0);
        reversed_cm = cm;
        *id = 42u; return E_OK;
    }
    assert((cm == 10.0f || (VC_FOLLOW_V4 && cm > 0.0f && cm <= VC4_JCT_ADVANCE_CM + VC4_JCT_SIDE_EXTRA_CM)) &&
           speed > 0 && dir == MOTION_DIR_FORWARD);
    *id = 42u; return E_OK;
}
ER motion_turn_degrees(INT angle, INT speed, motion_turn_direction_t dir, uint32_t *id)
{
    assert((angle == 90 || angle == 180 || angle == 360 ||
            (VC_FOLLOW_V4 && (angle == VC4_JCT_SIDE_TURN_DEG ||
                              angle == VC4_JCT_SIDE_TRIM_DEG)) ||
            angle == 90 - VC_TURN_SHORT_DEG || angle == 180 - VC_TURN_SHORT_DEG ||
            angle == 360 - VC_TURN_SHORT_DEG || angle == VC_SPIN_STEP_DEG) &&
           speed > 0);
    sent.type = MOTION_CMD_TURN;
    sent.target_angle_deg = angle;
    sent.target_speed_cps = speed;
    sent.turn_direction = dir;
    *id = 42u; return E_OK;
}
ER motion_get_result(uint32_t id, motion_status_snapshot_t *snapshot)
{
    assert(id == 42u);
    if (!result_ready) return E_OBJ;
    *snapshot = result; return E_OK;
}
/* B2's fault acknowledgement, as in motion_command.c. */
static uint32_t acked_fault_id;
static bool acked_fault_valid;
void motion_acknowledge_fault(void)
{
    if (input.motion.status == MOTION_STATUS_FAULT ||
        input.motion.status == MOTION_STATUS_TIMEOUT) {
        acked_fault_id = input.motion.command_id;
        acked_fault_valid = true;
    }
}
bool motion_fault_active(const motion_status_snapshot_t *s)
{
    return (s->status == MOTION_STATUS_FAULT ||
            s->status == MOTION_STATUS_TIMEOUT) &&
        !(acked_fault_valid && acked_fault_id == s->command_id);
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
    permission = boot_handshake ? boot_mission_enabled && task_ticks >= 3u : true;
    if (++task_ticks == (boot_handshake ? 7u : calibrated_startup ? 8u : 4u)) longjmp(task_done, 1);
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
uint16_t b3_barcode_rejects(void) { return 0u; }
void b3_line_raw(uint16_t *a, uint16_t *b) { if (a) *a = 300u; if (b) *b = 300u; }
void imu_live_get(imu_live_t *p) { *p = (imu_live_t){0}; }
void s5_mission_set_ramp(bool on) { (void)on; }
static UINT junction_beeps;
void b3_buzzer_request(B3_BEEP pattern)
{
    if (pattern == B3_BEEP_JUNCTION) ++junction_beeps;
}
ER b3_get_timed_state(B3_STATE *state, uint32_t *sample_ms)
{
    clock_ms += snapshot_delay;
    *state = (B3_STATE){0};
    state->line_flags = force_no_line ? LINE_F_LOST : LINE_F_DETECTED;
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
    if (boot_handshake && task_ticks == 1u) {
        notice->snapshot.b_valid = false;
        notice->events = IMU_NOTICE_UNAVAILABLE;
    }
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
    acked_fault_valid = false;
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
/* Drive centred over floor long enough for the barcode sensor's floor
 * level to be learned (both line sensors black: B3's JUNCTION flag is on
 * the whole time, as on the real car). */
static void drive_centred(VEHICLE_CONTROLLER *c, UINT ticks)
{
    input.left = input.centre = 900u;
    input.line_flags = LINE_F_DETECTED | LINE_F_JUNCTION;
    input.motion.left_speed_cps = input.motion.right_speed_cps = 1000;
    for (UINT i = 0u; i < ticks; ++i) step(c, 20u, 0u);
}
/* Barcode sensor black for `mm` of travel at 1000 counts/s (84 mm/s). */
static void gp28_black_mm(VEHICLE_CONTROLLER *c, UINT mm)
{
    UINT ticks = (mm * 1000u + 1679u) / 1680u; /* 1.68 mm per 20 ms tick */
    input.right = 1u;
    for (UINT i = 0u; i < ticks; ++i) step(c, 20u, 0u);
    input.right = 0u;
}
/* A real crossing: floor learned, then 18 mm of crossing tape. */
static void confirm_junction(VEHICLE_CONTROLLER *c)
{
    drive_centred(c, 60u);
    input.right = 1u;
    for (UINT i = 0u; i < 14u && !c->junction_latched; ++i) step(c, 20u, 0u);
    assert(c->junction_latched);
}

/* One unfiltered steering sample: the mapping, without the low-pass. */
static INT steer_once(VEHICLE_CONTROLLER *p, const VC_INPUT *v)
{
    p->error_filter_valid = false;
    return line_follow_steering(p, v);
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
    input.left = input.centre = 900u;
    step(&c, 20u, 0u); /* Centred: ease off at most the release rate, no snap. */
    assert(sent.steering_permille < B3_LINE_STEER_MAX &&
        sent.steering_permille >= B3_LINE_STEER_MAX - VC_STEER_RELEASE_PER_TICK);
    step(&c, 20u, 0u); step(&c, 20u, 0u); step(&c, 20u, 0u);
    assert(sent.steering_permille == 0); /* straight within 4 ticks */
    input.left = 900u; input.centre = 30u;
    settle(&c, 30u);
    input.left = 30u; input.centre = 900u;
    step(&c, 20u, 0u); /* Reversal is rate-limited too, no full swing. */
    assert(sent.steering_permille ==
        B3_LINE_STEER_MAX - VC_STEER_RELEASE_PER_TICK);
    /* (filtered error is 0 here, so the full release step applies) */
    step(&c, 20u, 0u); step(&c, 20u, 0u); step(&c, 20u, 0u);
    assert(sent.steering_permille < 0); /* turning back within 80 ms */
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
    }
    assert(B3_LINE_LEFT_ACTIVE(1000u) && !B3_LINE_LEFT_ACTIVE(1001u));
    /* Edge steering for this robot (both sensors on black when centred):
     * nothing while both read black, full correction as soon as one
     * sensor is ~40% onto the white, in the right direction. */
    {
        VEHICLE_CONTROLLER p = {0};
        VC_INPUT v = {0};
        v.left = 1000u; v.centre = 1000u;          /* centred */
        assert(steer_once(&p, &v) == 0);
        v.left = 1000u; v.centre = 970u;           /* noise: deadband */
        assert(steer_once(&p, &v) == 0);
        v.left = 1000u; v.centre = 780u;           /* left sensor at edge */
        assert(steer_once(&p, &v) > 0 && steer_once(&p, &v) < 250);
        v.left = 1000u; v.centre = 500u;           /* left half off: full */
        INT moderate = steer_once(&p, &v);
        assert(moderate > 0 && moderate < B3_LINE_STEER_MAX);
        assert(p.last_line_search_direction == 1); /* line is to the right */
        v.left = 500u; v.centre = 1000u;           /* right half off */
        assert(steer_once(&p, &v) == -moderate);
        assert(p.last_line_search_direction == -1);
        v.left = 0u; v.centre = 1000u;
        assert(steer_once(&p, &v) == -B3_LINE_STEER_MAX);
    }
    /* Straight with GP26 noise: alternating +-110 single-sample spikes
     * (~11 raw counts on its 100-count span) used to cross the deadband
     * every tick; the low-pass keeps the steering straight. */
    {
        VEHICLE_CONTROLLER p = {0};
        VC_INPUT v = {0};
        v.centre = 900u;
        for (UINT i = 0u; i < 20u; ++i) {
            v.left = (i & 1u) ? 1000u : 790u;
            INT s = line_follow_steering(&p, &v);
            if (i > 0u) assert(s == 0);
        }
    }
    puts("PASS straight-line ADC noise filtered, no steering flicker");
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
    {
        uint32_t started = c.recovery_ms;
        /* Reproduce COM6: clearance toggles every few hundred ms while
         * both sensors are white. The search must advance, then stop. */
        input.clearance = true;
        step(&c, VC_LINE_REACQUIRE_MS, 0u);
        assert(c.recovery_ms == started);
        assert(sent.steering_permille == -VC_LINE_SEARCH_STEER);
        for (UINT i = 0u; i < 20u; ++i) {
            input.clearance = false; step(&c, 200u, 0u);
            input.clearance = true; step(&c, 200u, 0u);
            assert(c.recovery_ms == started);
        }
        assert(sent.type == MOTION_CMD_STOP && c.running);
        UINT prior_drives = drives;
        input.clearance = false; step(&c, 20u, 0u);
        input.clearance = true; step(&c, 20u, 0u);
        assert(drives == prior_drives);
        input.line_flags = LINE_F_DETECTED;
        step(&c, 20u, 0u);
        assert(!c.line_recovery_active && drives == prior_drives + 1u);
    }

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
    input.left = input.centre = 1000u;
    input.line_flags = LINE_F_DETECTED | LINE_F_JUNCTION;
    confirm_junction(&c);
    assert(c.running && c.state == ROBOT_LINE_FOLLOW);
    assert(sent.type == MOTION_CMD_DRIVE && sent.steering_permille == 0);
    assert(left_wheel() > 0 && left_wheel() == right_wheel());

    c = fresh(); command(&c, RCMD_START, 0u);
    input.line_flags = LINE_F_DETECTED;
    input.left = 700u; input.centre = 700u;
    step(&c, 20u, 0u); /* acceleration from standstill is ramped */
    assert(sent.target_speed_cps == VC_MIN_WHEEL_CPS);
    settle(&c, 30u);
    INT straight_speed = sent.target_speed_cps;
    assert(sent.steering_permille == 0 && straight_speed == 1000);
    /* Sharp curve at 20%: inner wheel would be far below the minimum.  While
     * following it is held AT the minimum (no pivot lurch); outer at least
     * VC_FOLLOW_MIN_OUTER_RATIO times faster; slower than straight. */
    input.left = 900u; input.centre = 30u; settle(&c, 10u);
    assert(sent.steering_permille > 0 && wheel_ok(right_wheel()) &&
        right_wheel() > 0 &&
        left_wheel() >= right_wheel() * VC_FOLLOW_MIN_OUTER_RATIO - 2 &&
        sent.target_speed_cps <= straight_speed * 7 / 10);
    /* Moderate curve: inner wheel raised to the sustainable minimum. */
    input.left = 840u; input.centre = 500u; settle(&c, 10u);
    assert(sent.steering_permille > 0 && wheel_ok(right_wheel()) &&
        right_wheel() > 0 && left_wheel() > right_wheel());
    /* Crossing a barcode: speed frozen, even when the junction flag or a
     * steering correction would normally slow the car, then normal again. */
    input.left = 700u; input.centre = 700u; settle(&c, 60u);
    assert(sent.target_speed_cps == straight_speed);
    input.barcode_active = true; step(&c, 20u, 0u);
    assert(sent.target_speed_cps == straight_speed);
    input.line_flags = LINE_F_DETECTED | LINE_F_JUNCTION; settle(&c, 5u);
    assert(sent.target_speed_cps == straight_speed);
    input.line_flags = LINE_F_DETECTED;
    input.left = 840u; input.centre = 500u; settle(&c, 5u);
    assert(sent.target_speed_cps == straight_speed);
    input.left = 700u; input.centre = 700u; settle(&c, 10u);
    assert(sent.target_speed_cps == straight_speed);
    input.barcode_active = false; settle(&c, 60u);
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

    /* Centred on the 18 mm line, both line sensors are black and B3's
     * JUNCTION flag is on all the time: that alone is NOT a junction (a
     * barcode's turn used to run at once, right after the read). */
    c = fresh(); command(&c, RCMD_START, 0u);
    step(&c, 20u, B3_NAV_RIGHT);
    assert(c.pending_navigation == B3_NAV_RIGHT);
    drive_centred(&c, 150u); /* 3 s, 250 mm */
    assert(!c.junction_latched && c.navigation == 0u &&
        c.pending_navigation == B3_NAV_RIGHT && c.state == ROBOT_LINE_FOLLOW);
    /* Barcode bars under the barcode sensor (wide bar 9 mm) are not the
     * crossing either. */
    for (UINT bar = 0u; bar < 5u; ++bar) {
        gp28_black_mm(&c, 9u);
        drive_centred(&c, 3u);
    }
    assert(!c.junction_latched && c.navigation == 0u);
    /* The crossing tape (18 mm) is: the queued turn runs there. */
    input.right = 1u;
    for (UINT i = 0u; i < 14u && c.navigation == 0u; ++i) step(&c, 20u, 0u);
    assert(c.junction_latched && c.navigation == B3_NAV_RIGHT &&
        c.state == ROBOT_NAV_COMMAND);
    /* Unknown floor level (not yet learned): never a junction. */
    c = fresh(); command(&c, RCMD_START, 0u);
    input.right = 1u; drive_centred(&c, 30u);
    assert(!c.junction_latched);
    puts("PASS junction needs the crossing tape under the barcode sensor, not just a centred line");

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

    /* Barcode actions beyond the four turns. */
    {
        INT normal_speed;
        /* SLOW / NORMAL act at once, without a crossing. */
        c = fresh(); command(&c, RCMD_START, 0u);
        input.left = input.centre = 700u; input.line_flags = LINE_F_DETECTED;
        settle(&c, 60u);
        normal_speed = sent.target_speed_cps;
        assert(normal_speed == 1000);
        step(&c, 20u, B3_NAV_SLOW);
        assert(c.slow_mode && c.pending_navigation == B3_NAV_NONE);
        settle(&c, 5u);
        assert(sent.target_speed_cps == normal_speed / 2);
        step(&c, 20u, B3_NAV_NORMAL);
        settle(&c, 60u);
        assert(!c.slow_mode && sent.target_speed_cps == normal_speed);
        step(&c, 20u, B3_NAV_SLOW);
        command(&c, RCMD_STOP, 0u); command(&c, RCMD_START, 0u);
        assert(!c.slow_mode); /* a new run starts at normal speed */

        /* STOP: stand 3 s at the crossing, then cross it straight. */
        c = fresh(); command(&c, RCMD_START, 0u);
        step(&c, 20u, B3_NAV_STOP);
        confirm_junction(&c);
        assert(c.navigation == B3_NAV_STOP && c.state == ROBOT_NAV_COMMAND);
        settle(&c, VC_NAV_STOP_MS / 20u - 2u);
        assert(c.navigation == B3_NAV_STOP && c.navigation_id == 0u &&
            c.running);
        settle(&c, 3u);
        assert(c.navigation == B3_NAV_STRAIGHT && c.navigation_id == 42u);

        /* SPIN: one 360-degree turn at the crossing, then follow again. */
        c = fresh(); command(&c, RCMD_START, 0u);
        step(&c, 20u, B3_NAV_SPIN);
        confirm_junction(&c);
        step(&c, 20u, 0u);
        assert(sent.type == MOTION_CMD_TURN && sent.target_angle_deg == 360 &&
            c.navigation_id == 42u);
        result = (motion_status_snapshot_t){0};
        result.status = MOTION_STATUS_COMPLETED;
        result.physically_stopped = true; result_ready = true;
        step(&c, 20u, 0u); step(&c, 20u, 0u);
        assert(c.running && c.navigation == 0u && c.state == ROBOT_LINE_FOLLOW);
        result_ready = false;

        /* FINISH: stop at the crossing and end the run (START restarts). */
        c = fresh(); command(&c, RCMD_START, 0u);
        step(&c, 20u, B3_NAV_FINISH);
        confirm_junction(&c);
        step(&c, 20u, 0u);
        assert(!c.running && c.state == ROBOT_IDLE && c.stopped);
        UINT before_finish = drives;
        settle(&c, 20u);
        assert(drives == before_finish);
        command(&c, RCMD_START, 0u); step(&c, 20u, 0u);
        assert(c.running && drives == before_finish + 1u);

        /* LINE_ONLY mode ignores every barcode action. */
        c = fresh(); command(&c, RCMD_MODE, MODE_LINE_ONLY);
        command(&c, RCMD_START, 0u);
        step(&c, 20u, B3_NAV_SLOW); step(&c, 20u, B3_NAV_FINISH);
        assert(!c.slow_mode && c.pending_navigation == B3_NAV_NONE);
    }
    puts("PASS barcode actions: SLOW/NORMAL now, STOP 3 s, SPIN 360, FINISH ends the run");

    /* Motor stall: stop, wait, retry (twice); then stay stopped until
     * START, which restarts even though B2 still publishes the stalled
     * command's FAULT status (that used to need a reflash). */
    {
        UINT before;
        c = fresh(); command(&c, RCMD_START, 0u); settle(&c, 5u);
        input.motion.status = MOTION_STATUS_FAULT;
        input.motion.fault = MOTION_FAULT_STALL;
        for (UINT attempt = 1u; attempt <= VC_STALL_AUTO_RETRIES; ++attempt) {
            input.motion.command_id = 6u + attempt;
            input.faults = FAULT_MOTOR_STALL;
            before = drives;
            step(&c, 20u, 0u);
            assert(c.running && c.stall_retries == attempt && drives == before);
            input.faults = 0u; /* cleared by the retry */
            settle(&c, VC_STALL_RETRY_MS / 20u - 1u);
            assert(c.running && drives == before); /* waiting */
            settle(&c, 2u);
            assert(c.running && drives > before && /* driving again */
                c.state == ROBOT_LINE_FOLLOW);
        }
        input.motion.command_id = 9u;
        input.faults = FAULT_MOTOR_STALL;
        step(&c, 20u, 0u);
        assert(!c.running && c.state == ROBOT_SAFE_STOP); /* retries used up */
        input.faults = 0u;
        settle(&c, 10u);
        assert(!c.running);
        /* START: B2 still says FAULT for command 9, and B5 still reports a
         * mission FAULT for a moment while it re-arms. */
        input.obstacle_fault = true;
        command(&c, RCMD_START, 0u);
        before = drives;
        step(&c, 20u, 0u); step(&c, 20u, 0u);
        assert(c.running && drives == before); /* waiting for B5 */
        input.obstacle_fault = false;
        step(&c, 20u, 0u);
        assert(c.running && drives == before + 1u &&
            c.state == ROBOT_LINE_FOLLOW);
        /* A NEW fault after the restart still stops (retry budget reset). */
        input.motion.command_id = 10u;
        input.motion.fault = MOTION_FAULT_ENCODER;
        step(&c, 20u, 0u);
        assert(!c.running && c.state == ROBOT_SAFE_STOP);
        /* B5 still faulted after the grace period: safe stop as before. */
        input.motion.status = MOTION_STATUS_COMPLETED;
        input.motion.fault = MOTION_FAULT_NONE;
        input.obstacle_fault = true;
        command(&c, RCMD_START, 0u);
        settle(&c, VC_FAULT_ACK_GRACE_MS / 20u + 1u);
        assert(!c.running && c.state == ROBOT_SAFE_STOP);
        input.obstacle_fault = false;
    }
    puts("PASS motor stall: 2 automatic retries, then START restarts without reflashing");

    /* Spread sensors: the line runs BETWEEN them, so both on floor is the
     * centred position, not a lost line (that ran the hard search between
     * every correction: the jerky following). */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 50u);
        assert(c.state == ROBOT_LINE_FOLLOW && !c.line_recovery_active &&
            sent.type == MOTION_CMD_DRIVE && sent.steering_permille == 0);
        /* Right sensor touches the line: gentle, proportional correction. */
        input.left = 250u; input.line_flags = LINE_F_DETECTED;
        settle(&c, 10u);
        assert(c.state == ROBOT_LINE_FOLLOW && sent.steering_permille > 0 &&
            sent.steering_permille < B3_LINE_STEER_MAX / 2);
        /* Back between the sensors: straight again, still following. */
        input.left = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 10u);
        assert(c.state == ROBOT_LINE_FOLLOW && sent.steering_permille == 0);
        /* Sharp curve: the line passes right under the left sensor and out
         * past it.  Coast, then a growing arc left, then search left. */
        input.centre = 1000u; input.line_flags = LINE_F_DETECTED;
        settle(&c, 10u);
        assert(c.follow_steer <= -B3_LINE_STEER_MAX + 20); /* before wheel limits */
        input.centre = 0u; input.line_flags = LINE_F_LOST;
        /* The car was already steering hard left when the line left the
         * left sensor: a curve tighter than the turn.  No straight coast:
         * it keeps turning left at once. */
        step(&c, 20u, 0u);
        assert(c.state == ROBOT_LINE_FOLLOW && c.straddle_coast_ms == 0u &&
            c.follow_steer <= -VC_STRADDLE_BEND_STEER);
        settle(&c, 30u);
        assert(c.state == ROBOT_LINE_FOLLOW &&
            c.follow_steer == -VC_STRADDLE_OUTSIDE_STEER);
        settle(&c, VC_STRADDLE_OUTSIDE_MS / 20u - 30u);
        assert(c.state == ROBOT_LINE_RECOVERY && sent.type == MOTION_CMD_TURN &&
            sent.turn_direction == MOTION_TURN_LEFT);    /* spin search left */
        settle(&c, 5u);
        assert(c.state == ROBOT_LINE_RECOVERY); /* stays lost */
        input.centre = 300u; input.line_flags = LINE_F_DETECTED; /* graze */
        step(&c, 20u, 0u);
        assert(c.state == ROBOT_LINE_FOLLOW);
        /* No sensor touches the line for VC_STRADDLE_BLIND_MS: lost.  Before
         * that the car drifts toward the side that last touched (seek). */
        input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, VC_STRADDLE_SEEK_MS / 20u - 1u);
        assert(c.state == ROBOT_LINE_FOLLOW && c.follow_steer == 0);
        settle(&c, 30u);
        assert(c.follow_steer < 0 && c.follow_steer >= -VC_STRADDLE_SEEK_MAX);
        settle(&c, VC_STRADDLE_BLIND_MS / 20u - VC_STRADDLE_SEEK_MS / 20u - 31u);
        assert(c.state == ROBOT_LINE_FOLLOW);
        settle(&c, 3u);
        assert(c.state == ROBOT_LINE_RECOVERY);
        /* Both black on the spread layout = a crossing tape: straight. */
        input.left = input.centre = 1000u;
        input.line_flags = LINE_F_DETECTED | LINE_F_JUNCTION;
        settle(&c, 10u);
        assert(c.state == ROBOT_LINE_FOLLOW && sent.steering_permille == 0);
    }
    puts("PASS spread sensors: centred on floor follows, out-past-sensor turns, blind -> search");

    /* The 2026-11-18 zig-zag: right exit, then left exit.  The second exit
     * is on the other side, so the car coasts longer instead of swinging
     * the line straight back across. */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 10u);
        input.left = 1000u; input.line_flags = LINE_F_DETECTED; settle(&c, 5u);
        input.left = 0u; input.line_flags = LINE_F_LOST; settle(&c, 3u);
        input.centre = 1000u; input.line_flags = LINE_F_DETECTED; settle(&c, 5u);
        input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, VC_STRADDLE_FLIP_COAST_MS / 20u - 2u);
        assert(c.state == ROBOT_LINE_FOLLOW && c.follow_steer == 0);
        settle(&c, 4u);
        assert(c.follow_steer < 0);
    }
    puts("PASS spread sensors: exit on the other side coasts longer (no zig-zag)");

    /* Layout auto-detect: a crossing tape keeps the spread layout; the
     * line under both sensors for 6 cm means they are close together. */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.motion.left_speed_cps = input.motion.right_speed_cps = 1000;
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 10u);
        input.left = input.centre = 1000u;           /* 18 mm tape */
        input.line_flags = LINE_F_DETECTED | LINE_F_JUNCTION;
        settle(&c, 12u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 5u);
        assert(c.line_straddle);
        input.left = input.centre = 1000u;
        input.line_flags = LINE_F_DETECTED | LINE_F_JUNCTION;
        settle(&c, 40u);                              /* 67 mm */
        assert(c.line_straddle);
        settle(&c, 25u);                              /* 109 mm */
        assert(!c.line_straddle);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 2u);                               /* close: really lost */
        assert(c.state == ROBOT_LINE_RECOVERY);
        input.motion.left_speed_cps = input.motion.right_speed_cps = 0;
    }
    puts("PASS layout auto-detect: line under both sensors -> close-together following");

    /* Close layout junction: the GP28 tape alone, even off-centre (B3's
     * balanced JUNCTION flag off) and after normal corrections. */
    {
        c = fresh(); command(&c, RCMD_START, 0u);
        drive_centred(&c, 60u);           /* floor learned */
        input.left = 1000u; input.centre = 600u;  /* off-centre */
        input.line_flags = LINE_F_DETECTED;      /* no JUNCTION flag */
        c.curve_steer = 450;                     /* recent correction */
        step(&c, 20u, B3_NAV_LEFT);
        assert(c.pending_navigation == B3_NAV_LEFT);
        input.right = 1u;
        for (UINT i = 0u; i < 14u && c.navigation == 0u; ++i) step(&c, 20u, 0u);
        assert(c.navigation == B3_NAV_LEFT);
        input.right = 0u;
        /* A hard curve still blocks it (the line can pass under GP28). */
        c = fresh(); command(&c, RCMD_START, 0u);
        drive_centred(&c, 60u);
        step(&c, 20u, B3_NAV_LEFT);
        input.left = 1000u; input.centre = 30u; input.line_flags = LINE_F_DETECTED;
        input.right = 1u;
        for (UINT i = 0u; i < 14u; ++i) step(&c, 20u, 0u);
        assert(c.navigation == 0u && c.curve_steer >= VC_JUNCTION_MAX_CURVE_STEER);
        input.right = 0u;
    }
    puts("PASS close-layout junction: GP28 tape counts off-centre and after corrections");

    /* Spread layout junction: L + R >= 1400 for 10 mm; GP28 tape + one
     * saturated sensor (side branch). */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.motion.left_speed_cps = input.motion.right_speed_cps = 1000;
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 60u);
        step(&c, 20u, B3_NAV_RIGHT);
        input.left = 800u; input.centre = 700u; input.line_flags = LINE_F_DETECTED;
        for (UINT i = 0u; i < 14u && c.navigation == 0u; ++i) step(&c, 20u, 0u);
        assert(c.navigation == B3_NAV_RIGHT);
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        input.motion.left_speed_cps = input.motion.right_speed_cps = 1000;
        settle(&c, 60u);                 /* GP28 floor learned */
        step(&c, 20u, B3_NAV_LEFT);
        input.centre = 950u; input.line_flags = LINE_F_DETECTED; /* left branch */
        input.right = 1u;
        for (UINT i = 0u; i < 14u && c.navigation == 0u; ++i) step(&c, 20u, 0u);
        assert(c.navigation == B3_NAV_LEFT);
        input.right = 0u;
        input.motion.left_speed_cps = input.motion.right_speed_cps = 0;
    }
    puts("PASS spread-layout junction: both sensors on the crossing, or a branch at GP28");

    /* Tape crossed at an angle with a weak right sensor: left black, then
     * right black 10 mm later, never both at once.  Still a junction;
     * one touch each side 6 cm apart (normal following) is not. */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.motion.left_speed_cps = input.motion.right_speed_cps = 1000;
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 20u);
        step(&c, 20u, B3_NAV_LEFT);
        input.centre = 1000u; input.line_flags = LINE_F_DETECTED; settle(&c, 3u);
        input.centre = 0u; input.line_flags = LINE_F_LOST; settle(&c, 36u);
        input.left = 400u; input.line_flags = LINE_F_DETECTED; settle(&c, 3u);
        input.left = 0u; input.line_flags = LINE_F_LOST; settle(&c, 3u);
        assert(c.navigation == 0u && !c.junction_latched);
        junction_beeps = 0u;
        input.centre = 1000u; input.line_flags = LINE_F_DETECTED; settle(&c, 3u);
        input.centre = 0u; input.line_flags = LINE_F_LOST; settle(&c, 3u);
        input.left = 400u; input.line_flags = LINE_F_DETECTED;
        for (UINT i = 0u; i < 6u && c.navigation == 0u; ++i) step(&c, 20u, 0u);
        assert(c.navigation == B3_NAV_LEFT && junction_beeps == 1u);
        input.left = 0u;
        input.motion.left_speed_cps = input.motion.right_speed_cps = 0;
    }
    puts("PASS spread-layout junction: both sides touched within 25 mm, beeps");

    /* Barcode turn: keep following past the junction until the wheels are
     * on it (approach distance), only then stop and turn. */
    {
        c = fresh(); c.line_straddle = true;
        c.approach_um = 50000u;
        command(&c, RCMD_START, 0u);
        input.motion.left_speed_cps = input.motion.right_speed_cps = 1000;
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 60u);
        step(&c, 20u, B3_NAV_LEFT);
        input.left = input.centre = 1000u; input.line_flags = LINE_F_DETECTED;
        for (UINT i = 0u; i < 6u && !c.approach_active; ++i) step(&c, 20u, 0u);
        assert(c.approach_active && c.navigation == 0u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        UINT before = drives;
        settle(&c, 20u);                       /* 34 mm: still following */
        assert(c.navigation == 0u && drives > before &&
            c.state == ROBOT_LINE_FOLLOW);
        settle(&c, 12u);                       /* past 50 mm: turn */
        assert(c.navigation == B3_NAV_LEFT && !c.approach_active);
        input.motion.left_speed_cps = input.motion.right_speed_cps = 0;
    }
    puts("PASS barcode turn waits until the wheels reach the junction");

    /* Bend: the line leaves the same sensor again within a second, so the
     * car turns back toward it at once (no coasting). */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 10u);
        input.centre = 1000u; input.line_flags = LINE_F_DETECTED; settle(&c, 5u);
        input.centre = 0u; input.line_flags = LINE_F_LOST; settle(&c, 10u);
        input.centre = 1000u; input.line_flags = LINE_F_DETECTED; settle(&c, 5u);
        input.centre = 0u; input.line_flags = LINE_F_LOST; step(&c, 20u, 0u);
        settle(&c, 3u);
        assert(c.straddle_coast_ms == 0u && c.follow_steer < -200);
    }
    puts("PASS bend: repeated exit on the same side keeps turning, no coast");

    /* Pivot only for a line held >= 700 ms under one sensor (the hump),
     * never for normal 300-450 ms touches (3e849eb's 250 ms rule pivoted
     * on every touch: full-left/full-right zig-zag, 2026-11-18 run log). */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 10u);
        input.left = 1000u; input.line_flags = LINE_F_DETECTED;
        bool pivoted = false;
        for (UINT i = 0u; i < 22u; ++i) {             /* 440 ms touch */
            step(&c, 20u, 0u);
            if (right_wheel() == 0) pivoted = true;
        }
        assert(!pivoted);
        input.left = 0u; input.line_flags = LINE_F_LOST; settle(&c, 20u);
        input.centre = 1000u; input.line_flags = LINE_F_DETECTED;
        settle(&c, 30u);                              /* 600 ms: no pivot */
        assert(left_wheel() > 0);
        settle(&c, 20u);                              /* 1 s: hump, pivot */
        assert(left_wheel() == 0 && right_wheel() > 0);
        input.centre = 0u; input.line_flags = LINE_F_LOST;
    }
    puts("PASS pivot only when a sensor holds the line >= 700 ms (hump), not per touch");

    /* U-turn (spread): B2 turns 160 deg, then the car keeps turning left
     * until a sensor finds the line; both on floor is not "centred". */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.motion.left_speed_cps = input.motion.right_speed_cps = 1000;
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 60u);
        step(&c, 20u, B3_NAV_UTURN);
        input.left = input.centre = 1000u; input.line_flags = LINE_F_DETECTED;
        for (UINT i = 0u; i < 8u && c.navigation == 0u; ++i) step(&c, 20u, 0u);
        assert(c.navigation == B3_NAV_UTURN);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        input.motion.physically_stopped = true;
        step(&c, 20u, 0u);
        assert(sent.type == MOTION_CMD_TURN &&
            sent.target_angle_deg == 180 - VC_TURN_SHORT_DEG &&
            sent.turn_direction == MOTION_TURN_LEFT);
        result_ready = true; result.status = MOTION_STATUS_COMPLETED;
        result.physically_stopped = true;
        step(&c, 20u, 0u);
        assert(c.navigation == 0u);
        result_ready = false;
        settle(&c, 10u);                      /* both on floor: still turning */
        assert(c.state == ROBOT_LINE_RECOVERY && sent.type == MOTION_CMD_TURN &&
            sent.target_angle_deg == VC_SPIN_STEP_DEG &&
            sent.turn_direction == MOTION_TURN_LEFT);    /* on the spot */
        input.centre = 1000u; input.line_flags = LINE_F_DETECTED; /* found */
        step(&c, 20u, 0u);
        assert(c.state == ROBOT_LINE_FOLLOW);
        input.centre = 0u; input.line_flags = LINE_F_LOST;
        input.motion.left_speed_cps = input.motion.right_speed_cps = 0;
    }
    puts("PASS U-turn: turns short, then keeps turning until a sensor finds the line");

    /* Barcode beside the line: the left line sensor flickers over the bars
     * (the right one never).  It is ignored on the strip: no steering
     * toward the barcode, no junction. */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.motion.left_speed_cps = input.motion.right_speed_cps = 1000;
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 60u);
        step(&c, 20u, B3_NAV_LEFT);
        input.barcode_active = true;
        for (UINT i = 0u; i < 40u; ++i) {
            input.right = (UB)((i / 2u) & 1u);           /* GP28 bars */
            input.centre = ((i / 3u) & 1u) ? 1000u : 0u; /* left sensor bars */
            input.line_flags = input.centre ? LINE_F_DETECTED : LINE_F_LOST;
            if (i == 30u) input.left = 400u;             /* line touches right */
            if (i == 33u) input.left = 0u;
            step(&c, 20u, 0u);
        }
        assert(c.bar_ignore == -1 && !c.junction_latched &&
            c.navigation == 0u && !c.approach_active);
        assert(c.follow_steer >= 0);       /* not pulled toward the bars */
        input.barcode_active = false; input.centre = 0u; input.right = 0u;
        input.line_flags = LINE_F_LOST;
        settle(&c, 30u);
        assert(!c.bars_active && c.pending_navigation == B3_NAV_LEFT);
        input.motion.left_speed_cps = input.motion.right_speed_cps = 0;
    }
    puts("PASS barcode bars under a line sensor: ignored, no fake junction");

    /* Linear spread steering gives a useful correction on a moderate
     * left-sensor reading, reaches full steer, then settles to zero. */
    {
        VEHICLE_CONTROLLER p = {0};
        VC_INPUT v = {0};
        p.line_straddle = true;
        v.centre = 350u;
        INT moderate = 0;
        for (UINT i = 0u; i < 10u; ++i) {
            moderate = line_follow_steering(&p, &v);
        }
        assert(moderate < -B3_LINE_STEER_MAX / 3);
        v.centre = 1000u;
        INT full = 0;
        for (UINT i = 0u; i < 12u; ++i) {
            full = line_follow_steering(&p, &v);
        }
        assert(full == -B3_LINE_STEER_MAX);
        v.centre = 0u;
        INT rest = 1;
        for (UINT i = 0u; i < 12u; ++i) rest = line_follow_steering(&p, &v);
        assert(rest == 0);
    }
    puts("PASS spread steering: strong left correction, full scale, settles to 0");

    /* Line search on the spot: 15 deg steps toward the last side, 90 deg,
     * then back the other way; the line found during a step stops it. */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 5u);
        c.straddle_lost = true; c.last_line_search_direction = 1;
        step(&c, 20u, 0u);
        assert(c.state == ROBOT_LINE_RECOVERY && sent.type == MOTION_CMD_TURN &&
            sent.turn_direction == MOTION_TURN_RIGHT);
        for (UINT k = 0u; k < VC_SPIN_FIRST_STEPS; ++k) {
            result_ready = true; result.status = MOTION_STATUS_COMPLETED;
            step(&c, 20u, 0u);                  /* step done, next issued */
            result_ready = false; step(&c, 20u, 0u);
        }
        assert(sent.type == MOTION_CMD_TURN &&
            sent.turn_direction == MOTION_TURN_LEFT && c.spin_sweep == 1u);
        UINT stops_before = stops;
        input.centre = 600u; input.line_flags = LINE_F_DETECTED; /* found */
        step(&c, 20u, 0u);
        assert(stops == stops_before + 1u && c.spin_id == 0u &&
            c.state == ROBOT_LINE_FOLLOW);
        step(&c, 20u, 0u);
        assert(sent.type == MOTION_CMD_DRIVE);
        input.centre = 0u; input.line_flags = LINE_F_LOST;
    }
    puts("PASS line search: in-place steps, widening sweep, stops cleanly when found");

    /* Junction: both sensors >= 700 at once for 2 ticks is a crossing even
     * right after START (no steady-following wait needed). */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 3u);
        step(&c, 20u, B3_NAV_RIGHT);
        input.left = input.centre = 900u; input.line_flags = LINE_F_DETECTED;
        for (UINT i = 0u; i < 6u && c.navigation == 0u; ++i) step(&c, 20u, 0u);
        assert(c.navigation == B3_NAV_RIGHT);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
    }
    puts("PASS junction: both sensors black at once counts immediately");

    /* Turning authority (spread): a full correction at following speed
     * gives a 5:1 wheel ratio with the inner wheel >= 150 cps, i.e. ~667
     * permille; the old 300 cps floor capped it near 500. */
    {
        motion_command_t m = {0};
        m.target_speed_cps = 360; m.steering_permille = -800;
        straddle_wheels(&m, false);
        INT l = m.target_speed_cps * (1000 + m.steering_permille) / 1000;
        INT r = m.target_speed_cps * (1000 - m.steering_permille) / 1000;
        assert(l >= VC_STRADDLE_MIN_INNER_CPS - 1 && r <= 5 * l + 5 &&
            m.steering_permille <= -650);
        m.target_speed_cps = 700; m.steering_permille = 300;  /* gentle */
        straddle_wheels(&m, false);
        assert(m.steering_permille == 300 && m.target_speed_cps == 700);
    }
    puts("PASS spread wheels: 5:1 ratio, inner >= 150 cps, gentle steering untouched");

    /* Light touch on a straight (little steering): the usual coast, so a
     * centred line is not pushed across (no zig-zag). */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 10u);
        input.centre = 500u; input.line_flags = LINE_F_DETECTED; settle(&c, 3u);
        assert(-c.follow_steer < VC_STRADDLE_BEND_STEER);
        input.centre = 0u; input.line_flags = LINE_F_LOST; step(&c, 20u, 0u);
        assert(c.straddle_coast_ms == VC_STRADDLE_COAST_MS);
    }
    puts("PASS light touch on a straight still coasts (no zig-zag)");
#if VC_BEND_HOLD
    /* Bend hold (SITProjects line_track.c seek): the left sensor touching
     * twice within 80 mm is a bend, and while both sensors are white the car
     * keeps turning left instead of straightening and letting the line run
     * out.  A single touch on a straight still releases to straight. */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 10u);
        c.travel_um = 1000000u;
        input.centre = 350u; input.line_flags = LINE_F_DETECTED;
        settle(&c, 4u);                            /* touch 1 (light) */
        assert(c.bend_side == 0 && c.bend_touch_side == -1);
        c.travel_um += 10000u;
        input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 6u);                            /* straight: released */
        assert(c.bend_side == 0 && c.follow_steer > -VC_BEND_SEEK_MIN);
        c.travel_um += 30000u;
        input.centre = 350u; input.line_flags = LINE_F_DETECTED;
        settle(&c, 4u);                            /* touch 2: bend */
        assert(c.bend_side == -1 && c.follow_steer <= -VC_BEND_SEEK_MIN);
        input.centre = 0u; input.line_flags = LINE_F_LOST;
        for (UINT i = 0u; i < 15u; ++i) {
            c.travel_um += 2000u;
            step(&c, 20u, 0u);
            assert(c.follow_steer <= -VC_BEND_SEEK_MIN); /* never straight */
        }
        assert(c.follow_steer <= -VC_BEND_SEEK_MAX + VC_STEER_SLEW_PER_TICK);
        /* The right sensor briefly touching inside the bend: gentle,
         * bend kept; a long right touch makes it a right bend. */
        input.left = 350u; input.line_flags = LINE_F_DETECTED;
        c.travel_um += 5000u; step(&c, 20u, 0u);
        assert(c.bend_side == -1 && c.bend_counter);
        c.travel_um += 70000u; step(&c, 20u, 0u);
        assert(c.bend_side == 1 && !c.bend_counter);
    }
    puts("PASS bend: keeps turning with the bend while both sensors are white");
#endif

    /* Search never turns the car round: 60 one way, back 120, back 60 to
     * the start heading, then stops (armed). */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 5u);
        c.straddle_lost = true; c.last_line_search_direction = -1;
        step(&c, 20u, 0u);
        INT heading = 0, most = 0;
        for (UINT k = 0u; k < 40u && !c.spin_done; ++k) {
            INT dir = sent.turn_direction == MOTION_TURN_RIGHT ? 1 : -1;
            result_ready = true; result.status = MOTION_STATUS_COMPLETED;
            step(&c, 20u, 0u);
            heading += dir * VC_SPIN_STEP_DEG;
            if (heading > most) most = heading;
            if (-heading > most) most = -heading;
            result_ready = false; step(&c, 20u, 0u);
        }
        assert(c.spin_done && heading == 0 &&
            most == (INT)VC_SPIN_FIRST_STEPS * VC_SPIN_STEP_DEG);
        UINT turns = 0u;
        for (UINT k = 0u; k < 10u; ++k) {
            sent.type = MOTION_CMD_STOP; step(&c, 20u, 0u);
            if (sent.type == MOTION_CMD_TURN) ++turns;
        }
        assert(turns == 0u);
    }
    puts("PASS search: +-60 deg then back to the start heading, never turns round");

    /* Rejected barcode: stop, reverse past the strip start, drive again. */
    {
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_START, 0u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        input.motion.left_speed_cps = input.motion.right_speed_cps = 1000;
        input.motion.physically_stopped = false;
        settle(&c, 10u);
        for (UINT i = 0u; i < 30u; ++i) {             /* ~50 mm of bars */
            input.right = (UB)((i / 3u) & 1u ? 0u : 1u);
            step(&c, 20u, 0u);
        }
        input.right = 0u; settle(&c, 10u);            /* paused-scan reject */
        input.barcode_rejects = 1u; step(&c, 20u, 0u);
        assert(c.rescan_phase == 0u && c.rescan_pending); /* still on strip */
        settle(&c, 14u);                              /* strip passed */
        assert(c.rescan_phase == 1u && c.state == ROBOT_NAV_COMMAND);
        assert(c.rescan_back_um > 130000u && c.rescan_back_um < 160000u);
        step(&c, 20u, 0u);
        assert(c.rescan_phase == 1u); /* waits until really stopped */
        input.motion.physically_stopped = true;
        reversed_cm = 0.0f;
        step(&c, 20u, 0u);
        assert(c.rescan_phase == 2u && reversed_cm > 13.0f && reversed_cm < 16.0f);
        result_ready = false; step(&c, 20u, 0u);
        assert(c.rescan_phase == 2u);
        result_ready = true; result.status = MOTION_STATUS_COMPLETED;
        result.physically_stopped = true;
        input.motion.physically_stopped = false;
        step(&c, 20u, 0u); settle(&c, 2u);
        assert(c.rescan_phase == 0u && c.state == ROBOT_LINE_FOLLOW &&
            sent.type == MOTION_CMD_DRIVE);
        result_ready = false;
        /* Rejected again on the second pass: no endless reversing. */
        input.barcode_rejects = 2u; step(&c, 20u, 0u);
        assert(c.rescan_phase == 0u && c.state == ROBOT_LINE_FOLLOW);
        /* A read barcode resets the tries; the next bad one re-reads. */
        step(&c, 20u, B3_NAV_SLOW);
        input.barcode_rejects = 3u; step(&c, 20u, 0u);
        assert(c.rescan_phase == 1u);
        /* Not in LINE_ONLY mode (no barcode actions there). */
        c = fresh(); c.line_straddle = true;
        command(&c, RCMD_MODE, MODE_LINE_ONLY); command(&c, RCMD_START, 0u);
        settle(&c, 5u);
        input.barcode_rejects = 4u; step(&c, 20u, 0u);
        assert(c.rescan_phase == 0u);
        input.barcode_rejects = 0u;
        input.motion.left_speed_cps = input.motion.right_speed_cps = 0;
    }
    puts("PASS rejected barcode: back up once and read it again");

    /* 2026-11-18 run 1: the barcode is read (reversed) while backing up
     * and the sensors sweep over the line; afterwards the car must not
     * turn until a real junction ahead. */
    {
        c = fresh(); c.line_straddle = true; c.approach_um = 50000u;
        command(&c, RCMD_START, 0u);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        input.motion.left_speed_cps = input.motion.right_speed_cps = 1000;
        input.motion.physically_stopped = false;
        settle(&c, 30u);
        input.barcode_rejects = 5u; step(&c, 20u, 0u);
        settle(&c, 30u);
        assert(c.rescan_phase == 1u);
        input.motion.physically_stopped = true; step(&c, 20u, 0u);
        assert(c.rescan_phase == 2u);
        input.motion.physically_stopped = false;
        step(&c, 20u, B3_NAV_LEFT);                   /* read while reversing */
        input.left = input.centre = 1000u; input.line_flags = LINE_F_DETECTED;
        settle(&c, 5u);                               /* line under both */
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        result_ready = true; result.status = MOTION_STATUS_COMPLETED;
        result.physically_stopped = true;
        step(&c, 20u, 0u);
        assert(c.rescan_phase == 0u && !c.junction_latched &&
            c.pending_navigation == B3_NAV_LEFT);
        settle(&c, 6u);
        /* Crooked after the reverse: crosses the line, left then right
         * sensor 10 mm apart.  Not a junction (not steady yet). */
        input.centre = 1000u; input.line_flags = LINE_F_DETECTED; settle(&c, 3u);
        input.centre = 0u; input.left = 400u; settle(&c, 3u);
        input.left = 0u; input.line_flags = LINE_F_LOST;
        settle(&c, 50u);                              /* no junction */
        assert(c.navigation == 0u && c.pending_navigation == B3_NAV_LEFT &&
            !c.approach_active);
        /* Steady now: a real crossing is a junction. */
        input.left = input.centre = 1000u; input.line_flags = LINE_F_DETECTED;
        for (UINT i = 0u; i < 6u && !c.approach_active; ++i) step(&c, 20u, 0u);
        assert(c.approach_active);
        input.left = input.centre = 0u; input.line_flags = LINE_F_LOST;
        result_ready = false;
        input.barcode_rejects = 0u;
        input.motion.left_speed_cps = input.motion.right_speed_cps = 0;
    }
    puts("PASS re-read: line crossed while reversing is not a junction");

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
        c.curve_steer = B3_LINE_STEER_MAX;
        c.last_speed = 2000;
        input.left = input.centre = 800u;
        step(&c, 20u, 0u);
        assert(drives == prior_drives + 1u && c.running);
        assert(!c.obstacle_handoff && c.curve_steer == 0);
        assert(c.last_speed < 2000); /* Restart gently after the bypass. */
    }
    /* An obstacle encountered during line recovery must not consume the
     * post-bypass search window while B5 owns the wheels. */
    c = fresh(); command(&c, RCMD_START, 0u);
    input.line_flags = LINE_F_LOST;
    step(&c, 20u, 0u);
    assert(c.state == ROBOT_LINE_RECOVERY);
    input.obstacle_active = true;
    step(&c, VC_LINE_SEARCH_MS + 100u, 0u);
    input.obstacle_active = false;
    {
        UINT prior_drives = drives;
        step(&c, 20u, 0u);
        assert(c.running && drives == prior_drives + 1u);
        assert(c.recovery_ms == input.now_ms);
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
    /* Real startup handshake: no B5 clearance until sensing is enabled,
     * transient IMU unavailability before clearance, then recovery.
     * Do not consume autostart or send DRIVE during the warmup fault. */
    c = fresh(); (void)c;
    boot_handshake = true; boot_mission_enabled = false;
    stale_line = stale_imu = delayed_startup = false;
    clock_ms = 19000u;
    task_ticks = remote_head = remote_count = 0u;
    permission = false;
    if (setjmp(task_done) == 0) controller_task(0, NULL);
    assert(drives > 0u);
    boot_handshake = false;
    puts("PASS boot arming survives IMU warmup failure and waits for B5 clearance");
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
    assert(drives > 0u && first_calibrated_drive_tick == 0u);
    calibrated_startup = false;
    puts("PASS startup searches immediately with fresh white samples, then follows calibrated black");
    c = fresh(); (void)c;
    force_no_line = true;
    boot_handshake = true; boot_mission_enabled = false;
    clock_ms = 22000u;
    task_ticks = remote_head = remote_count = 0u;
    permission = false;
    if (setjmp(task_done) == 0) controller_task(0, NULL);
    assert(drives > 0u && boot_mission_enabled);
    assert(published == ROBOT_LINE_RECOVERY);
    force_no_line = boot_handshake = false;
    puts("PASS startup off tape enables B5 clearance and issues bounded line-search DRIVE");
    assert(vehicle_controller_start() == E_OK);
    /* Small corrections retain their radius even at the startup speed. */
    for (INT steering = -200; steering <= 200; steering += 25) {
        motion_command_t gentle = {0};
        gentle.target_speed_cps = 100;
        gentle.steering_permille = steering;
        apply_wheel_minimum(&gentle, false);
        assert(gentle.steering_permille == steering);
        assert(gentle.target_speed_cps * (1000 - (steering < 0 ? -steering : steering)) /
            1000 >= VC_MIN_WHEEL_CPS);
    }
    /* START clears B2's latched stall/encoder faults (operator restart). */
    c = fresh(); cleared_faults = 0u;
    command(&c, RCMD_START, 0u);
    assert((cleared_faults & (FAULT_MOTOR_STALL | FAULT_ENCODER)) ==
        (FAULT_MOTOR_STALL | FAULT_ENCODER));
    /* PAUSE/STOP/MODE republish the state even if another writer (B5)
     * changed the telemetry state behind the controller's back. */
    c = fresh(); command(&c, RCMD_START, 0u); step(&c, 20u, 0u);
    command(&c, RCMD_PAUSE, 0u); assert(published == ROBOT_IDLE);
    published = ROBOT_OBSTACLE_AVOID;
    command(&c, RCMD_PAUSE, 0u); assert(published == ROBOT_IDLE);
    published = ROBOT_OBSTACLE_AVOID;
    command(&c, RCMD_STOP, 0u); assert(published == ROBOT_SAFE_STOP);
    published = ROBOT_OBSTACLE_AVOID;
    command(&c, RCMD_STOP, 0u); assert(published == ROBOT_SAFE_STOP);
    published = ROBOT_OBSTACLE_AVOID;
    command(&c, RCMD_MODE, MODE_LINE_ONLY); assert(published == ROBOT_IDLE);
    puts("PASS: controller commands, curves, B5 handoff, boot readiness, hazards and task adapter");
    return 0;
}
