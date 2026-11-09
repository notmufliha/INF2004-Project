#include "mock_platform.h"
#include "../../app_program/vehicle_controller.c"
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <setjmp.h>
#include <string.h>

static motion_command_t sent;
static UINT drives, stops, resets;
static ER send_error;
static motion_status_snapshot_t result;
static bool result_ready;
static ROBOT_STATE published;
static bool permission = true;
static uint32_t generation = 1u;
static VC_INPUT input;
static REMOTE_CMD remote_queue[8];
static UINT remote_head, remote_count, task_ticks;
static uint32_t clock_ms;
static bool stale_line, stale_imu;
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
    if (command->type == MOTION_CMD_DRIVE) ++drives;
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
    if (++task_ticks == 4u) longjmp(task_done, 1);
    return E_OK;
}
INT tk_rcv_mbf(ID id, void *data, TMO timeout)
{
    (void)timeout;
    if (id != mbf_remote_cmd || remote_head == remote_count) return E_TMOUT;
    memcpy(data, &remote_queue[remote_head++], sizeof(REMOTE_CMD));
    return sizeof(REMOTE_CMD);
}
ER b3_get_timed_state(B3_STATE *state, uint32_t *sample_ms)
{
    *state = (B3_STATE){0};
    state->line_flags = LINE_F_DETECTED;
    *sample_ms = stale_line ? clock_ms - 101u : clock_ms;
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
    static const char letters[] = {'A', 'B', 'C', 'D'};
    UINT i;
    send_error = E_OK; result_ready = false; drives = stops = 0u;
    input = (VC_INPUT){0};
    input.now_ms = 100u; input.clearance = input.line_valid = input.imu_valid = true;
    input.motion_valid = true;
    input.motion.physically_stopped = true;
    input.line_flags = LINE_F_DETECTED;
    input.left = 3000u; input.centre = 1000u;
    input.safety_generation = generation;
    vehicle_controller_init(&c);
    for (i = 0U; i < 4U; ++i) {
        REMOTE_CMD map = {0};
        map.type = RCMD_BARCODE_MAP;
        map.arg = ((UINT)(letters[i] - 'A') << 2U) | i;
        vehicle_controller_command(&c, &map);
    }
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

int main(void)
{
    VEHICLE_CONTROLLER c = fresh();
    VEHICLE_CONTROLLER unconfigured;
    vehicle_controller_init(&unconfigured);
    command(&unconfigured, RCMD_START, 0u);
    assert(!unconfigured.running);

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
    step(&c, 20u, 1u); assert(c.navigation == 0u);
    command(&c, RCMD_SPEED, 100u); input.left = 3000u; input.centre = 1000u;
    step(&c, 20u, 0u);
    assert(sent.steering_permille < 0 && sent.target_speed_cps * 1350 <= 5000000);
    input.hump = true; step(&c, 20u, 0u);
    assert(c.state == ROBOT_HUMP && sent.target_speed_cps < 2500);
    command(&c, RCMD_SPEED, 0u); step(&c, 20u, 0u);
    assert(c.state == ROBOT_IDLE);
    command(&c, RCMD_RESET_PEAK, 0u); assert(resets == 1u);

    c = fresh(); command(&c, RCMD_START, 0u); step(&c, 20u, 0u);
    input.clearance = false; step(&c, 20u, 0u);
    assert(c.state == ROBOT_OBSTACLE_SCAN && c.running);
    input.clearance = true; step(&c, 20u, 0u); assert(c.state == ROBOT_LINE_FOLLOW);
    input.faults = FAULT_WIFI_LOSS; step(&c, 20u, 0u); assert(c.running);
    input.line_flags = LINE_F_LOST; step(&c, 20u, 0u);
    assert(c.state == ROBOT_LINE_RECOVERY);
    input.clearance = false; step(&c, VC_RECOVERY_MS, 0u);
    assert(c.state == ROBOT_SAFE_STOP && !c.running);

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

    for (UB route = BARCODE_ROUTE_FORWARD;
         route < BARCODE_ROUTE_COUNT; ++route) {
        UB letter = (UB)(route + 1U);
        UB expected_nav = route == BARCODE_ROUTE_FORWARD ? B3_NAV_NONE :
            route == BARCODE_ROUTE_UTURN ? B3_NAV_UTURN :
            route == BARCODE_ROUTE_LEFT ? B3_NAV_LEFT : B3_NAV_RIGHT;

        c = fresh(); command(&c, RCMD_START, 0u); step(&c, 20u, 0u);
        step(&c, 20u, letter);
        assert(c.pending_barcode == letter && c.navigation == 0u);
        assert(c.state == ROBOT_LINE_FOLLOW);
        input.line_flags = LINE_F_DETECTED | LINE_F_JUNCTION;
        step(&c, 20u, 0u);
        assert(c.pending_barcode == 0u && c.navigation == expected_nav);
        if (route == BARCODE_ROUTE_FORWARD) {
            assert(c.state == ROBOT_LINE_FOLLOW && c.navigation_id == 0u);
            continue;
        }
        assert(c.state == ROBOT_NAV_COMMAND && c.navigation_id == 0u);
        input.clearance = false; step(&c, 20u, 0u); assert(c.navigation == expected_nav);
        input.clearance = true; step(&c, 20u, 0u); assert(c.navigation_id == 42u);
        result = (motion_status_snapshot_t){0}; result.status = MOTION_STATUS_COMPLETED;
        result.physically_stopped = true; result_ready = true;
        step(&c, 20u, 0u); step(&c, 20u, 0u); assert(c.state == ROBOT_LINE_FOLLOW);
    }
    c = fresh(); command(&c, RCMD_START, 0u);
    step(&c, 20u, 4u);
    input.line_flags = LINE_F_DETECTED | LINE_F_JUNCTION;
    step(&c, 20u, 0u);
    step(&c, 20u, 0u);
    assert(sent.turn_direction == MOTION_TURN_RIGHT);
    result = (motion_status_snapshot_t){0};
    result.status = MOTION_STATUS_CANCELLED; result_ready = true;
    step(&c, 20u, 0u); assert(!c.running && c.state == ROBOT_SAFE_STOP);
    c = fresh(); command(&c, RCMD_START, 0u);
    step(&c, 20u, 2u);
    input.line_flags = LINE_F_DETECTED | LINE_F_JUNCTION;
    step(&c, 20u, 0u);
    input.clearance = false;
    step(&c, VC_RECOVERY_MS, 0u); assert(!c.running);

    c = fresh(); command(&c, RCMD_START, 0u);
    step(&c, 20u, 5u);
    input.line_flags = LINE_F_DETECTED | LINE_F_JUNCTION;
    step(&c, 20u, 0u);
    assert(!c.running && c.state == ROBOT_SAFE_STOP);

    c = fresh(); command(&c, RCMD_START, 0u);
    step(&c, 20u, 1u);
    step(&c, 20u, 2u);
    assert(!c.running && c.state == ROBOT_SAFE_STOP);
    c = fresh(); command(&c, RCMD_START, 0u);
    input.line_flags = LINE_F_LOST; step(&c, 20u, 0u);
    input.line_flags = LINE_F_DETECTED; step(&c, 20u, 0u);
    assert(c.state == ROBOT_LINE_FOLLOW && c.running);
    c = fresh(); command(&c, RCMD_START, 0u);
    input.now_ms = UINT32_MAX - 10u; step(&c, 0u, 0u); step(&c, 20u, 0u);
    assert(drives == 2u);

    /* Execute the real task adapter: stop dominates queued START; stale
     * producer timestamps never authorize movement, even with fresh reads. */
    for (UINT scenario = 0u; scenario < 4u; ++scenario) {
        c = fresh(); (void)c;
        clock_ms = 1000u + scenario * 1000u;
        task_ticks = remote_head = 0u; remote_count = scenario == 1u ? 6u : 5u;
        for (UINT i = 0U; i < 4U; ++i) {
            remote_queue[i] = (REMOTE_CMD){0};
            remote_queue[i].type = RCMD_BARCODE_MAP;
            remote_queue[i].arg = (i << 2U) | i;
        }
        remote_queue[4] = (REMOTE_CMD){0};
        remote_queue[4].type = scenario == 1u ? RCMD_ESTOP : RCMD_START;
        remote_queue[5] = (REMOTE_CMD){0};
        remote_queue[5].type = RCMD_START;
        stale_line = scenario == 2u; stale_imu = scenario == 3u;
        permission = true;
        if (setjmp(task_done) == 0) controller_task(0, NULL);
        assert(scenario == 0u ? drives > 0u : drives == 0u);
    }
    assert(vehicle_controller_start() == E_OK);
    puts("PASS: controller commands, DRIVE cadence, hazards, navigation, wrap and task adapter");
    return 0;
}
