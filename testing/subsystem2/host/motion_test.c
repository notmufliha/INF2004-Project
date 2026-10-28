#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "mock_platform.h"
/* Include the real implementation to exercise its state machine, not a copy. */
#include "../../../app_program/subsystem2/motion_command.c"
#include "../../../app_program/subsystem2/pid_control.c"
#define ROBOT_CONFIG_H
#define ROBOT_ENABLE_DRIVE 1
#define ROBOT_WHEELS_UP_MOTOR_TEST 0
#define ROBOT_MOTION_TEST 6
#define ROBOT_MOTION_TEST_SPEED_CPS 1000
#define ROBOT_MOTION_TEST_BOOT_MS 5000u
#include "../../../app_program/subsystem2/motion_self_test.c"

ID app_start_task(FP entry, INT priority, UW core, const char *name)
{ (void)entry; (void)priority; (void)core; (void)name; return 1; }

static uint32_t clock_ms;
static INT encoder_left;
static INT encoder_right;
static uint32_t safety_generation;
static uint32_t acknowledged;
static bool latched;
static bool clearance;
static INT pwm_left;
static INT pwm_right;
static INT semaphore;
static UINT checks;
#define CHECK(expr) do { assert(expr); checks++; } while (0)

ER tk_get_tim(SYSTIM *time)
{ time->hi = 0u; time->lo = clock_ms; return E_OK; }
ID tk_cre_sem(const T_CSEM *sem)
{ semaphore = sem->isemcnt; return 1; }
ER tk_wai_sem(ID id, INT count, INT timeout)
{
    (void)id; (void)count; (void)timeout;
    if (semaphore == 0) return E_TMOUT;
    semaphore = 0; return E_OK;
}
ER tk_sig_sem(ID id, INT count)
{ (void)id; (void)count; semaphore = 1; return E_OK; }
ER tk_wup_tsk(ID id) { (void)id; return E_OK; }
ER tk_can_wup(ID id) { (void)id; return E_OK; }
ER tk_slp_tsk(INT timeout) { (void)timeout; return E_SYS; }
ER tk_dly_tsk(INT delay) { (void)delay; return E_OK; }
INT tm_printf(UB *format, ...) { (void)format; return 0; }
void encoder_get_counts(INT *left, INT *right)
{ *left = encoder_left; *right = encoder_right; }
void b2_request_stop(void)
{
    if (!latched) safety_generation++;
    latched = true; clearance = false; pwm_left = 0; pwm_right = 0;
}
void b2_safety_force_stop(void) { pwm_left = 0; pwm_right = 0; }
void b2_safety_acknowledge_stop(void)
{ b2_safety_force_stop(); acknowledged = safety_generation; }
bool b2_safety_stop_requested(void)
{ return acknowledged != safety_generation; }
uint32_t b2_safety_stop_generation(void) { return safety_generation; }
bool b2_safety_motion_allowed(void)
{ return !latched && clearance && acknowledged == safety_generation; }
bool b2_motor_is_stopped(void)
{ return pwm_left == 0 && pwm_right == 0 && !b2_safety_stop_requested(); }
void b2_safety_apply_pwm(int left, int right, uint32_t generation)
{
    if (generation == safety_generation && b2_safety_motion_allowed())
    { pwm_left = left; pwm_right = right; }
    else b2_safety_force_stop();
}

static pid_state_t setup(void)
{
    pid_state_t state = {0};
    clock_ms = 0; encoder_left = 0; encoder_right = 0;
    safety_generation = 1; acknowledged = 1;
    latched = false; clearance = true; pwm_left = 0; pwm_right = 0;
    CHECK(motion_command_init() == E_OK);
    CHECK(pid_initialize_state(&state) == E_OK);
    return state;
}

static void tick(pid_state_t *state, INT left, INT right)
{
    clock_ms += 10; encoder_left += left; encoder_right += right;
    CHECK(pid_sample(state) == E_OK);
    pid_guard(state); pid_accept(state); pid_guard(state);
    pid_execute(state); pid_publish(state);
    state->left_previous = state->left_now;
    state->right_previous = state->right_now;
    state->previous_ms = state->now_ms;
}

static motion_command_t command(motion_cmd_type_t type)
{
    motion_command_t value = {0};
    value.type = type; value.direction = MOTION_DIR_FORWARD;
    value.target_speed_cps = 1000; value.target_distance_counts = 1000;
    value.target_angle_deg = 90; value.safety_generation = safety_generation;
    return value;
}

static uint32_t send(motion_command_t value)
{
    uint32_t id = 0;
    CHECK(motion_send_command_with_id(&value, &id) == E_OK);
    CHECK(id != 0);
    return id;
}

static void test_profiles(void)
{
    motion_profile_t p;
    p = motion_profile_targets(20, 0, 10, 0, 100, false, true);
    CHECK(p.left_speed < p.right_speed);
    p = motion_profile_targets(20, .5f, 10, 0, 0, false, true);
    CHECK(p.left_speed == 30 && p.right_speed == 10);
    p = motion_profile_targets(20, 0, 100, 40, 100, true, true);
    CHECK(p.left_speed == 0 && p.right_speed > 0);
    CHECK(!p.right_reached);
    p = motion_profile_targets(20, 0, 95, 95, 100, true, true);
    CHECK(p.left_speed < 20 && p.left_speed > 0);
}

static void test_validation_and_queue(void)
{
    pid_state_t state = setup();
    motion_command_t value = command(MOTION_CMD_MOVE);
    motion_status_snapshot_t result;
    uint32_t first = send(value), id = 99;
    CHECK(motion_send_command_with_id(&value, &id) == E_OBJ && id == 0);
    value.type = MOTION_CMD_STOP;
    send(value);
    CHECK(motion_get_result(first, &result) == E_OK);
    CHECK(result.status == MOTION_STATUS_CANCELLED);
    tick(&state, 0, 0);
    CHECK(pwm_left == 0 && pwm_right == 0);
    value = command(MOTION_CMD_DRIVE); value.steering_permille = 1001;
    CHECK(motion_send_command(&value) == E_PAR);
    value.steering_permille = 1000; value.target_speed_cps = 5000;
    CHECK(motion_send_command(&value) == E_PAR);
    value = command(MOTION_CMD_MOVE); value.steering_permille = 1;
    CHECK(motion_send_command(&value) == E_PAR);
}

static void test_move_and_settle(bool reverse)
{
    pid_state_t state = setup();
    motion_command_t value = command(MOTION_CMD_MOVE);
    motion_status_snapshot_t result;
    INT sign = reverse ? -1 : 1;
    uint32_t id;
    value.direction = reverse ? MOTION_DIR_REVERSE : MOTION_DIR_FORWARD;
    id = send(value); tick(&state, 0, 0);
    CHECK(pwm_left * sign > 0 && pwm_right * sign > 0);
    tick(&state, sign * 1001, sign * 100);
    CHECK(state.status == MOTION_STATUS_RUNNING);
    CHECK(pwm_left == 0 && pwm_right * sign > 0);
    tick(&state, 0, sign * 900);
    CHECK(state.settling && state.status == MOTION_STATUS_RUNNING);
    CHECK(pwm_left == 0 && pwm_right == 0);
    for (INT i = 0; i < 10; i++) tick(&state, 0, 0);
    CHECK(motion_get_result(id, &result) == E_OK);
    CHECK(result.status == MOTION_STATUS_COMPLETED);
    CHECK(result.physically_stopped);
    CHECK(result.direction == value.direction);
}

static void test_turn(bool right)
{
    pid_state_t state = setup();
    motion_command_t value = command(MOTION_CMD_TURN);
    INT left_sign = right ? 1 : -1;
    uint32_t id;
    motion_status_snapshot_t result;
    value.turn_direction = right ? MOTION_TURN_RIGHT : MOTION_TURN_LEFT;
    INT left_target = (INT)(pid_turn_track(&value) *
        ENC_LEFT_COUNTS_PER_REV / (4.0f * ENC_WHEEL_DIAMETER_CM)) + 2;
    INT right_target = (INT)(pid_turn_track(&value) *
        ENC_RIGHT_COUNTS_PER_REV / (4.0f * ENC_WHEEL_DIAMETER_CM)) + 2;
    if (right)
    {
        CHECK(left_target > 1190 && left_target < 1210);
    }
    else
    {
        CHECK(left_target > 1160 && left_target < 1180);
    }
    id = send(value); tick(&state, 0, 0);
    CHECK(pwm_left * left_sign > 0 && pwm_right * left_sign < 0);
    tick(&state, left_sign * 950, -left_sign * 100);
    CHECK(!pid_profile(&state).left_reached && !state.settling);
    tick(&state, left_sign * (left_target - 950), 0);
    CHECK(pwm_left == 0 && pwm_right != 0);
    tick(&state, 0, -left_sign * (right_target - 100));
    CHECK(state.settling);
    for (INT i = 0; i < 10; i++) tick(&state, 0, 0);
    CHECK(motion_get_result(id, &result) == E_OK);
    CHECK(result.status == MOTION_STATUS_COMPLETED);
}

static void test_guards(void)
{
    pid_state_t state = setup();
    motion_status_snapshot_t result;
    uint32_t id = send(command(MOTION_CMD_DRIVE));
    tick(&state, 0, 0);
    for (INT i = 0; i < 10; i++) tick(&state, 1, 1);
    CHECK(motion_get_result(id, &result) == E_OK);
    CHECK(result.fault == MOTION_FAULT_WATCHDOG);
    CHECK(pwm_left == 0 && pwm_right == 0);
    state = setup(); id = send(command(MOTION_CMD_MOVE));
    tick(&state, 0, 0);
    for (INT i = 0; i < 50; i++) tick(&state, 1, 0);
    CHECK(motion_get_result(id, &result) == E_OK);
    CHECK(result.fault == MOTION_FAULT_STALL);
    CHECK(pwm_left == 0 && pwm_right == 0);
    state = setup(); id = send(command(MOTION_CMD_MOVE));
    tick(&state, 0, 0); b2_request_stop(); tick(&state, 0, 0);
    CHECK(motion_get_result(id, &result) == E_OK);
    CHECK(result.fault == MOTION_FAULT_SAFETY);
    latched = false; clearance = true; tick(&state, 0, 0);
    CHECK(pwm_left == 0 && pwm_right == 0);
}

static void test_results_and_stale(void)
{
    pid_state_t state = setup();
    motion_status_snapshot_t result;
    uint32_t id = send(command(MOTION_CMD_MOVE));
    b2_request_stop(); tick(&state, 0, 0);
    CHECK(motion_get_result(id, &result) == E_OK);
    CHECK(result.status == MOTION_STATUS_CANCELLED);
    state = setup(); id = send(command(MOTION_CMD_DRIVE));
    tick(&state, 0, 0);
    send(command(MOTION_CMD_TURN)); tick(&state, 0, 0);
    CHECK(motion_get_result(id, &result) == E_OK);
    CHECK(result.status == MOTION_STATUS_CANCELLED);
    for (UINT i = 0; i < MOTION_RESULT_HISTORY; i++)
    {
        result.command_id = 100u + i; result.status = MOTION_STATUS_TIMEOUT;
        CHECK(motion_update_status(&result) == E_OK);
    }
    CHECK(motion_get_result(id, &result) == E_OBJ);
    CHECK(motion_get_result(100, &result) == E_OK);
    CHECK(motion_get_result(0, &result) == E_PAR);
}

static void test_feedback_and_time(void)
{
    pid_state_t state = setup();
    motion_command_t value = command(MOTION_CMD_DRIVE);
    motion_profile_t profile;
    send(value); tick(&state, 0, 0);
    profile = pid_profile(&state);
    CHECK(fabsf(profile.left_speed - profile.right_speed) < .001f);
    /* Refreshes preserve history; run across multiple ring-buffer wraps. */
    for (INT i = 0; i < 100; i++)
    {
        send(value); tick(&state, 10, 9);
        CHECK(state.status == MOTION_STATUS_RUNNING);
        CHECK(pwm_left >= 0 && pwm_left <= PID_MAX_PWM);
    }
    clock_ms += 30;
    CHECK(pid_sample(&state) == E_SYS);
}

static void test_timeouts_and_standstill(void)
{
    pid_state_t state = setup();
    motion_status_snapshot_t result;
    motion_command_t value = command(MOTION_CMD_MOVE);
    uint32_t id;
    value.target_distance_counts = 100000;
    value.timeout_ms = 1000;
    id = send(value); tick(&state, 0, 0);
    for (INT i = 0; i < 100; i++) tick(&state, 1, 1);
    CHECK(motion_get_result(id, &result) == E_OK);
    CHECK(result.status == MOTION_STATUS_TIMEOUT);
    CHECK(result.fault == MOTION_FAULT_DURATION);
    state = setup(); id = send(command(MOTION_CMD_STOP));
    tick(&state, 0, 0);
    for (INT i = 0; i < 150; i++) tick(&state, 1, 0);
    CHECK(motion_get_result(id, &result) == E_OK);
    CHECK(result.fault == MOTION_FAULT_STOPPING);
    CHECK(!result.physically_stopped);
    state = setup(); id = send(command(MOTION_CMD_TURN));
    tick(&state, 0, 0);
    /* Wrong encoder direction must not count as successful progress. */
    for (INT i = 0; i < 50; i++) tick(&state, 1, 1);
    CHECK(motion_get_result(id, &result) == E_OK);
    CHECK(result.fault == MOTION_FAULT_STALL);
}

static void test_stop_and_steering(void)
{
    pid_state_t state = setup();
    motion_command_t value = command(MOTION_CMD_DRIVE);
    motion_status_snapshot_t result;
    uint32_t movement = send(value), stop;
    tick(&state, 0, 0);
    CHECK(pwm_left > 0 && pwm_right > 0);
    value.type = MOTION_CMD_STOP; stop = send(value);
    CHECK(pwm_left == 0 && pwm_right == 0);
    tick(&state, 1, 1);
    CHECK(motion_get_result(movement, &result) == E_OK);
    CHECK(result.status == MOTION_STATUS_CANCELLED);
    for (INT i = 0; i < 10; i++) tick(&state, 0, 0);
    CHECK(motion_get_result(stop, &result) == E_OK);
    CHECK(result.command_type == MOTION_CMD_STOP);
    CHECK(result.status == MOTION_STATUS_COMPLETED);
    state = setup(); value = command(MOTION_CMD_DRIVE);
    value.steering_permille = 1000;
    send(value); tick(&state, 0, 0);
    CHECK(pwm_left > 0 && pwm_right == 0);
    state = setup(); value.steering_permille = -1000;
    send(value); tick(&state, 0, 0);
    CHECK(pwm_left == 0 && pwm_right > 0);
}

static void test_physical_api_and_history(void)
{
    pid_state_t state = setup();
    motion_command_t queued;
    motion_status_snapshot_t result = {0}, copy;
    uint32_t id;
    CHECK(motion_move_cm(NAN, 1000, MOTION_DIR_FORWARD, &id) == E_PAR);
    CHECK(id == 0);
    CHECK(motion_move_cm(INFINITY, 1000, MOTION_DIR_FORWARD, &id) == E_PAR);
    CHECK(motion_move_cm(-1, 1000, MOTION_DIR_FORWARD, &id) == E_PAR);
    CHECK(motion_move_cm(10, 1000, MOTION_DIR_REVERSE, &id) == E_OK);
    CHECK(motion_receive_command(&queued) == E_OK);
    CHECK(queued.direction == MOTION_DIR_REVERSE);
    CHECK(queued.target_distance_counts > 1200);
    CHECK(queued.target_distance_counts < 1250);
    CHECK(pid_deadline(&queued) > 4000);
    result.command_id = id; result.status = MOTION_STATUS_COMPLETED;
    CHECK(motion_update_status(&result) == E_OK);
    result.status = MOTION_STATUS_CANCELLED;
    CHECK(motion_update_status(&result) == E_OK);
    CHECK(motion_get_result(id, &copy) == E_OK);
    CHECK(copy.status == MOTION_STATUS_COMPLETED);
    CHECK(motion_turn_degrees(180, 1000, MOTION_TURN_RIGHT, &id) == E_OK);
    tick(&state, 0, 0);
    CHECK(state.command.turn_direction == MOTION_TURN_RIGHT);
    CHECK(state.maximum_interval_ms == 10);
}

int main(void)
{
    motion_command_t hardware_test;
    motion_status_snapshot_t test_result = {0};
    hardware_test = b2_test_command(1);
    CHECK(hardware_test.type == MOTION_CMD_MOVE);
    CHECK(hardware_test.direction == MOTION_DIR_FORWARD);
    CHECK(hardware_test.target_distance_counts > 1200);
    hardware_test = b2_test_command(2);
    CHECK(hardware_test.direction == MOTION_DIR_REVERSE);
    hardware_test = b2_test_command(3);
    CHECK(hardware_test.type == MOTION_CMD_DRIVE);
    hardware_test = b2_test_command(4);
    CHECK(hardware_test.type == MOTION_CMD_TURN);
    CHECK(hardware_test.turn_direction == MOTION_TURN_LEFT);
    hardware_test = b2_test_command(5);
    CHECK(hardware_test.turn_direction == MOTION_TURN_RIGHT);
    CHECK(hardware_test.target_angle_deg == 90);
    (void)setup();
    clearance = false;
    CHECK(!b2_test_drive(hardware_test, 123u));
    CHECK(pwm_left == 0 && pwm_right == 0);
    test_result.command_id = 500u;
    test_result.status = MOTION_STATUS_CANCELLED;
    CHECK(motion_update_status(&test_result) == E_OK);
    CHECK(!b2_test_wait_result(500u));
    test_result.command_id = 501u;
    test_result.status = MOTION_STATUS_COMPLETED;
    test_result.physically_stopped = true;
    CHECK(motion_update_status(&test_result) == E_OK);
    CHECK(b2_test_wait_result(501u));
    test_profiles(); test_validation_and_queue();
    test_move_and_settle(false); test_move_and_settle(true);
    test_turn(false); test_turn(true); test_guards();
    test_results_and_stale(); test_feedback_and_time();
    test_timeouts_and_standstill(); test_stop_and_steering();
    test_physical_api_and_history();
    printf("PASS: %u subsystem 2 host assertions\n", checks);
    return 0;
}
