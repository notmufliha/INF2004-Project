/* Reuse the deterministic B2/B3/IMU fakes, exercise production default policy. */
#define main legacy_controller_main
#include "controller_test.c"
#undef main
int main(void)
{
    VEHICLE_CONTROLLER c = fresh();
    assert(ROBOT_CLEAR_PATH_FORWARD == 1);
    command(&c, RCMD_START, 0u);
    input.line_flags = LINE_F_LOST;
    for (unsigned i = 0; i < 1000u; ++i) {
        step(&c, 20u, 0u);
        assert(c.running && sent.type == MOTION_CMD_DRIVE);
        assert(sent.direction == MOTION_DIR_FORWARD && sent.steering_permille == 0);
    }
    INT cruise_speed = sent.target_speed_cps;
    input.approach_slow = true;
    step(&c, 20u, 0u);
    assert(sent.target_speed_cps == VC_MIN_WHEEL_CPS && sent.target_speed_cps < cruise_speed);
    input.approach_slow = false;
    step(&c, 20u, 0u);
    assert(sent.target_speed_cps == cruise_speed);
    input.line_flags = LINE_F_DETECTED;
    input.left = 1000u; input.centre = 0u;
    step(&c, 20u, 0u);
    assert(sent.steering_permille != 0); /* Tape restores steering. */
    input.line_flags = LINE_F_LOST;
    step(&c, 20u, 0u);
    assert(sent.steering_permille == 0);
    input.clearance = false;
    step(&c, 20u, 0u);
    assert(c.stopped);
    command(&c, RCMD_STOP, 0u);
    input.clearance = true;
    unsigned before = drives;
    step(&c, 20u, 0u);
    assert(!c.running && drives == before);
    c = fresh(); command(&c, RCMD_START, 0u);
    input.imu_valid = false;
    step(&c, 20u, 0u);
    assert(!c.running && c.stopped);
    c = fresh(); (void)c;
    force_no_line = true;
    clock_ms = 30000u;
    task_ticks = remote_head = remote_count = 0u;
    stale_line = stale_imu = delayed_startup = false;
    permission = false;
    if (setjmp(task_done) == 0) controller_task(0, NULL);
    assert(drives > 0u && sent.steering_permille == 0);
    puts("PASS default clear-forward: no off-line timeout, line steering returns, obstacle/STOP/IMU stop");
    return 0;
}
