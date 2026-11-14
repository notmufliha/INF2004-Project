#define main legacy_controller_main
#include "controller_test.c"
#undef main
int main(void)
{
    VEHICLE_CONTROLLER c = fresh();
    assert(ROBOT_OBSTACLE_TEST == 1);
    command(&c, RCMD_START, 0u);
    input.line_valid = false;
    for (unsigned i = 0; i < 100u; ++i) {
        input.line_flags = i % 2 ? LINE_F_DETECTED : LINE_F_LOST;
        step(&c, 20u, B3_NAV_RIGHT);
        assert(c.running && sent.type == MOTION_CMD_DRIVE);
        assert(sent.direction == MOTION_DIR_FORWARD && sent.steering_permille == 0);
        assert(sent.target_speed_cps <= ROBOT_BYPASS_SPEED_CPS);
    }
    input.obstacle_active = true;
    unsigned before = drives;
    step(&c, 20u, 0u);
    assert(drives == before && c.obstacle_handoff);
    input.obstacle_active = false;
    input.clearance = false;
    step(&c, 20u, 0u);
    assert(c.stopped);
    input.clearance = true;
    step(&c, 20u, 0u);
    assert(sent.steering_permille == 0);
    command(&c, RCMD_STOP, 0u);
    before = drives;
    step(&c, 20u, 0u);
    assert(!c.running && drives == before);
    c = fresh(); command(&c, RCMD_START, 0u);
    input.imu_valid = false;
    step(&c, 20u, 0u);
    assert(!c.running && c.stopped);
    /* Boot while B5 scans an obstacle: arm the mission without front DRIVE
     * clearance, and leave the bypass commands alone. */
    c = fresh(); (void)c;
    cleared_faults = 0u;
    obstacle_state = S5_MISSION_BUSY;
    permission = false;
    clock_ms = 30000u;
    task_ticks = remote_head = remote_count = 0u;
    force_no_line = true;
    stale_line = stale_imu = delayed_startup = false;
    if (setjmp(task_done) == 0) controller_task(0, NULL);
    assert(cleared_faults == (FAULT_MOTOR_STALL | FAULT_ENCODER));
    assert(boot_mission_enabled && drives == 0u);
    puts("PASS obstacle test: straight drive, bypass handoff, STOP/IMU stop, autostart during obstacle scan");
    return 0;
}
