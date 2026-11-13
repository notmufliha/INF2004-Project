#include "../subsystem2/host/mock_platform.h"
#include <assert.h>
#include <stdio.h>
#define SUBSYSTEM5_H
#define EXPORT
#include "../../app_program/subsystem5/s5_mission.c"
#include "../../app_program/subsystem2/b2_motor.c"
static uint32_t clock_ms = 100u;
static int left_pwm, right_pwm;
ER tk_get_tim(SYSTIM *time) { time->lo = clock_ms; return E_OK; }
void motor_stop(void) { left_pwm = right_pwm = 0; }
void motor_left_set(int pwm) { left_pwm = pwm; }
void motor_right_set(int pwm) { right_pwm = pwm; }
INT tm_printf(UB *fmt, ...) { (void)fmt; return 0; }
int main(void)
{
    uint32_t generation;
    s5_mission_init();
    assert(!s5_mission_approach_slow());
    s5_mission_front(true, 30.0f);
    assert(s5_mission_approach_slow());
    s5_mission_front(false, 0.0f);
    assert(s5_mission_approach_slow()); /* Missing data cannot accelerate the approach. */
    s5_mission_front(true, 31.0f);
    assert(!s5_mission_approach_slow());
    s5_mission_front(false, 20.0f);
    assert(!s5_mission_approach_slow());
    assert(b2_safety_init() == E_OK);
    b2_safety_acknowledge_stop();
    b2_request_forward();
    assert(!b2_safety_motion_allowed());
    s5_mission_enable(true);
    b2_request_forward();
    generation = b2_safety_stop_generation();
    assert(b2_safety_motion_allowed());
    b2_safety_apply_pwm(25, 25, generation);
    assert(left_pwm == 25 && right_pwm == 25);
    s5_mission_publish(S5_MISSION_BUSY);
    assert(s5_mission_state() == S5_MISSION_BUSY);
    s5_mission_enable(false);
    assert(left_pwm == 0 && right_pwm == 0 && !b2_safety_motion_allowed());
    b2_safety_acknowledge_stop();
    b2_request_forward(); /* A delayed B5 tick cannot override mission STOP. */
    b2_safety_apply_pwm(25, 25, b2_safety_stop_generation());
    assert(left_pwm == 0 && right_pwm == 0);
    s5_mission_enable(true);
    b2_request_forward();
    b2_safety_apply_pwm(25, 25, generation);
    assert(left_pwm == 0 && right_pwm == 0);
    b2_safety_apply_pwm(25, 25, b2_safety_stop_generation());
    assert(left_pwm == 25 && right_pwm == 25);
    clock_ms += ROBOT_MOTOR_WATCHDOG_MS + 1u;
    assert(!b2_safety_motion_allowed());
    b2_safety_apply_pwm(25, 25, b2_safety_stop_generation());
    assert(left_pwm == 0 && right_pwm == 0);
    puts("PASS mission STOP gate, delayed B5 permission, stale commands and watchdog");
    return 0;
}
