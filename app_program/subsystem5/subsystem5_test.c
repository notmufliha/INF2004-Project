#include "subsystem5.h"
#include "s5_logic.h"
#include "s5_sim.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "obstacle_scan.h"
#include "../test_report.h"
#if APP_TEST
EXPORT void subsystem5_test(void)
{
    S5SimResult l = s5_sim_run(S5_SCENARIO_LEFT_CLEAR, 0, 0);
    S5SimResult r = s5_sim_run(S5_SCENARIO_RIGHT_CLEAR, 0, 0);
    S5SimResult b = s5_sim_run(S5_SCENARIO_BOTH_BLOCKED, 0, 0);
    S5SimResult rev = s5_sim_run(S5_SCENARIO_REVERSE_CLEARS, 0, 0);
    S5SimResult inv = s5_sim_run(S5_SCENARIO_INVALID_COARSE, 0, 0);
    S5SimResult stop = s5_sim_run(S5_SCENARIO_STOP_TIMEOUT, 0, 0);
    S5SimResult move = s5_sim_run(S5_SCENARIO_MOTION_TIMEOUT, 0, 0);
    S5SimResult back = s5_sim_run(S5_SCENARIO_REVERSE_TIMEOUT, 0, 0);
    S5SimResult line = s5_sim_run(S5_SCENARIO_LINE_TIMEOUT, 0, 0);
    S5SimResult clear = s5_sim_run(S5_SCENARIO_CLEAR_PATH, 0, 0);
    S5SimResult echo = s5_sim_run(S5_SCENARIO_FRONT_TIMEOUT, 0, 0);
    S5SimResult fine = s5_sim_run(S5_SCENARIO_FINE_INVALID, 0, 0);
    uint16_t angles[S5_SCAN_COUNT];
    s5_make_fine_angles(90u, angles);
    TEST_INIT();
    TEST_CHECK("S5-U01", s5_echo_us_to_cm(1000u) > 17.0f &&
               s5_echo_us_to_cm(1000u) < 17.3f, "echo conversion");
    TEST_CHECK("S5-U02", s5_servo_pulse_us(90u) == 1500u, "servo centre");
    TEST_CHECK("S5-U03", l.controller.state == S5_DONE && l.controller.side == S5_SIDE_LEFT, "left route");
    TEST_CHECK("S5-U04", r.controller.state == S5_DONE && r.controller.side == S5_SIDE_RIGHT, "right route");
    TEST_CHECK("S5-U05", b.controller.error == S5_ERR_NO_CLEAR_ROUTE && b.controller.reverse_attempts == 1u, "blocked after retry");
    TEST_CHECK("S5-U06", rev.controller.state == S5_DONE && rev.controller.reverse_attempts == 1u, "reverse and retry");
    TEST_CHECK("S5-U07", inv.controller.state == S5_DONE && inv.controller.side == S5_SIDE_RIGHT, "invalid side rejected");
    TEST_CHECK("S5-U08", stop.controller.error == S5_ERR_STOP_TIMEOUT, "stop timeout");
    TEST_CHECK("S5-U09", move.controller.error == S5_ERR_MOTION_TIMEOUT, "motion timeout");
    TEST_CHECK("S5-U10", back.controller.error == S5_ERR_REVERSE_TIMEOUT, "reverse timeout");
    TEST_CHECK("S5-U11", line.controller.error == S5_ERR_LINE_TIMEOUT, "line timeout");
    TEST_CHECK("S5-U12", clear.controller.state == S5_FOLLOW, "continue clear path");
    TEST_CHECK("S5-U13", echo.controller.error == S5_ERR_FRONT_SENSOR, "invalid front sensor");
    TEST_CHECK("S5-U14", fine.controller.error == S5_ERR_FINE_SCAN, "invalid fine scan");
    TEST_CHECK("S5-U15", l.controller.profile.valid && l.controller.profile.estimated_width_cm > 10.0f &&
               l.controller.profile.estimated_width_cm < 12.0f, "geometric width estimate");
    TEST_CHECK("S5-U16", l.controller.profile.nearest_cm == 15.0f &&
               l.controller.profile.nearest_angle_deg == 90u, "closest point");
    TEST_CHECK("S5-U17", l.controller.profile.left_clearance_cm == 90.0f &&
               l.controller.profile.right_clearance_cm == 65.0f, "both clearances");
    TEST_CHECK("S5-U18", angles[0] == 70u && angles[4] == 110u, "adaptive fine angles");
    TEST_CHECK("S5-U19", l.controller.profile.required_lateral_clearance_cm >
               l.controller.profile.estimated_width_cm / 2.0f, "margin added");
    TEST_CHECK("S5-U20", l.controller.profile.center_x_cm > -0.1f &&
               l.controller.profile.center_x_cm < 0.1f &&
               l.controller.profile.center_y_cm > 14.9f &&
               l.controller.profile.center_y_cm < 15.1f,
               "obstacle location relative to sensor");
    TEST_SUMMARY("subsystem5");
}
#endif
