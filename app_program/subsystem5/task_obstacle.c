#include "subsystem5.h"
#include "s5_sim.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "obstacle_scan.h"
#if S5_SIMULATION
static void s5_print_trace(S5Scenario scenario, unsigned step,
                           S5State state, S5Command cmd, void *unused)
{
    (void)unused;
    tm_printf((UB*)"[S5-SIM] %s step=%d state=%s command=%s\n",
              s5_scenario_name(scenario), (int)step,
              s5_state_name(state), s5_command_name(cmd));
}
#endif
EXPORT void s5_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
#if S5_SIMULATION
    int i;
    tm_printf((UB*)"[S5-SIM] SOFTWARE DEMO ONLY: NO PHYSICAL MOVEMENT\n");
    for (i = 0; i < (int)S5_SCENARIO_COUNT; ++i) {
        S5SimResult r = s5_sim_run((S5Scenario)i, s5_print_trace, 0);
        const S5Profile *p = &r.controller.profile;
        tm_printf((UB*)"[S5-PROFILE] %s valid=%d nearest_cm_x100=%d angle=%d width_cm_x100=%d left_cm_x100=%d right_cm_x100=%d required_cm_x100=%d\n",
                  s5_scenario_name((S5Scenario)i), (int)p->valid,
                  (int)(p->nearest_cm * 100.0f), (int)p->nearest_angle_deg,
                  (int)(p->estimated_width_cm * 100.0f),
                  (int)(p->left_clearance_cm * 100.0f),
                  (int)(p->right_clearance_cm * 100.0f),
                  (int)(p->required_lateral_clearance_cm * 100.0f));
        tm_printf((UB*)"[S5-SIM] result=%s error=%s reverses=%d\n",
                  s5_state_name(r.controller.state),
                  s5_error_name(r.controller.error), (int)r.controller.reverse_attempts);
        tk_dly_tsk(50);
    }
    for (;;) tk_dly_tsk(1000);
#elif S5_HARDWARE_ENABLED
    /* No real motors are commanded. Hardware integration is a separate step. */
    float cm;
    s5_ultrasonic_init();
#if S5_SERVO_ENABLED
    s5_servo_init();
    tk_dly_tsk(500);
#endif
    for (;;) {
        if (s5_ultrasonic_read_cm(&cm))
            tm_printf((UB*)"[S5] distance_cm_x100=%d\n", (int)(cm * 100.0f));
        else tm_printf((UB*)"[S5] invalid echo: unsafe to assume clearance\n");
        tk_dly_tsk(250);
    }
#else
    for (;;) tk_dly_tsk(1000);
#endif
}
