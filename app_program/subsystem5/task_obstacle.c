#include "subsystem5.h"
#include "s5_logic.h"
#include "s5_sim.h"
#include "s5_bridge.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "obstacle_scan.h"

#if S5_SIMULATION
#include "s5_pico_demo.h"
#endif

#if !S5_SIMULATION && S5_HARDWARE_ENABLED
static void print_scan(const char *label, const S5Scan *s)
{
    int i;
    for (i = 0; i < S5_SCAN_COUNT; ++i) {
        const S5ScanPoint *p = &s->points[i];
        tm_printf((UB*)"[S5-%s] angle=%d valid=%d cm_x100=%d\n",
                  label, (int)p->angle_deg, (int)p->valid,
                  (int)(p->distance_cm * 100.0f));
    }
}
static void print_profile(const S5Profile *p)
{
    tm_printf((UB*)"[S5-PROFILE] valid=%d width_cm_x100=%d nearest_cm_x100=%d bearing_deg=%d center_x_cm_x100=%d center_y_cm_x100=%d left_cm_x100=%d right_cm_x100=%d required_cm_x100=%d side=%d\n",
              (int)p->valid, (int)(p->estimated_width_cm * 100.0f),
              (int)(p->nearest_cm * 100.0f), (int)p->nearest_angle_deg,
              (int)(p->center_x_cm * 100.0f), (int)(p->center_y_cm * 100.0f),
              (int)(p->left_clearance_cm * 100.0f),
              (int)(p->right_clearance_cm * 100.0f),
              (int)(p->required_lateral_clearance_cm * 100.0f),
              (int)p->bypass_side);
}

#if S5_MOTION_BRIDGE_ENABLED
static void emergency_halt(const char *reason)
{
    tm_printf((UB*)"[S5-FAULT] %s; requesting emergency stop\n", reason);
    s5_bridge_emergency_stop();
    s5_servo_angle(90u);
    for (;;) tk_dly_tsk(1000);
}

#endif /* S5_MOTION_BRIDGE_ENABLED */


#if !S5_SERVO_ENABLED
/* HC-SR04P sensor-only bench test; does not drive servo or motors.
 * Sensor VCC must connect to 3V3 so ECHO remains 3.3V safe.
 */
static void sensor_only_mode(void)
{
    unsigned invalid_count = 0u;
    tm_printf((UB*)"\n[S5-HCSR04P] SENSOR TEST: TRIG=GP16 ECHO=GP17 VCC=3V3 GND=GND\n");
    tm_printf((UB*)"[S5-HCSR04P] Servo and motor commands are DISABLED\n");
    tm_printf((UB*)"[S5-HCSR04P] Move an object in front of the sensor to check readings\n");
    for (;;) {
        float cm = 0.0f;
        bool valid = s5_ultrasonic_read_cm(&cm);
        if (valid && cm >= 2.0f && cm <= S5_MAX_RANGE_CM) {
            invalid_count = 0u;
            tm_printf((UB*)"[S5-HCSR04P] distance_cm_x100=%d (%s)\n",
                      (int)(cm * 100.0f),
                      cm < S5_STOP_CM ? "WITHIN STOP THRESHOLD" : "outside stop threshold");
        } else {
            invalid_count++;
            tm_printf((UB*)"[S5-HCSR04P] NO VALID ECHO (count=%d); NOT safe to assume clear\n",
                      (int)invalid_count);
        }
        tk_dly_tsk(500);
    }
}
#endif /* !S5_SERVO_ENABLED */

#if S5_SERVO_ENABLED && S5_SERVO_SWEEP_TEST && !S5_MOTION_BRIDGE_ENABLED
/* Stationary hardware validation. No motor commands or motion bridge.
 * Servo signal GP12, HC-SR04P TRIG GP16, ECHO GP17.
 * Power servo from an adequate external supply, sharing ground with Pico.
 */
static void servo_sweep_test_mode(void)
{
    static const uint16_t angles[] = {30u, 60u, 90u, 120u, 150u, 120u, 90u, 60u};
    unsigned i;
    tm_printf((UB*)"\n[S5-SERVO] SERVO + HC-SR04P BENCH TEST\n");
    tm_printf((UB*)"[S5-SERVO] Servo signal GP12, TRIG GP16, ECHO GP17; motors DISABLED\n");
    tm_printf((UB*)"[S5-SERVO] Servo needs adequate power and common GND with Pico\n");
    for (;;) {
        for (i = 0u; i < sizeof(angles)/sizeof(angles[0]); ++i) {
            float cm = 0.0f;
            bool valid;
            s5_servo_angle(angles[i]);
            tk_dly_tsk(350); /* Wait for servo to reach target before echo. */
            valid = s5_ultrasonic_read_cm(&cm);
            if (valid && cm >= 2.0f && cm <= S5_MAX_RANGE_CM) {
                tm_printf((UB*)"[S5-SERVO] angle=%d distance_cm_x100=%d\n",
                          (int)angles[i], (int)(cm * 100.0f));
            } else {
                tm_printf((UB*)"[S5-SERVO] angle=%d NO VALID ECHO (not clear)\n",
                          (int)angles[i]);
            }
            tk_dly_tsk(150);
        }
        s5_servo_angle(90u);
        tk_dly_tsk(800);
    }
}
#endif

#if !S5_MOTION_BRIDGE_ENABLED
/* Real sensor + servo scan/profiling test, with motor bridge disabled.
 * Operates the physical HC-SR04P on GP16/GP17 and servo on GP12.
 * This is real hardware sensing, NOT autonomous vehicle motion.
 * Do not enable motors without Buddy 2/3 integration and emergency stop.
 */
static void stationary_profile_mode(void)
{
    unsigned cycle = 0u;
    tm_printf((UB*)"[S5-HARDWARE] REAL OBSTACLE SCANNING: TRIG=GP16 ECHO=GP17 SERVO=GP12\n");
    tm_printf((UB*)"[S5-HARDWARE] Coarse + fine scans and obstacle profile; MOTORS DISABLED\n");
    tm_printf((UB*)"[S5-HARDWARE] Point sensor toward a wall or move objects through its field of view\n");
    for (;;) {
        S5Scan coarse = {0}, fine = {0};
        S5Side side;
        S5Profile profile;
        uint16_t center;
        float front_cm = 0.0f;
        bool front_valid;
        s5_servo_angle(90u);
        tk_dly_tsk(350);
        front_valid = s5_ultrasonic_read_cm(&front_cm);
        tm_printf((UB*)"[S5-HARDWARE] cycle=%d front_valid=%d front_cm_x100=%d\n",
                  (int)++cycle, (int)front_valid, (int)(front_cm * 100.0f));
        if (!front_valid || front_cm < 2.0f || front_cm > S5_MAX_RANGE_CM) {
            tm_printf((UB*)"[S5-FAULT] 0xA1 NO VALID FRONT ECHO; do not assume clear\n");
        }
        s5_coarse_scan(&coarse);
        print_scan("COARSE", &coarse);
        side = s5_choose_side(&coarse);
        if (coarse.nearest_index >= 0) {
            center = coarse.points[coarse.nearest_index].angle_deg;
            s5_fine_scan(&fine, center);
            print_scan("FINE", &fine);
            profile = s5_estimate_profile(&fine, side);
            print_profile(&profile);
            tm_printf((UB*)"[S5-HARDWARE] suggested_side=%d (0=none,1=left,2=right); NO MOTOR ACTION\n",
                      (int)side);
            if (side == S5_SIDE_NONE)
                tm_printf((UB*)"[S5-HARDWARE] NO SAFE SIDE CONFIRMED; STOP REQUIRED IF MOVING\n");
        } else {
            tm_printf((UB*)"[S5-FAULT] 0xA1 NO VALID COARSE ECHO; no route conclusion\n");
        }
        s5_servo_angle(90u);
        tk_dly_tsk(1000);
    }
}

#endif /* !S5_MOTION_BRIDGE_ENABLED */
#if S5_MOTION_BRIDGE_ENABLED
/* Actual controller: synchronous scans are converted into ready events on
 * the next tick; motion/line events must come from real team bridge. */
static void live_controller_mode(void)
{
    S5Controller c;
    S5Scan coarse = {0}, fine = {0};
    bool coarse_ready = false, fine_ready = false;
    unsigned invalid_motion_echo = 0;
    bool stop_requested = false;
    if (!s5_bridge_ready()) emergency_halt("Buddy 2/3 motion bridge unavailable");
    s5_controller_init(&c);
    tm_printf((UB*)"[S5] LIVE CONTROLLER GP16=TRIG GP17=ECHO GP12=SERVO\n");
    for (;;) {
        S5Input in = {0};
        S5Command cmd;
        float cm = 0.0f;
        /* Sensor always faces forward outside scans. Monitoring continues
         * during movement. Repeated invalid readings fail closed. */
        in.front_valid = s5_ultrasonic_read_cm(&cm);
        in.front_cm = cm;
        if (c.state == S5_AVOID || c.state == S5_RECOVER || c.state == S5_REVERSE) {
            if (!in.front_valid) {
                if (++invalid_motion_echo >= S5_FRONT_INVALID_LIMIT)
                    emergency_halt("echo invalid during motion");
            } else {
                invalid_motion_echo = 0;
                if (cm < 10.0f) emergency_halt("very close obstacle during motion");
            }
        } else invalid_motion_echo = 0;
        in.stopped = s5_bridge_stopped();
        in.motion_complete = s5_bridge_motion_complete();
        in.line_found = s5_bridge_line_found();
        in.coarse_ready = coarse_ready;
        in.fine_ready = fine_ready;
        if (coarse_ready) in.coarse = coarse;
        if (fine_ready) in.fine = fine;
        coarse_ready = fine_ready = false;
        cmd = s5_controller_step(&c, &in);
        if (cmd != S5_CMD_NONE) {
            tm_printf((UB*)"[S5] state=%s command=%s error=%s\n",
                      s5_state_name(c.state), s5_command_name(cmd),
                      s5_error_name(c.error));
        }
        switch (cmd) {
        case S5_CMD_NONE: break;
        case S5_CMD_SCAN_COARSE:
            s5_coarse_scan(&coarse);
            print_scan("COARSE", &coarse);
            coarse_ready = true;
            break;
        case S5_CMD_SCAN_FINE:
            s5_fine_scan(&fine, c.fine_center_deg);
            print_scan("FINE", &fine);
            fine_ready = true;
            break;
        case S5_CMD_HALT:
            emergency_halt(s5_error_name(c.error));
            break;
        case S5_CMD_STOP:
            if (!stop_requested) {
                if (!s5_bridge_request(cmd, &c.profile))
                    emergency_halt("stop request rejected");
                stop_requested = true;
            }
            break;
        case S5_CMD_GO_LEFT:
        case S5_CMD_GO_RIGHT:
            print_profile(&c.profile);
            /* fall through */
        default:
            if (!s5_bridge_request(cmd, &c.profile))
                emergency_halt("motion or line command rejected");
            if (cmd == S5_CMD_RESUME) {
                /* Resume is a completed handoff, not a permanent DONE state. */
                s5_controller_init(&c);
                stop_requested = false;
            }
            break;
        }
        tk_dly_tsk(100);
    }
}
#endif /* S5_MOTION_BRIDGE_ENABLED */
#endif /* hardware mode */

EXPORT void s5_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
#if S5_SIMULATION
    s5_pico_demo_run_forever();
#elif S5_HARDWARE_ENABLED
    s5_ultrasonic_init();
#if S5_SERVO_ENABLED
    s5_servo_init();
    tk_dly_tsk(600);
#if S5_MOTION_BRIDGE_ENABLED
    live_controller_mode();
#elif S5_SERVO_SWEEP_TEST
    servo_sweep_test_mode();
#else
    stationary_profile_mode();
#endif
#else
    sensor_only_mode();
#endif
#else
    tm_printf((UB*)"[S5] hardware/servo disabled\n");
    for (;;) tk_dly_tsk(1000);
#endif
}
