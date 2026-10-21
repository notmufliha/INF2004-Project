/* Buddy 2 motor owner: PWM is written ONLY here. The B5 task sends a
 * forward/stop heartbeat. Stale commands, invalid sensor data and power-up
 * all produce zero duty. Single-core RTOS build only.
 */
#include "b2_motor.h"
#include "../robot_config.h"
#include <sys/sysdef.h>
#include <tk/syslib.h>

#if TK_SUPPORT_SMP
#error "Motor request mailbox requires SMP=0."
#endif

#define B2_CMD_STOP 0u
#define B2_CMD_FORWARD 1u
#define RESETS_RESET_REG  (*(volatile UW *)0x4000C000u)
#define RESETS_DONE_REG   (*(volatile UW *)0x4000C008u)
#define PWM_RESET_MASK (1u << 14u)

static volatile uint32_t request_word = 0u;
static volatile uint32_t motor_stopped = 1u;
static bool initialized = false;
static uint32_t millis(void) { SYSTIM t; tk_get_tim(&t); return (uint32_t)t.lo; }
static void publish(uint32_t cmd)
{
    uint32_t old = request_word;
    request_word = ((old + 0x100u) & ~0xffu) | cmd;
}
void b2_request_stop(void) { publish(B2_CMD_STOP); }
void b2_request_forward(void)
{
#if ROBOT_ENABLE_DRIVE && !ROBOT_WHEELS_UP_MOTOR_TEST
    publish(B2_CMD_FORWARD);
#else
    publish(B2_CMD_STOP);
#endif
}
bool b2_motor_is_stopped(void) { return motor_stopped != 0u; }
static void stop_pwm(void)
{
    pwm_set_cc(ROBOT_MOTOR_L_FWD_PIN, 0u);
    pwm_set_cc(ROBOT_MOTOR_L_REV_PIN, 0u);
    pwm_set_cc(ROBOT_MOTOR_R_FWD_PIN, 0u);
    pwm_set_cc(ROBOT_MOTOR_R_REV_PIN, 0u);
    motor_stopped = 1u;
}
static void wheels(uint32_t l, uint32_t r)
{
    if (l > ROBOT_PWM_TOP) l = ROBOT_PWM_TOP;
    if (r > ROBOT_PWM_TOP) r = ROBOT_PWM_TOP;
    /* Break-before-make: no forward/reverse overlap on either motor. */
    pwm_set_cc(ROBOT_MOTOR_L_REV_PIN, 0u);
    pwm_set_cc(ROBOT_MOTOR_R_REV_PIN, 0u);
    pwm_set_cc(ROBOT_MOTOR_L_FWD_PIN, l);
    pwm_set_cc(ROBOT_MOTOR_R_FWD_PIN, r);
    motor_stopped = (l == 0u && r == 0u);
}
static bool pwm_init(void)
{
    static const uint32_t pins[4] = {
        ROBOT_MOTOR_L_FWD_PIN, ROBOT_MOTOR_L_REV_PIN,
        ROBOT_MOTOR_R_FWD_PIN, ROBOT_MOTOR_R_REV_PIN
    };
    uint32_t limit = 1000000u;
    int i;
    /* The board's PWM peripheral may be held in reset on power-on. */
    RESETS_RESET_REG &= ~PWM_RESET_MASK;
    while (!(RESETS_DONE_REG & PWM_RESET_MASK)) {
        if (--limit == 0u) return false;
    }
    for (i = 0; i < 4; ++i)
        if (pwm_set_enabled(pins[i], FALSE) != E_OK) return false;
    for (i = 0; i < 4; ++i) {
        if (pwm_set_wrap(pins[i], ROBOT_PWM_TOP) != E_OK) return false;
        if (pwm_set_cc(pins[i], 0u) != E_OK) return false;
    }
    /* 125 MHz / 125 / 1000 = 1 kHz. GP8/9 are PWM slice 4,
       GP10/11 slice 5, GP12 servo slice 6 (no timer conflict). */
    out_w(PWM_BASE + (((ROBOT_MOTOR_L_FWD_PIN >> 1u) & 7u) * 0x14u) + PWM_CHx_DIV, 125u << 4u);
    out_w(PWM_BASE + (((ROBOT_MOTOR_R_FWD_PIN >> 1u) & 7u) * 0x14u) + PWM_CHx_DIV, 125u << 4u);
    for (i = 0; i < 4; ++i)
        if (pwm_set_pin(pins[i]) != E_OK) return false;
    for (i = 0; i < 4; ++i)
        if (pwm_set_enabled(pins[i], TRUE) != E_OK) return false;
    stop_pwm();
    return true;
}
void b2_motor_task(INT stacd, void *exinf)
{
    uint32_t last_word = 0u, last_forward = 0u;
    bool was_running = false;
    (void)stacd; (void)exinf;
    initialized = pwm_init();
    if (!initialized) {
        tm_printf((UB*)"[B2-FAULT] PWM init failed; MOTOR LOCKED OFF\n");
        for (;;) tk_dly_tsk(1000);
    }
    tm_printf((UB*)"[B2] PWM READY M1 GP%d/GP%d M2 GP%d/GP%d duty=%d/999 drive=%d diagnostic=%d\n",
              (int)ROBOT_MOTOR_L_FWD_PIN, (int)ROBOT_MOTOR_L_REV_PIN,
              (int)ROBOT_MOTOR_R_FWD_PIN, (int)ROBOT_MOTOR_R_REV_PIN,
              (int)ROBOT_FORWARD_PWM, (int)ROBOT_ENABLE_DRIVE,
              (int)ROBOT_WHEELS_UP_MOTOR_TEST);
#if ROBOT_WHEELS_UP_MOTOR_TEST
    tm_printf((UB*)"[B2-TEST] WHEELS MUST BE RAISED. Starting in 3 seconds\n");
    tk_dly_tsk(3000);
    tm_printf((UB*)"[B2-TEST] LEFT only\n");
    wheels(ROBOT_MOTOR_TEST_PWM, 0u);
    tk_dly_tsk(ROBOT_MOTOR_TEST_PULSE_MS); stop_pwm(); tk_dly_tsk(700);
    tm_printf((UB*)"[B2-TEST] RIGHT only\n");
    wheels(0u, ROBOT_MOTOR_TEST_PWM);
    tk_dly_tsk(ROBOT_MOTOR_TEST_PULSE_MS); stop_pwm(); tk_dly_tsk(700);
    tm_printf((UB*)"[B2-TEST] BOTH forward\n");
    wheels(ROBOT_MOTOR_TEST_PWM, ROBOT_MOTOR_TEST_PWM);
    tk_dly_tsk(ROBOT_MOTOR_TEST_PULSE_MS); stop_pwm();
    tm_printf((UB*)"[B2-TEST] FINISHED. Motors will stay off until reflashed\n");
    for (;;) { stop_pwm(); tk_dly_tsk(1000); }
#else
    tk_dly_tsk(ROBOT_BOOT_DELAY_MS);
    tm_printf((UB*)"[B2] WAITING_FOR_CLEAR: motors OFF until Buddy 5 sends FORWARD\n");
    for (;;) {
        uint32_t now = millis();
        uint32_t word = request_word;
        uint32_t cmd = word & 0xffu;
        if (word != last_word) {
            last_word = word;
            if (cmd == B2_CMD_FORWARD) last_forward = now;
        }
        if (cmd != B2_CMD_FORWARD ||
            (uint32_t)(now - last_forward) > ROBOT_MOTOR_WATCHDOG_MS) {
            if (was_running) {
                stop_pwm(); was_running = false;
                tm_printf((UB*)"[B2] STOP: %s\n",
                          cmd != B2_CMD_FORWARD ? "Buddy 5 stop request" : "forward heartbeat expired");
            }
        } else if (!was_running) {
#if ROBOT_ENABLE_DRIVE
            wheels(ROBOT_FORWARD_PWM, ROBOT_FORWARD_PWM);
            was_running = true;
            tm_printf((UB*)"[B2] FORWARD left=%d right=%d\n",
                      (int)ROBOT_FORWARD_PWM, (int)ROBOT_FORWARD_PWM);
#endif
        }
        /* Do not repeatedly print STOPPED every five seconds. The motor
         * remains stopped until a real, fresh Buddy 5 FORWARD request.
         * Transitions are already logged in the branches above. */
        tk_dly_tsk(ROBOT_MOTOR_PERIOD_MS);
    }
#endif
}
