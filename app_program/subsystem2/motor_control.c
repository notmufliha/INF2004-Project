/*
 * Buddy 2 - Motion Control
 * Basic left motor test using micro T-Kernel BSP.
 */

#include "../app.h"
#include "motor_control.h"

#define MOTOR_L_A_PIN   8U      // Left motor PWM output A
#define MOTOR_L_B_PIN   9U      // Left motor PWM output B
#define MOTOR_R_A_PIN   10U     // Right motor PWM output A
#define MOTOR_R_B_PIN   11U     // Right motor PWM output B

#define MOTOR_PWM_WRAP  999U
#define MOTOR_TEST_DUTY 50

/* RP2040 peripheral reset control. */
#define RESETS_RESET_REG  (*(volatile UW *)0x4000C000U)
#define RESETS_DONE_REG   (*(volatile UW *)0x4000C008U)

#define PWM_RESET_MASK    (1U << 14)
#define PWM_RESET_TIMEOUT 1000000U

/* Indicates whether motor PWM initialisation succeeded. */
static BOOL motor_ready = FALSE;

/*
 * Stop both motors.
 */
void motor_stop(void)
{
    pwm_set_cc(MOTOR_L_A_PIN, 0U);
    pwm_set_cc(MOTOR_L_B_PIN, 0U);

    pwm_set_cc(MOTOR_R_A_PIN, 0U);
    pwm_set_cc(MOTOR_R_B_PIN, 0U);
}

/*
 * Initialise left and right motor PWM.
 */

void motor_init(void)
{
    UW timeout = PWM_RESET_TIMEOUT;

    motor_ready = FALSE;

    /* Release the PWM peripheral from reset. */
    RESETS_RESET_REG &= ~PWM_RESET_MASK;

    while ((RESETS_DONE_REG & PWM_RESET_MASK) == 0U) {
        if (timeout == 0U) {
            tm_printf((UB*)"[B2] PWM reset timeout\n");
            return;
        }

        timeout--;
    }

    /* Disable both PWM slices before configuration. */
    if (pwm_set_enabled(MOTOR_L_A_PIN, FALSE) != E_OK ||
        pwm_set_enabled(MOTOR_R_A_PIN, FALSE) != E_OK) {
        tm_printf((UB*)"[B2] PWM disable failed\n");
        return;
    }

    /* Configure PWM periods for both motors. */
    if (pwm_set_wrap(MOTOR_L_A_PIN, MOTOR_PWM_WRAP) != E_OK ||
        pwm_set_wrap(MOTOR_R_A_PIN, MOTOR_PWM_WRAP) != E_OK) {
        tm_printf((UB*)"[B2] PWM wrap configuration failed\n");
        return;
    }

    /* Clear all motor outputs. */
    motor_stop();

    /* Configure motor GPIO pins for PWM. */
    if (pwm_set_pin(MOTOR_L_A_PIN) != E_OK ||
        pwm_set_pin(MOTOR_L_B_PIN) != E_OK ||
        pwm_set_pin(MOTOR_R_A_PIN) != E_OK ||
        pwm_set_pin(MOTOR_R_B_PIN) != E_OK) {
        tm_printf((UB*)"[B2] PWM pin configuration failed\n");
        return;
    }

    /* Enable both PWM slices. */
    if (pwm_set_enabled(MOTOR_L_A_PIN, TRUE) != E_OK ||
        pwm_set_enabled(MOTOR_R_A_PIN, TRUE) != E_OK) {
        tm_printf((UB*)"[B2] PWM enable failed\n");
        return;
    }

    motor_ready = TRUE;

    tm_printf((UB*)"[B2] Both motor PWM channels initialized\n");
}

/*
 * Control left motor speed.
 *
 * Positive: PWM on GP8
 * Negative: PWM on GP9
 * Zero:     Both outputs off
 *
 * Actual wheel direction depends on motor wiring.
 */

void motor_left_set(int speed_percent)
{
    UW duty;

    if (!motor_ready) {
        return;
    }

    if (speed_percent > 100) {
        speed_percent = 100;
    }

    if (speed_percent < -100) {
        speed_percent = -100;
    }

    if (speed_percent < 0) {
        duty = (UW)(-speed_percent);
    } else {
        duty = (UW)speed_percent;
    }

    duty = (duty * MOTOR_PWM_WRAP) / 100U;

    /* Stop only the left motor before updating PWM. */
    pwm_set_cc(MOTOR_L_A_PIN, 0U);
    pwm_set_cc(MOTOR_L_B_PIN, 0U);

    /* Positive = forward, negative = reverse. */
    if (speed_percent > 0) {
        pwm_set_cc(MOTOR_L_B_PIN, duty);
    } else if (speed_percent < 0) {
        pwm_set_cc(MOTOR_L_A_PIN, duty);
    }
}

/*
 * Control right motor PWM duty cycle.
 *
 * Positive: PWM on GP10 (M2A)
 * Negative: PWM on GP11 (M2B)
 *
 * Direction may need reversing after physical testing.
 */
void motor_right_set(int speed_percent)
{
    UW duty;

    if (!motor_ready) {
        return;
    }

    if (speed_percent > 100) {
        speed_percent = 100;
    }

    if (speed_percent < -100) {
        speed_percent = -100;
    }

    if (speed_percent < 0) {
        duty = (UW)(-speed_percent);
    } else {
        duty = (UW)speed_percent;
    }

    duty = (duty * MOTOR_PWM_WRAP) / 100U;

    /* Stop right motor before changing direction. */
    pwm_set_cc(MOTOR_R_A_PIN, 0U);
    pwm_set_cc(MOTOR_R_B_PIN, 0U);

    if (speed_percent > 0) {
        pwm_set_cc(MOTOR_R_A_PIN, duty);
    } else if (speed_percent < 0) {
        pwm_set_cc(MOTOR_R_B_PIN, duty);
    }
}

/*
 * Temporary dual-motor movement test.
 */
void motor_test_task(int stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    /* Allow time to connect serial monitor. */
    tk_dly_tsk(5000);

    if (!motor_ready) {
        tm_printf((UB*)"[B2] PWM not ready\n");
        tk_ext_tsk();
        return;
    }

    /* Test 1: Both motors forward. */
    tm_printf((UB*)"[B2] Both motors forward\n");

    motor_left_set(50);
    motor_right_set(50);

    tk_dly_tsk(2000);
    motor_stop();

    /* Pause before changing direction. */
    tk_dly_tsk(1000);

    /* Test 2: Both motors reverse. */
    tm_printf((UB*)"[B2] Both motors reverse\n");

    motor_left_set(-50);
    motor_right_set(-50);

    tk_dly_tsk(2000);
    motor_stop();

    tm_printf((UB*)"[B2] Dual-motor test completed\n");

    tk_ext_tsk();
}


/*
 * Buddy 2 - Motor Speed Baseline Test
 *
 * Runs both motors forward at 30% PWM
 * for 5 seconds, then stops automatically.
 */
void motor_baseline_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    tm_printf((UB*)"[B2] Motor baseline test starting\n");

    /* Allow time to prepare. */
    tk_dly_tsk(3000);

    tm_printf((UB*)"[B2] Running both motors at 30%% PWM\n");

    motor_left_set(30);
    motor_right_set(30);

    /* Run motors for 5 seconds. */
    tk_dly_tsk(5000);

    motor_stop();

    tm_printf((UB*)"[B2] Motors stopped - baseline complete\n");

    /* Keep the temporary test task alive after stopping. */
    while (1) {
        tk_dly_tsk(1000);
    }
}
