/*
 * Buddy 2 - Motion Control
 * Basic left motor test using micro T-Kernel BSP.
 */

#include <stdbool.h>
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

static bool gb_motor_ready = false;

static ER motor_init_fail(ER error);

/** @brief Set both motor PWM compare values to zero. */
void motor_stop(void)
{
    pwm_set_cc(MOTOR_L_A_PIN, 0U);
    pwm_set_cc(MOTOR_L_B_PIN, 0U);
    pwm_set_cc(MOTOR_R_A_PIN, 0U);
    pwm_set_cc(MOTOR_R_B_PIN, 0U);
}

/** @brief Initialize both motor PWM slices and their GPIO pins. */
ER motor_init(void)
{
    UW timeout = PWM_RESET_TIMEOUT;
    gb_motor_ready = false;
    /* Release the PWM peripheral from reset. */
    RESETS_RESET_REG &= ~PWM_RESET_MASK;

    while ((RESETS_DONE_REG & PWM_RESET_MASK) == 0U)
    {
        if (0U == timeout)
        {
            tm_printf((UB *)"[B2] PWM reset timeout\n");
            /*
             * PWM is not confirmed ready.
             * Do not access its registers during cleanup.
             */
            gb_motor_ready = false;
            return (E_TMOUT);
        }
        timeout--;
    }
    /* Disable both PWM slices before configuration. */
    if (E_OK != pwm_set_enabled(MOTOR_L_A_PIN, false) ||
        E_OK != pwm_set_enabled(MOTOR_R_A_PIN, false))
    {
        tm_printf((UB *)"[B2] PWM disable failed\n");
        return (motor_init_fail(E_OBJ));
    }
    /* Configure PWM periods for both motors. */
    if (E_OK != pwm_set_wrap(MOTOR_L_A_PIN, MOTOR_PWM_WRAP) ||
        E_OK != pwm_set_wrap(MOTOR_R_A_PIN, MOTOR_PWM_WRAP))
    {
        tm_printf((UB *)"[B2] PWM wrap configuration failed\n");
        return (motor_init_fail(E_OBJ));
    }
    /* Clear all motor outputs. */
    motor_stop();
    /* Configure motor GPIO pins for PWM. */
    if (E_OK != pwm_set_pin(MOTOR_L_A_PIN) ||
        E_OK != pwm_set_pin(MOTOR_L_B_PIN) ||
        E_OK != pwm_set_pin(MOTOR_R_A_PIN) ||
        E_OK != pwm_set_pin(MOTOR_R_B_PIN))
    {
        tm_printf((UB *)"[B2] PWM pin configuration failed\n");
        return (motor_init_fail(E_OBJ));
    }
    /* Enable both PWM slices. */
    if (E_OK != pwm_set_enabled(MOTOR_L_A_PIN, true) ||
        E_OK != pwm_set_enabled(MOTOR_R_A_PIN, true))
    {
        tm_printf((UB *)"[B2] PWM enable failed\n");
        return (motor_init_fail(E_OBJ));
    }
    gb_motor_ready = true;

    tm_printf((UB *)"[B2] Both motor PWM channels initialized\n");
    return (E_OK);
}

/** @brief Apply signed left motor duty: positive GP9, negative GP8. */
void motor_left_set(int speed_percent)
{
    UW duty;

    if (!gb_motor_ready)
    {
        return;
    }
    if (speed_percent > 100)
    {
        speed_percent = 100;
    }
    if (speed_percent < -100)
    {
        speed_percent = -100;
    }
    if (speed_percent < 0)
    {
        duty = (UW)(-speed_percent);
    }
    else
    {
        duty = (UW)speed_percent;
    }
    duty = (duty * MOTOR_PWM_WRAP) / 100U;
    /* Stop only the left motor before updating PWM. */
    pwm_set_cc(MOTOR_L_A_PIN, 0U);
    pwm_set_cc(MOTOR_L_B_PIN, 0U);
    /* Positive = forward, negative = reverse. */
    if (speed_percent > 0)
    {
        pwm_set_cc(MOTOR_L_B_PIN, duty);
    }
    else if (speed_percent < 0)
    {
        pwm_set_cc(MOTOR_L_A_PIN, duty);
    }
    else
    {
        /* No additional action is required. */
    }
}

/** @brief Apply signed right motor duty: positive GP10, negative GP11. */
void motor_right_set(int speed_percent)
{
    UW duty;

    if (!gb_motor_ready)
    {
        return;
    }
    if (speed_percent > 100)
    {
        speed_percent = 100;
    }
    if (speed_percent < -100)
    {
        speed_percent = -100;
    }
    if (speed_percent < 0)
    {
        duty = (UW)(-speed_percent);
    }
    else
    {
        duty = (UW)speed_percent;
    }
    duty = (duty * MOTOR_PWM_WRAP) / 100U;
    /* Stop right motor before changing direction. */
    pwm_set_cc(MOTOR_R_A_PIN, 0U);
    pwm_set_cc(MOTOR_R_B_PIN, 0U);
    if (speed_percent > 0)
    {
        pwm_set_cc(MOTOR_R_A_PIN, duty);
    }
    else if (speed_percent < 0)
    {
        pwm_set_cc(MOTOR_R_B_PIN, duty);
    }
    else
    {
        /* No additional action is required. */
    }
}

/** @brief Disable motor PWM after an initialization failure. */
static ER motor_init_fail(ER error)
{
    gb_motor_ready = false;
    /* Clear motor PWM compare values. */
    motor_stop();
    /* Disable both motor PWM slices. */
    (void)pwm_set_enabled(MOTOR_L_A_PIN, false);
    (void)pwm_set_enabled(MOTOR_R_A_PIN, false);
    return (error);
}
