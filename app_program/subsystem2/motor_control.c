#include "motor_control.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define MOTOR_L_A_PIN  8U
#define MOTOR_L_B_PIN  9U

#define PWM_WRAP       9999U
#define PWM_CLOCK_DIV  1.25f

static void motor_set_pwm(uint pin, uint16_t level)
{
    pwm_set_gpio_level(pin, level);
}

void motor_init(void)
{
    gpio_set_function(MOTOR_L_A_PIN, GPIO_FUNC_PWM);
/*
 * Buddy 2 - Motion Control
 * Basic left motor test using micro T-Kernel BSP.
 */

#include "../app.h"
#include "motor_control.h"

#define MOTOR_L_A_PIN   8U
#define MOTOR_L_B_PIN   9U

#define MOTOR_PWM_WRAP  999U
#define MOTOR_TEST_DUTY 50

#define RESETS_RESET_REG  (*(volatile UW *)0x4000C000U)
#define RESETS_DONE_REG   (*(volatile UW *)0x4000C008U)

#define PWM_RESET_MASK    (1U << 14)

/* Forward declaration for PWM diagnostics. */
static void motor_pwm_debug(void);

#define RESETS_RESET_DONE (*(volatile UW *)0x4000C008U)

#define PWM_BASE_ADDR 0x40050000U

#define PWM_SLICE4_CSR (*(volatile UW *)(PWM_BASE_ADDR + 0x50U))
#define PWM_SLICE4_DIV (*(volatile UW *)(PWM_BASE_ADDR + 0x54U))
#define PWM_SLICE4_CTR (*(volatile UW *)(PWM_BASE_ADDR + 0x58U))
#define PWM_SLICE4_CC  (*(volatile UW *)(PWM_BASE_ADDR + 0x5CU))
#define PWM_SLICE4_TOP (*(volatile UW *)(PWM_BASE_ADDR + 0x60U))

/*
 * Stop both motor-driver inputs.
 */
void motor_stop(void)
{
    pwm_set_cc(MOTOR_L_A_PIN, 0U);
    pwm_set_cc(MOTOR_L_B_PIN, 0U);
}

/*
 * Initialise left motor PWM.
 */
void motor_init(void)
{
    /* Release PWM peripheral from reset. */
    RESETS_RESET_REG &= ~PWM_RESET_MASK;

    /* Wait until PWM reset completes. */
    while ((RESETS_DONE_REG & PWM_RESET_MASK) == 0U) {
        /* Wait */
    }

    tm_printf((UB*)"[B2] PWM peripheral ready\n");

    /* Configure PWM slice 4. */
    pwm_set_enabled(MOTOR_L_A_PIN, FALSE);

    pwm_set_wrap(MOTOR_L_A_PIN, MOTOR_PWM_WRAP);

    motor_stop();

    pwm_set_pin(MOTOR_L_A_PIN);
    pwm_set_pin(MOTOR_L_B_PIN);

    pwm_set_enabled(MOTOR_L_A_PIN, TRUE);
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

    /* Clear both channels before changing direction. */
    motor_stop();

    if (speed_percent > 0) {
        pwm_set_cc(MOTOR_L_A_PIN, duty);
    } else if (speed_percent < 0) {
        pwm_set_cc(MOTOR_L_B_PIN, duty);
    }
}


/*
 * Temporary motor test task.
 */


void motor_test_task(int stacd, void *exinf)
{
    ER ercd;

    (void)stacd;
    (void)exinf;

    tk_dly_tsk(5000);

    tm_printf((UB*)"[B2] PWM motor test starting\n");

    /* Disable PWM while setting duty cycle. */
    pwm_set_enabled(MOTOR_L_A_PIN, FALSE);

    /* Set 50% duty cycle. */
    motor_left_set(MOTOR_TEST_DUTY);

    /* Enable PWM after duty cycle is configured. */
    pwm_set_enabled(MOTOR_L_A_PIN, TRUE);

    motor_pwm_debug();
    tk_dly_tsk(100);
    motor_pwm_debug();

    /* Run for approximately 2 seconds. */
    ercd = tk_dly_tsk(2000);

    /* Stop PWM output. */
    motor_stop();
    pwm_set_enabled(MOTOR_L_A_PIN, FALSE);

    if (ercd != E_OK) {
        tm_printf((UB*)"[B2] Motor delay error\n");
    }

    tm_printf((UB*)"[B2] PWM motor test stopped\n");

    tk_ext_tsk();
}

static void motor_pwm_debug(void)
{
    tm_printf((UB*)"[B2] PWM CSR=%08x DIV=%08x\n",
              PWM_SLICE4_CSR, PWM_SLICE4_DIV);

    tm_printf((UB*)"[B2] PWM CTR=%08x CC=%08x TOP=%08x\n",
              PWM_SLICE4_CTR, PWM_SLICE4_CC, PWM_SLICE4_TOP);

    tm_printf((UB*)"[B2] RESET_DONE=%08x\n", RESETS_RESET_DONE);
}
