/*
 * Buddy 2 - Buddy 5 Safety Interface
 *
 * Buddy 5 publishes obstacle STOP and clear-path requests.
 * Buddy 2 is the sole owner of motor PWM.
 *
 * Shared state is protected using micro T-Kernel spinlocks.
 *
 * Flow: Buddy 5 requests STOP → Buddy 2 applies zero PWM → Buddy 2
 * acknowledges STOP →
 *       Buddy 5 can report a clear path → Buddy 1 must issue a new DRIVE
 * command.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "b2_motor.h"
#include "motor_control.h"
#include "../robot_config.h"
#include <tk/smp.h>
#define B2_SAFETY_HEARTBEAT_MS ROBOT_MOTOR_WATCHDOG_MS

static uint32_t g_stop_generation = 1u;
static uint32_t g_acknowledged_generation = 0u;
static uint32_t g_clear_heartbeat_ms = 0u;
static bool gb_clear_valid = false;
static bool gb_stopped = true;
/*
 * STOP remains latched until a fresh clear-path report
 * arrives after Buddy 2 acknowledges the STOP.
 */

static bool gb_stop_latched = true;
#if TK_SUPPORT_SMP

static T_SPLOCK g_safety_lock = 0;
#endif

static void safety_lock_enter(UINT *p_interrupt_status);
static void safety_lock_leave(UINT interrupt_status);
static uint32_t b2_now_ms(void);

ER b2_safety_init(void)
{
    g_stop_generation = 1u;
    g_acknowledged_generation = 0u;
    g_clear_heartbeat_ms = 0u;
    gb_clear_valid = false;
    gb_stopped = true;
    gb_stop_latched = true;
#if TK_SUPPORT_SMP
    g_safety_lock = 0;
#endif

    motor_stop();
    return (E_OK);
}

/** @brief Latch STOP and immediately disable PWM under the safety lock. */
void b2_request_stop(void)
{
    UINT interrupt_status;

    safety_lock_enter(&interrupt_status);
    /*
     * A new STOP event invalidates the previous
     * movement authorization.
     *
     * Repeated STOP requests while already latched
     * do not increment the generation.
     */
    if (!gb_stop_latched)
    {
        g_stop_generation++;
    }
    gb_clear_valid = false;
    gb_stop_latched = true;
    /*
     * Disable motor PWM immediately while holding
     * the safety spinlock.
     *
     * The PI controller acknowledges the STOP
     * generation separately.
     */
    motor_stop();
    gb_stopped = true;

    safety_lock_leave(interrupt_status);
}

/** @brief Record clearance only after the current STOP is acknowledged. */
void b2_request_forward(void)
{
    UINT interrupt_status;
    uint32_t now = b2_now_ms();

    safety_lock_enter(&interrupt_status);
    /*
     * A clear-path report cannot cancel an
     * unacknowledged STOP.
     */
    if (g_stop_generation == g_acknowledged_generation)
    {
        g_clear_heartbeat_ms = now;
        gb_clear_valid = true;
        gb_stop_latched = false;
    }
    safety_lock_leave(interrupt_status);
}

bool b2_safety_stop_requested(void)
{
    UINT interrupt_status;
    bool b_pending;

    safety_lock_enter(&interrupt_status);
    b_pending = (g_stop_generation != g_acknowledged_generation);

    safety_lock_leave(interrupt_status);
    return (b_pending);
}

uint32_t b2_safety_stop_generation(void)
{
    UINT interrupt_status;
    uint32_t generation;

    safety_lock_enter(&interrupt_status);
    generation = g_stop_generation;

    safety_lock_leave(interrupt_status);
    return (generation);
}

bool b2_safety_motion_allowed(void)
{
    UINT interrupt_status;
    uint32_t last;
    bool b_allowed;
    uint32_t now = b2_now_ms();
#if !ROBOT_ENABLE_DRIVE
    return (false);
#endif

    safety_lock_enter(&interrupt_status);
    b_allowed =
        (g_stop_generation == g_acknowledged_generation) &&
        (!gb_stop_latched) &&
        (gb_clear_valid);
    last = g_clear_heartbeat_ms;

    safety_lock_leave(interrupt_status);
    if (!b_allowed)
    {
        return (false);
    }
    return (uint32_t)(now - last) <=
        B2_SAFETY_HEARTBEAT_MS;
}

/** @brief Disable PWM without acknowledging a safety STOP generation. */
void b2_safety_force_stop(void)
{
    UINT interrupt_status;

    safety_lock_enter(&interrupt_status);
    motor_stop();
    gb_stopped = true;

    safety_lock_leave(interrupt_status);
}

/** @brief Disable PWM and acknowledge the current STOP generation. */
void b2_safety_acknowledge_stop(void)
{
    UINT interrupt_status;
    /*
     * Only Buddy 2's controller calls this function.
     */
    safety_lock_enter(&interrupt_status);
    motor_stop();
    gb_stopped = true;
    g_acknowledged_generation = g_stop_generation;

    safety_lock_leave(interrupt_status);
}

void b2_safety_apply_pwm(
    int left_percent,
    int right_percent,
    uint32_t command_safety_generation
)
{
    UINT interrupt_status;
    uint32_t now = b2_now_ms();
    bool b_allowed;

    safety_lock_enter(&interrupt_status);
    b_allowed =
        (command_safety_generation == g_stop_generation) &&
        (g_stop_generation == g_acknowledged_generation) &&
        (!gb_stop_latched) &&
        (gb_clear_valid) &&
        ((uint32_t)(now - g_clear_heartbeat_ms) <=
            B2_SAFETY_HEARTBEAT_MS);
#if !ROBOT_ENABLE_DRIVE
    b_allowed = false;
#endif

    if (!b_allowed)
    {
        motor_stop();
        gb_stopped = true;
        g_acknowledged_generation = g_stop_generation;
    }
    else
    {
        gb_stopped = false;
        motor_left_set(left_percent);
        motor_right_set(right_percent);
    }
    safety_lock_leave(interrupt_status);
}

bool b2_motor_is_stopped(void)
{
    UINT interrupt_status;
    bool b_result;

    safety_lock_enter(&interrupt_status);
    b_result =
        (gb_stopped) &&
        (g_stop_generation == g_acknowledged_generation);

    safety_lock_leave(interrupt_status);
    return (b_result);
}

/** @brief Legacy task entry; do not start in the integrated application. */
void b2_motor_task(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;
    tm_printf((UB *)"[B2] Legacy motor task disabled\n");
}

static void safety_lock_enter(UINT *p_interrupt_status)
{
#if TK_SUPPORT_SMP
    (void)ISpinLock(&g_safety_lock, p_interrupt_status);
#else
    DI(*p_interrupt_status);
#endif
}

static void safety_lock_leave(UINT interrupt_status)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&g_safety_lock, interrupt_status);
#else
    EI(interrupt_status);
#endif
}

static uint32_t b2_now_ms(void)
{
    SYSTIM now;

    if (E_OK != tk_get_tim(&now))
    {
        return (0u);
    }
    return ((uint32_t)now.lo);
}
