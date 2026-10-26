/*
 * Buddy 2 - Buddy 5 Safety Interface
 *
 * Buddy 5 publishes obstacle STOP and clear-path requests.
 * Buddy 2 is the sole owner of motor PWM.
 *
 * Shared state is protected using micro T-Kernel spinlocks.
 * 
 * Flow: Buddy 5 requests STOP → Buddy 2 applies zero PWM → Buddy 2 acknowledges STOP → 
 *       Buddy 5 can report a clear path → Buddy 1 must issue a new DRIVE command.
 * 
 */

#include "b2_motor.h"
#include "motor_control.h"
#include "../robot_config.h"
#include "../app.h"
#include <tk/smp.h>

#define B2_SAFETY_HEARTBEAT_MS ROBOT_MOTOR_WATCHDOG_MS

static uint32_t stop_generation = 1u;
static uint32_t acknowledged_generation = 0u;
static uint32_t clear_heartbeat_ms = 0u;
static uint32_t clear_valid = 0u;
static uint32_t stopped = 1u;

/*
 * STOP remains latched until a fresh clear-path report
 * arrives after Buddy 2 acknowledges the STOP.
 */
static uint32_t stop_latched = 1u;

#if TK_SUPPORT_SMP
static T_SPLOCK safety_lock = 0;
#endif

static void safety_lock_enter(UINT *intsts)
{
#if TK_SUPPORT_SMP
    (void)ISpinLock(&safety_lock, intsts);
#else
    DI(*intsts);
#endif
}

static void safety_lock_leave(UINT intsts)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&safety_lock, intsts);
#else
    EI(intsts);
#endif
}

static uint32_t b2_now_ms(void)
{
    SYSTIM now;

    if (tk_get_tim(&now) != E_OK) {
        return 0u;
    }

    return (uint32_t)now.lo;
}

ER b2_safety_init(void)
{
    stop_generation = 1u;
    acknowledged_generation = 0u;
    clear_heartbeat_ms = 0u;
    clear_valid = 0u;
    stopped = 1u;
    stop_latched = 1u;

#if TK_SUPPORT_SMP
    safety_lock = 0;
#endif

    motor_stop();

    return E_OK;
}

void b2_request_stop(void)
{
    UINT intsts;

    safety_lock_enter(&intsts);

    /*
     * A new STOP event invalidates the previous
     * movement authorization.
     *
     * Repeated STOP requests while already latched
     * do not increment the generation.
     */
    if (stop_latched == 0u) {
        stop_generation++;
    }

    clear_valid = 0u;
    stop_latched = 1u;

    /*
     * Disable motor PWM immediately while holding
     * the safety spinlock.
     *
     * The PI controller acknowledges the STOP
     * generation separately.
     */
    motor_stop();
    stopped = 1u;

    safety_lock_leave(intsts);
}

void b2_request_forward(void)
{
    UINT intsts;
    uint32_t now = b2_now_ms();

    safety_lock_enter(&intsts);

    /*
     * A clear-path report cannot cancel an
     * unacknowledged STOP.
     */
    if (stop_generation == acknowledged_generation) {
        clear_heartbeat_ms = now;
        clear_valid = 1u;
        stop_latched = 0u;
    }

    safety_lock_leave(intsts);
}

bool b2_safety_stop_requested(void)
{
    UINT intsts;
    bool pending;

    safety_lock_enter(&intsts);

    pending = (stop_generation != acknowledged_generation);

    safety_lock_leave(intsts);

    return pending;
}

uint32_t b2_safety_stop_generation(void)
{
    UINT intsts;
    uint32_t generation;

    safety_lock_enter(&intsts);

    generation = stop_generation;

    safety_lock_leave(intsts);

    return generation;
}

bool b2_safety_motion_allowed(void)
{
    UINT intsts;
    uint32_t last;
    bool allowed;
    uint32_t now = b2_now_ms();

#if !ROBOT_ENABLE_DRIVE
    return false;
#endif

    safety_lock_enter(&intsts);

    allowed =
        (stop_generation == acknowledged_generation) &&
        (stop_latched == 0u) &&
        (clear_valid != 0u);

    last = clear_heartbeat_ms;

    safety_lock_leave(intsts);

    if (!allowed) {
        return false;
    }

    return (uint32_t)(now - last) <=
           B2_SAFETY_HEARTBEAT_MS;
}

// set motor PWM to zero without changing the STOP acknowledgement generation.
void b2_safety_force_stop(void)
{
    UINT intsts;

    safety_lock_enter(&intsts);

    motor_stop();
    stopped = 1u;

    safety_lock_leave(intsts);
}

void b2_safety_acknowledge_stop(void)
{
    UINT intsts;

    /*
     * Only Buddy 2's controller calls this function.
     */
    safety_lock_enter(&intsts);

    motor_stop();

    stopped = 1u;
    acknowledged_generation = stop_generation;

    safety_lock_leave(intsts);
}

void b2_safety_apply_pwm(
    int left_percent,
    int right_percent,
    uint32_t command_safety_generation
)
{
    UINT intsts;
    uint32_t now = b2_now_ms();
    bool allowed;

    safety_lock_enter(&intsts);

    allowed =
        (command_safety_generation == stop_generation) &&
        (stop_generation == acknowledged_generation) &&
        (stop_latched == 0u) &&
        (clear_valid != 0u) &&
        ((uint32_t)(now - clear_heartbeat_ms) <=
        B2_SAFETY_HEARTBEAT_MS);

#if !ROBOT_ENABLE_DRIVE
    allowed = false;
#endif

    if (!allowed) {
        motor_stop();
        stopped = 1u;
        acknowledged_generation = stop_generation;
    } else {
        stopped = 0u;
        motor_left_set(left_percent);
        motor_right_set(right_percent);
    }

    safety_lock_leave(intsts);
}

bool b2_motor_is_stopped(void)
{
    UINT intsts;
    bool result;

    safety_lock_enter(&intsts);

    result =
        (stopped != 0u) &&
        (stop_generation == acknowledged_generation);

    safety_lock_leave(intsts);

    return result;
}

/*
 * Legacy motor task is disabled.
 */
void b2_motor_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    tm_printf((UB*)"[B2] Legacy motor task disabled\n");
}
