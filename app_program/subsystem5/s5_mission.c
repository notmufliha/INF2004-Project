#include "subsystem5.h"
#include "s5_mission.h"
#include "../robot_config.h"
#include "../subsystem2/b2_motor.h"
#if TK_SUPPORT_SMP
#include <tk/smp.h>
static T_SPLOCK mission_lock;
#endif
static bool           enabled;
static bool           approach_slow;
static bool           on_ramp;
static S5MissionState mission_state;

static void
lock (UINT * irq)
{
#if TK_SUPPORT_SMP
    (void)ISpinLock(&mission_lock, irq);
#else
    DI(*irq);
#endif
}

static void
unlock (UINT irq)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&mission_lock, irq);
#else
    EI(irq);
#endif
}

void
s5_mission_init (void)
{
#if TK_SUPPORT_SMP
    (void)InitSpinLock(&mission_lock);
#endif
    enabled       = false;
    approach_slow = false;
    mission_state = S5_MISSION_FOLLOW;
}

void
s5_mission_enable (bool allow)
{
    UINT irq;
    lock(&irq);
    enabled = allow;
    unlock(irq);
    if (!allow)
    {
        b2_request_stop();
    }
}

bool
s5_mission_enabled (void)
{
    UINT irq;
    bool allow;
    lock(&irq);
    allow = enabled;
    unlock(irq);
    return allow;
}

void
s5_mission_publish (S5MissionState state)
{
    UINT irq;
    lock(&irq);
    mission_state = state;
    unlock(irq);
}

S5MissionState
s5_mission_state (void)
{
    UINT           irq;
    S5MissionState state;
    lock(&irq);
    state = mission_state;
    unlock(irq);
    return state;
}

void
s5_mission_front (bool valid, float cm)
{
    UINT irq;
    lock(&irq);
    /* Box mode: slow from 60 cm (then stop at 40, scan at 25). */
    if (valid)
    {
        approach_slow = cm > 0.0f &&
            cm <= (ROBOT_BOX_BYPASS ? 60.0f : ROBOT_OBSTACLE_APPROACH_CM);
    }
    unlock(irq);
}

bool
s5_mission_approach_slow (void)
{
    UINT irq;
    bool slow;
    lock(&irq);
    slow = approach_slow;
    unlock(irq);
    return slow;
}

void
s5_mission_set_ramp (bool ramp)
{
    UINT irq;
    lock(&irq);
    on_ramp = ramp;
    unlock(irq);
}

bool
s5_mission_ramp (void)
{
    UINT irq;
    bool ramp;
    lock(&irq);
    ramp = on_ramp;
    unlock(irq);
    return ramp;
}

/*** end of file ***/

