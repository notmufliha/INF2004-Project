#include "subsystem5.h"
#include "s5_mission.h"
#include "../subsystem2/b2_motor.h"
#if TK_SUPPORT_SMP
#include <tk/smp.h>
static T_SPLOCK mission_lock;
#endif
static bool enabled;
static S5MissionState mission_state;
static void lock(UINT *irq)
{
#if TK_SUPPORT_SMP
    (void)ISpinLock(&mission_lock, irq);
#else
    DI(*irq);
#endif
}
static void unlock(UINT irq)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&mission_lock, irq);
#else
    EI(irq);
#endif
}
void s5_mission_init(void)
{
#if TK_SUPPORT_SMP
    (void)InitSpinLock(&mission_lock);
#endif
    enabled = false;
    mission_state = S5_MISSION_FOLLOW;
}
void s5_mission_enable(bool allow)
{
    UINT irq;
    lock(&irq); enabled = allow; unlock(irq);
    if (!allow) b2_request_stop();
}
bool s5_mission_enabled(void)
{
    UINT irq;
    bool allow;
    lock(&irq); allow = enabled; unlock(irq);
    return allow;
}
void s5_mission_publish(S5MissionState state)
{
    UINT irq;
    lock(&irq); mission_state = state; unlock(irq);
}
S5MissionState s5_mission_state(void)
{
    UINT irq;
    S5MissionState state;
    lock(&irq); state = mission_state; unlock(irq);
    return state;
}
