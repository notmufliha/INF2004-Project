/** @file diag_stub.c
 * @brief Deterministic FastLock stand-in; not an SMP scheduling simulation.
 */
#include "motion_stub.h"
#include <assert.h>

ER       g_diag_create_error;
uint32_t g_diag_lock_depth;

/** @brief Supply a lock or an injected creation error. */
ER
CreateLock (FastLock * p_lock, const UB * p_name)
{
    (void)p_name;
    p_lock->id = 9;
    return g_diag_create_error;
}

/** @brief Reject recursive acquisition in native tests. */
void
Lock (FastLock * p_lock)
{
    assert(9 == p_lock->id);
    assert(0U == g_diag_lock_depth);
    ++g_diag_lock_depth;
}

/** @brief Verify every taken lock is released. */
void
Unlock (FastLock * p_lock)
{
    assert(9 == p_lock->id);
    assert(1U == g_diag_lock_depth);
    --g_diag_lock_depth;
}

/*** end of file ***/
