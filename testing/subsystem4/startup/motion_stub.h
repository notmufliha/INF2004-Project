/** @file motion_stub.h
 * @brief Native ABI additions for the B4 physical motion-test harness.
 */
#ifndef MOTION_STUB_H
#define MOTION_STUB_H

#include "boot_stub.h"

typedef uint8_t UB;

typedef struct
{
    INT        cnt;
    ID         id;
    const UB * p_name;
} FastLock;

ER   CreateLock (FastLock * p_lock, const UB * p_name);
void Lock (FastLock * p_lock);
void Unlock (FastLock * p_lock);

typedef struct
{
    void * p_next;
} T_MSG;

#define IMPORT extern
ER  tk_dly_tsk (TMO delay);
int tm_printf (const UB * p_format, ...);

#endif

/*** end of file ***/
