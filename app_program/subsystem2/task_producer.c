/*
 *    task_producer.c
 *    Subsystem 2 -- deterministic sample-value helper for tests.
 */

#include "task_producer.h"

/** @brief Generate a triangle-wave sample between 0 and 100. */
EXPORT INT subsystem2_make_value(UW seq)
{
    INT phase = (INT)(seq % 20);
    return ((phase < 10 ? phase : 20 - phase) * 10);
}
