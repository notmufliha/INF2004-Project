/*
 *    subsystem2.h
 *    Subsystem 2 -- Producer (sample source).
 *
 *    Public interface of subsystem 2.  Other subsystems include it as
 *    "../subsystem2/subsystem2.h".
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#define S2_PRI_PRODUCER        14    /* priority band 14-15 (TEAM.md) */
/* Core 1 motion loop runs above encoder/telemetry diagnostics. */
#define S2_PRI_CONTROL          5
#define S2_PERIOD_MS        300    /* one sample every 300 ms */
/* ---------------------------------------------------------------------
 *  Sample record retained for the fixed-size pool diagnostics and tests.
 *  The integrated robot does not send this record to Subsystem 3.
 * --------------------------------------------------------------------- */

typedef struct
{
    T_MSG hdr;        /* used by the kernel, do not touch */
    UW seq;        /* sample sequence number */
    UW time_ms;    /* system time when the sample was taken */
    INT value;        /* the "measurement", 0..100 */
    INT prc;        /* processor (core) that produced it: 1 or 2 */
} subsystem2_sample_msg_t;
#define S2_POOL_COUNT    4    /* at most 4 samples in flight */
/* Sample-pool resources inspected by the system monitor and tests. */
IMPORT ID gh_sem_slots;
IMPORT ID gh_mpf_samples;
ER subsystem2_init(void);

ER subsystem2_start(void);
#if APP_TEST

void subsystem2_test(void);
#endif
#endif /* SUBSYSTEM2_H */
