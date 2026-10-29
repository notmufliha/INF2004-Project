/*
 *    task_producer.c
 *    Subsystem 2 -- the producer task.
 *
 *    APIs shown: tk_wai_sem, tk_get_mpf, tk_snd_mbx, tk_get_tim,
 *                tk_dly_tsk, tk_get_prc
 *
 *    Replace subsystem2_make_value() with a real sensor read (ADC, I2C, ...) in
 *    your project.  Keep each peripheral owned by ONE task.
 */

#include "task_producer.h"
#include "subsystem2.h"
#include "../subsystem3/subsystem3.h"

/** @brief Generate a triangle-wave sample between 0 and 100. */
EXPORT INT subsystem2_make_value(UW seq)
{
    INT phase = (INT)(seq % 20);
    return ((phase < 10 ? phase : 20 - phase) * 10);
}

EXPORT void subsystem2_producer_task(INT stacd, void *p_exinf)
{
    UW seq = 0;
    subsystem2_sample_msg_t *p_msg = NULL;
    SYSTIM now = {0};
    ER err;

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        /* 1. Take a free slot.  If the consumer is behind and all
         * slots are in use, this task blocks here until one is
         * returned (flow control). */
        err = tk_wai_sem(gh_sem_slots, 1, TMO_FEVR);
        CHECK(err, "tk_wai_sem");
        /* 2. Get a memory block for the message.  The semaphore
         * guarantees one is free, so poll (TMO_POL = don't wait). */
        err = tk_get_mpf(gh_mpf_samples, (void **)&p_msg, TMO_POL);
        if (err < E_OK)
        {
            CHECK(err, "tk_get_mpf");
            tk_sig_sem(gh_sem_slots, 1);    /* give the slot back */
            tk_dly_tsk(S2_PERIOD_MS);
            continue;
        }
        /* 3. Fill it in.  tk_get_tim gives milliseconds since boot
         * as a 64-bit hi:lo pair; lo is enough for ~49 days. */
        tk_get_tim(&now);
        p_msg->seq     = seq++;
        p_msg->time_ms = now.lo;
        p_msg->value   = subsystem2_make_value(p_msg->seq);
#if TK_SUPPORT_SMP
        p_msg->prc     = tk_get_prc();    /* which core am I on? */
#else
        p_msg->prc     = 1;
#endif

        /* 4. Send the POINTER to subsystem 3.  From here on the block
         * belongs to the receiver: do not touch msg again. */
        err = tk_snd_mbx(mbx_samples, (T_MSG *)p_msg);
        CHECK(err, "tk_snd_mbx");
        /* 5. Wait for the next period.  Other tasks run meanwhile. */
        tk_dly_tsk(S2_PERIOD_MS);
    }
}
