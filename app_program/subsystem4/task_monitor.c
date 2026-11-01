/* NOTE: Each UB pointer cast below adapts a string/byte buffer to the
 * kernel console API. Character bytes are preserved and formats are not
 * changed.
 */
/** @file task_monitor.c
 * @brief Buddy 4 task monitor module.
 */
#include "task_monitor.h"
#include <stdbool.h>
/*
 *  task_monitor.c
 *  Subsystem 4 -- the monitor task and its alarm handler.
 *
 *  APIs shown: tk_sta_alm, tk_stp_alm, alarm handler, tk_wai_flg,
 *              tk_rcv_mbf, tk_loc_mtx/tk_unl_mtx, tk_ref_sem, tk_ref_mpf,
 *              tk_ref_tsk, tk_sus_tsk, tk_rsm_tsk, tk_chg_pri,
 *              tk_get_tid, tk_get_tim. */

#include "subsystem4.h"
#include "s4_runtime.h"
#include "../subsystem1/subsystem1.h" /* IF-05: tid_led. */
#include "../subsystem2/subsystem2.h" /* Shared sample semaphore and pool. */
#include "../subsystem3/subsystem3.h" /* IF-04: app_stats, mtx_stats. */

/* One task writes this aligned word; tests only take snapshots.
 * Volatile forces fresh loads on the cacheless RP2040.
 */
static volatile uint32_t g_lines_received;

/*
 * Alarm handler: one-shot, interrupt context (same rules as a cyclic
 * handler: keep it short, never wait).
 */

static void task_monitor_alarm_isr (void * p_exinf);

static const char * state_name (UINT tskstat);

static void report_monitor_stats (void);

static void update_monitor_led (UW * p_reports, bool * pb_led_paused);

/** @brief Read the single-writer diagnostic line count. */
uint32_t
task_monitor_line_count (void)
{
    return g_lines_received;
}

/** @brief Register the monitor's private, high-level interrupt callback. */
ID
task_monitor_create_alarm (void)
{
    T_CALM alarm = {0};

    alarm.almatr = TA_HLNG;
    /* The kernel requires FP for a high-level void(void *) callback. Its
     * assembly ISR wrapper preserves the interrupt context; this callback
     * must use the ordinary C ABI, not a raw compiler ISR attribute. */
    alarm.almhdr = (FP)task_monitor_alarm_isr;
    return tk_cre_alm(&alarm);
}

/** @brief Drain summary messages and report shared task statistics. */
void
task_monitor_task (INT stacd, void * p_exinf)
{
    UINT flgptn;
    UB   text[S4_SUMMARY_MAX + 1];
    INT  len;
    UW   reports      = 0;
    bool b_led_paused = false;
    ER   error_code;

    /* The kernel task arguments are unused; these casts discard no data. */
    (void)stacd;
    (void)p_exinf;

    tm_printf((UB *)"monitor: I am task %d\n", tk_get_tid());

    for (;;)
    {
        /* 1. Arm the one-shot alarm, then wait for EITHER bit.
              TWF_ORW = wake on any of the bits, TWF_CLR = clear them
              on wake-up.  The extra timeout is just a safety net. */
        tk_sta_alm(gh_monitor_alarm, S4_ALARM_MS);
        error_code = tk_wai_flg(gh_events, FLG_ALARM | FLG_BATCH,
                                TWF_ORW | TWF_CLR, &flgptn, S4_ALARM_MS * 2);
        tk_stp_alm(gh_monitor_alarm); /* cancel it if a batch came first. */

        if (E_TMOUT == error_code)
        {
            tm_printf((UB *)"monitor: nothing at all happened?\n");
        }
        else if (error_code < E_OK)
        {
            CHECK(error_code, "monitor events");
        }
        else
        {
            if ((flgptn & FLG_ALARM) != 0U)
            {
                tm_printf((UB *)"monitor: alarm -- no batch for %d ms\n",
                          S4_ALARM_MS);
            }

            /* 2. Drain the message buffer without waiting (TMO_POL).
                  tk_rcv_mbf returns the message length, or an error. */
            len = tk_rcv_mbf(gh_summary, text, TMO_POL);

            while (len > 0)
            {
                text[len] = '\0';
                g_lines_received++;
                tm_printf((UB *)"monitor: batch  %s\n", text);
                len = tk_rcv_mbf(gh_summary, text, TMO_POL);
            }

            report_monitor_stats();
            update_monitor_led(&reports, &b_led_paused);
        }
    }
}

/** @brief Return the console label for a kernel task state. */
static const char *
state_name (UINT tskstat)
{
    switch (tskstat)
    {
        case TTS_RUN:
            return "RUN";
        case TTS_RDY:
            return "READY";
        case TTS_WAI:
            return "WAIT";
        case TTS_SUS:
            return "SUSPEND";
        case TTS_WAS:
            return "WAIT+SUSPEND";
        case TTS_DMT:
            return "DORMANT";
        default:
            return "?";
    }
}

/** @brief Snapshot protected statistics and print available resources. */
static void
report_monitor_stats (void)
{
    APP_STATS snap;

    T_RSEM rsem;
    T_RMPF rmpf;
    SYSTIM now;

    /* 3. Copy the stats under subsystem 3's mutex, print outside
          it: hold a mutex for as short a time as possible. */

    ER error_code = tk_loc_mtx(mtx_stats, TMO_FEVR);

    if (error_code < E_OK)
    {
        CHECK(error_code, "monitor stats lock");
        return;
    }

    snap = app_stats;
    CHECK(tk_unl_mtx(mtx_stats), "monitor stats unlock");

    if ((tk_get_tim(&now) < E_OK) || (tk_ref_sem(gh_sem_slots, &rsem) < E_OK) ||
        (tk_ref_mpf(gh_mpf_samples, &rmpf) < E_OK))
    {
        tm_printf((UB *)"monitor: statistics unavailable\n");
        return;
    }

    INT average = 0;

    if (0U != snap.received)
    {
        /* Both kernel 32-bit values widen exactly to int64_t. The
         * nonzero denominator makes the quotient fit the signed sum. */
        average = (INT)((int64_t)snap.sum / (int64_t)snap.received);
    }

    tm_printf((UB *)"monitor: t=%u ms  samples=%u  min=%d max=%d avg=%d"
                    "  free slots=%d  free blocks=%d\n",
              now.lo, snap.received, snap.min, snap.max, average, rsem.semcnt,
              rmpf.frbcnt);
}

/** @brief Periodically pause or resume the demonstration LED task. */
static void
update_monitor_led (UW * p_reports, bool * pb_led_paused)
{
    T_RTSK rtsk;

    /* Every few reports, pause or resume subsystem 1's LED task
          (IF-05) and look at it with tk_ref_tsk. */

    ++(*p_reports);

    if (0U == (*p_reports % S4_LED_TOGGLE_REPORTS))
    {
        if ((*pb_led_paused))
        {
            tk_rsm_tsk(tid_led);
            tk_chg_pri(tid_led, S1_PRI_LED); /* back to normal. */
        }
        else
        {
            tk_sus_tsk(tid_led);
            /* Changing the priority of a task: */
            tk_chg_pri(tid_led, S1_PRI_LED + 1);
        }

        (*pb_led_paused) = !(*pb_led_paused);

        if (tk_ref_tsk(tid_led, &rtsk) < E_OK)
        {
            tm_printf((UB *)"monitor: LED task status unavailable\n");
            return;
        }

        tm_printf((UB *)"monitor: LED task %s, state=%s, pri=%d\n",
                  (*pb_led_paused) ? "paused" : "resumed",
                  state_name(rtsk.tskstat), rtsk.tskpri);
    }
}

/** @brief Set the alarm bit without waiting in interrupt context. */
static void
task_monitor_alarm_isr (void * p_exinf)
{
    /* The kernel callback argument is unused; no value conversion occurs. */
    (void)p_exinf;
    tk_set_flg(gh_events, FLG_ALARM);
}

/*** end of file ***/

