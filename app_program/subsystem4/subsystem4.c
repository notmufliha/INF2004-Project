/*
 *	subsystem4.c
 *	Subsystem 4 -- creates IMU demo objects/tasks and legacy monitor.
 *
 *	APIs shown: tk_cre_flg, tk_cre_mbf, tk_cre_alm
 */

#include "subsystem4.h"

EXPORT ID	mbf_summary;	/* IF-03 */
EXPORT ID	flg_events;	/* IF-03 */
EXPORT ID	alm_monitor;
EXPORT ID mbf_imu_data;
#if APP_TEST
EXPORT ID mbf_imu_test_events;
#endif

EXPORT ER subsystem4_init(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};
	T_CALM	calm = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = S4_IMU_QUEUE_DEPTH * S4_MESSAGE_BYTES(imu_data_t);
    cmbf.maxmsz = sizeof(imu_data_t);
    mbf_imu_data = tk_cre_mbf(&cmbf);
    if (mbf_imu_data < E_OK) {
        return mbf_imu_data;
    }
#if APP_TEST
    cmbf.bufsz = 32U * S4_MESSAGE_BYTES(imu_event_t);
    cmbf.maxmsz = sizeof(imu_event_t);
    mbf_imu_test_events = tk_cre_mbf(&cmbf);
    if (mbf_imu_test_events < E_OK) {
        return mbf_imu_test_events;
    }
#endif

	/* Event flag: a 32-bit pattern of bits.  Tasks wait for a bit (or a
	   combination of bits) to be set. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;	/* allow several waiters */
	cflg.iflgptn = 0;			/* all bits clear to begin with */
	flg_events = tk_cre_flg(&cflg);
	if (flg_events < E_OK) {
		return flg_events;
	}

	/* Message buffer: COPIES variable-length data into a kernel ring
	   buffer.  Here it carries short text lines from subsystem 3. */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 4 * S4_SUMMARY_MAX;	/* total buffer size, bytes */
	cmbf.maxmsz = S4_SUMMARY_MAX;		/* largest single message */
	mbf_summary = tk_cre_mbf(&cmbf);
	if (mbf_summary < E_OK) {
		return mbf_summary;
	}

	/* Alarm handler: called ONCE, a given time after tk_sta_alm().
	   The monitor task re-arms it each time round its loop. */
	calm.almatr = TA_HLNG;
	calm.almhdr = (FP)monitor_alarm_handler;
	alm_monitor = tk_cre_alm(&calm);
	return (alm_monitor < E_OK) ? alm_monitor : E_OK;
}

EXPORT ER subsystem4_start(void)
{
    /* Start receiver first; all objects were created during init. */
    ID imu_tid = app_start_task(imu_analysis_task, S4_PRI_IMU_ANALYSIS,
                               0, "imu_analysis_task");
    if (imu_tid < E_OK) {
        return imu_tid;
    }
    imu_tid = app_start_task(imu_sampling_task, S4_PRI_IMU_SAMPLE,
                            0, "imu_sampling_task");
    if (imu_tid < E_OK) {
        return imu_tid;
    }
	/* No affinity: the scheduler may run it on either core */
	ID tid = app_start_task(monitor_task, S4_PRI_MONITOR, 0, "monitor_task");
	return (tid < E_OK) ? tid : E_OK;
}
