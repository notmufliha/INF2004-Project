/*
 *	subsystem1.c
 *	Subsystem 1 -- WiFi Communication, Command & Telemetry.
 *	Creates the kernel objects (init) and starts the tasks (start).
 *
 *	Objects, and who uses them:
 *	  mbf_events      app tasks  -> comm_task     COMM_EVENT (fixed size)
 *	  mbf_tx          comm_task  -> WiFi task     topic + JSON (variable)
 *	  mbf_rx          WiFi task  -> comm_task     command text (variable)
 *	  mbf_remote_cmd  comm_task  -> Vehicle Ctrl  REMOTE_CMD   (IF-S1-CMD)
 *	  mtx_telem       guards the telemetry snapshot s1_telem
 *	  flg_comm        wakes comm_task as soon as something is queued
 *	  cyc_led         paces the status LED
 *
 *	APIs shown: tk_cre_mbf, tk_cre_mtx (TA_INHERIT), tk_cre_flg,
 *	            tk_cre_cyc, tk_sta_cyc
 */

#include "s1_private.h"

EXPORT ID	tid_led;		/* IF-05 (template example) */
EXPORT ID	tid_comm;
EXPORT ID	cyc_led;
EXPORT ID	mbf_events;
EXPORT ID	mbf_tx;
EXPORT ID	mbf_rx;
EXPORT ID	mbf_remote_cmd;		/* IF-S1-CMD */
EXPORT ID	mtx_telem;
EXPORT ID	flg_comm;

LOCAL ID make_mbf(SZ bufsz, INT maxmsz)
{
	T_CMBF cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = bufsz;
	cmbf.maxmsz = maxmsz;
	return tk_cre_mbf(&cmbf);
}

EXPORT ER subsystem1_init(void)
{
	T_CCYC	ccyc = {0};
	T_CMTX	cmtx = {0};
	T_CFLG	cflg = {0};

	mbf_events = make_mbf(EVENT_QUEUE_LEN *
			      (SZ)(sizeof(COMM_EVENT) + MBF_MSG_OVERHEAD),
			      (INT)sizeof(COMM_EVENT));
	if (mbf_events < E_OK) {
		return mbf_events;
	}

	mbf_tx = make_mbf(TX_BUF_SIZE, TX_RECORD_MAX);
	if (mbf_tx < E_OK) {
		return mbf_tx;
	}

	mbf_rx = make_mbf(RX_BUF_SIZE, COMM_CMD_MAX + 1);
	if (mbf_rx < E_OK) {
		return mbf_rx;
	}

	mbf_remote_cmd = make_mbf(REMOTE_CMD_QUEUE_LEN *
				  (SZ)(sizeof(REMOTE_CMD) + MBF_MSG_OVERHEAD),
				  (INT)sizeof(REMOTE_CMD));
	if (mbf_remote_cmd < E_OK) {
		return mbf_remote_cmd;
	}

	/* TA_INHERIT: if a high-priority sensor task waits for the snapshot
	   while low-priority comm_task holds it, comm_task is raised so the
	   wait is as short as possible (no priority inversion). */
	cmtx.mtxatr = TA_INHERIT;
	mtx_telem = tk_cre_mtx(&cmtx);
	if (mtx_telem < E_OK) {
		return mtx_telem;
	}

	cflg.flgatr  = TA_TFIFO | TA_WSGL;	/* only comm_task waits */
	cflg.iflgptn = 0;
	flg_comm = tk_cre_flg(&cflg);
	if (flg_comm < E_OK) {
		return flg_comm;
	}

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)led_cyclic_handler;
	ccyc.cyctim = S1_TICK_MS;
	ccyc.cycphs = S1_TICK_MS;
	cyc_led = tk_cre_cyc(&ccyc);
	return (cyc_led < E_OK) ? cyc_led : E_OK;
}

EXPORT ER subsystem1_start(void)
{
#if S1_STANDALONE_DEMO
	ID tid_demo;
#endif

	tid_comm = app_start_task(comm_task, S1_PRI_COMM, 0, "comm_task");
	if (tid_comm < E_OK) {
		return tid_comm;
	}

	tid_led = app_start_task(led_task, S1_PRI_LED, 0, "led_task");
	if (tid_led < E_OK) {
		return tid_led;
	}

#if S1_STANDALONE_DEMO
	tid_demo = app_start_task(demo_task, S1_PRI_DEMO, 0, "demo_task");
	if (tid_demo < E_OK) {
		return tid_demo;
	}
#endif

	return tk_sta_cyc(cyc_led);
}
