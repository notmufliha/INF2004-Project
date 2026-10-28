/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *	(add NET=lwip MQTT=1 to include the network tests S1-U09/U10).
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	output to testing/subsystem1/logs/ and record the result in
 *	testing/subsystem1/TEST_LOG.md.
 *
 *	The bit-exact packing of every frame is also checked on the laptop,
 *	against the Python decoder: testing/subsystem1/host/.
 */

#include "s1_private.h"
#include "../test_report.h"

#if APP_TEST

#define CONNECT_WAIT_MS		30000	/* S1-U09: time allowed to connect */
#define BURST_EVENTS		40	/* S1-U06: events posted in a burst */
#define BURST_LIMIT_MS		20	/* S1-U06: must finish within this  */
#define FAULT_ROUNDS		1000	/* S1-U20: set/clear repetitions    */

/* Build a 3-byte command frame: ver:3 id:4 arg:7 seq:8 reserved:2 */
LOCAL void make_cmd(UB out[WIRE_CMD_LEN], UW ver, UW id, UW arg, UW seq,
		    UW reserved)
{
	BITBUF b;

	bits_begin(&b, out, WIRE_CMD_LEN);
	bits_put(&b, ver, 3);
	bits_put(&b, id, 4);
	bits_put(&b, arg, 7);
	bits_put(&b, seq, 8);
	bits_put(&b, reserved, 2);
}

LOCAL CMD_RESULT decode(UW ver, UW id, UW arg, UW seq, UW reserved,
			PARSED_CMD *pc)
{
	UB f[WIRE_CMD_LEN];

	make_cmd(f, ver, id, arg, seq, reserved);
	return comm_cmd_decode(f, WIRE_CMD_LEN, pc);
}

LOCAL BOOL decodes_to(UW id, UW arg, UW seq)
{
	PARSED_CMD pc;

	if ((decode(WIRE_VERSION, id, arg, seq, 0U, &pc) != CMD_OK) ||
	    (pc.id != id) || (pc.arg != arg) || (pc.seq != seq)) {
		return FALSE;
	}
	if (id >= RCMD_COUNT) {
		return TRUE;		/* handled by S1, not forwarded */
	}
	return ((pc.cmd.type == id) && (pc.cmd.arg == arg) &&
		(pc.cmd.seq == seq)) ? TRUE : FALSE;
}

LOCAL CMD_RESULT result_of(UW ver, UW id, UW arg, UW reserved)
{
	PARSED_CMD pc;

	return decode(ver, id, arg, 7U, reserved, &pc);
}

EXPORT void subsystem1_test(void)
{
	T_RCYC	rcyc;
	T_RTSK	rtsk;
	ER	er;

	TEST_INIT();

	/* S1-U01: LED cyclic handler is running */
	er = tk_ref_cyc(cyc_led, &rcyc);
	TEST_CHECK("S1-U01", er == E_OK && (rcyc.cycstat & TCYC_STA),
		   "LED cyclic handler is running");

	/* S1-U02: comm_task sleeps between loops (not busy-waiting), so it
	   cannot starve the control tasks (NFR-B1-3) */
	{
		INT i, waiting = 0;
		for (i = 0; i < 10; i++) {
			tk_dly_tsk(37);
			if (tk_ref_tsk(tid_comm, &rtsk) == E_OK &&
			    (rtsk.tskstat & TTS_WAI)) {
				waiting++;
			}
		}
		TEST_CHECK("S1-U02", waiting >= 9, "comm_task sleeps between loops");
	}

	/* S1-U03: valid binary commands are accepted and decoded (FR-B1-5) */
	TEST_CHECK("S1-U03",
		   decodes_to(RCMD_START, 0, 0) &&
		   decodes_to(RCMD_STOP, 0, 17) &&
		   decodes_to(RCMD_SPEED, 40, 3) &&
		   decodes_to(RCMD_SPEED, 100, 255) &&
		   decodes_to(RCMD_MODE, MODE_LINE_ONLY, 1) &&
		   decodes_to(RCMD_ESTOP, 0, 9) &&
		   decodes_to(WCMD_PING, 0, 2) &&
		   decodes_to(WCMD_TELEM, COMM_TELEM_MIN_DS, 4) &&
		   decodes_to(WCMD_TELEM, COMM_TELEM_MAX_DS, 5),
		   "valid 3-byte commands accepted and decoded");

	/* S1-U04: invalid frames are rejected (NFR-B1-5) */
	{
		PARSED_CMD	pc;
		UB		f[WIRE_CMD_LEN + 1] = {0};

		make_cmd(f, WIRE_VERSION, RCMD_STOP, 0U, 0U, 0U);
		TEST_CHECK("S1-U04",
			   (comm_cmd_decode(f, 2, &pc) == CMD_ERR_LENGTH) &&
			   (comm_cmd_decode(f, 4, &pc) == CMD_ERR_LENGTH) &&
			   (result_of(2U, RCMD_STOP, 0U, 0U) == CMD_ERR_VERSION) &&
			   (result_of(WIRE_VERSION, RCMD_STOP, 0U, 1U) == CMD_ERR_RESERVED) &&
			   (result_of(WIRE_VERSION, 0U, 0U, 0U) == CMD_ERR_UNKNOWN) &&
			   (result_of(WIRE_VERSION, 15U, 0U, 0U) == CMD_ERR_UNKNOWN) &&
			   (result_of(WIRE_VERSION, RCMD_SPEED, 101U, 0U) == CMD_ERR_RANGE) &&
			   (result_of(WIRE_VERSION, RCMD_MODE, MODE_COUNT, 0U) == CMD_ERR_RANGE) &&
			   (result_of(WIRE_VERSION, WCMD_TELEM, 0U, 0U) == CMD_ERR_RANGE) &&
			   (result_of(WIRE_VERSION, RCMD_STOP, 5U, 0U) == CMD_ERR_EXTRA_ARG),
			   "invalid command frames rejected");
	}

	/* S1-U05: frames have the documented sizes, and packing never
	   writes past the buffer (guard byte) */
	{
		UB		buf[COMM_PAYLOAD_MAX + 1];
		COMM_TELEMETRY	t = {0};
		COMM_EVENT	ev = {0};
		STATUS_INFO	st = {0};
		INT		n_tel, n_bc, n_obs, n_err, n_st, n_ack, n_small;

		buf[COMM_PAYLOAD_MAX] = 0xA5U;
		n_tel = pack_telemetry(buf, COMM_PAYLOAD_MAX, 1, 2, 0, &t);
		ev.type = EVT_BARCODE;
		n_bc  = pack_event(buf, COMM_PAYLOAD_MAX, 1, &ev);
		ev.type = EVT_OBSTACLE;
		n_obs = pack_event(buf, COMM_PAYLOAD_MAX, 1, &ev);
		ev.type = EVT_ERROR;
		n_err = pack_event(buf, COMM_PAYLOAD_MAX, 1, &ev);
		n_st  = pack_status(buf, COMM_PAYLOAD_MAX, &st);
		n_ack = pack_ack(buf, COMM_PAYLOAD_MAX, 1, 1, CMD_OK);
		n_small = pack_telemetry(buf, 10, 1, 2, 0, &t);	/* too small */

		tm_printf((UB*)"[TEST] S1-U05 info: telemetry %d B, event %d-%d B, "
			  "status %d B, ack %d B; sizeof telem %d, event %d, "
			  "cmd %d\n", n_tel, n_bc, n_obs, n_st, n_ack,
			  (INT)sizeof(COMM_TELEMETRY), (INT)sizeof(COMM_EVENT),
			  (INT)sizeof(REMOTE_CMD));
		TEST_CHECK("S1-U05",
			   (n_tel == WIRE_TELEM_LEN) && (n_bc == 5) &&
			   (n_err == 7) && (n_obs == WIRE_EVT_MAX_LEN) &&
			   (n_st == WIRE_STATUS_LEN) && (n_ack == WIRE_ACK_LEN) &&
			   (n_small == -1) && (buf[COMM_PAYLOAD_MAX] == 0xA5U) &&
			   (sizeof(COMM_TELEMETRY) == 24U) &&
			   (sizeof(COMM_EVENT) == 8U) && (sizeof(REMOTE_CMD) == 4U),
			   "binary frame sizes and bounded packing");
	}

	/* S1-U06: reporting events never blocks, even when the queue is
	   full (NFR-B1-2): a burst of 40 must finish in under 20 ms */
	{
		UW	t0 = s1_now_ms();
		INT	i, ok = 0, full = 0;

		for (i = 0; i < BURST_EVENTS; i++) {
			er = comm_report_error(ERR_S1_TEST, (UB)i);
			if (er == E_OK) {
				ok++;
			} else if (er == E_TMOUT) {
				full++;
			}
		}
		TEST_CHECK("S1-U06",
			   ((s1_now_ms() - t0) < BURST_LIMIT_MS) &&
			   ((ok + full) == BURST_EVENTS),
			   "event reporting is non-blocking");
		tk_dly_tsk(500);		/* let comm_task drain them */
	}

	/* S1-U07: telemetry setters update the packed snapshot, saturate
	   out-of-range values, and keep the low 24 bits of encoder totals */
	{
		COMM_TELEMETRY t;

		comm_set_motion(123, -9000, 1000, -1);
		comm_set_line(11, 5000, 1, LINE_F_DETECTED | LINE_F_JUNCTION);
		comm_set_imu(3U, -700);
		comm_set_obstacle_dist(COMM_CM_INVALID);
		comm_get_telemetry(&t);
		TEST_CHECK("S1-U07",
			   (t.speed_l == 123) && (t.speed_r == -COMM_SPEED_MAX) &&
			   (t.enc_l == 1000U) && (t.enc_r == 0xFFFFFFU) &&
			   (t.ir_left == 11U) && (t.ir_centre == COMM_IR_MAX) &&
			   (t.ir_right == 1U) &&
			   (t.line_flags == (LINE_F_DETECTED | LINE_F_JUNCTION)) &&
			   (t.motion == 3U) && (t.pitch_ddeg == -COMM_PITCH_MAX) &&
			   (t.obs_dist_cm == 0x1FFU),
			   "packed telemetry setters/getter, saturation");
	}

	/* S1-U08: a received frame goes through validation:
	   one valid (PING), one rejected (unknown id) */
	{
		COMM_STATS	before, after;
		UB		ping[WIRE_CMD_LEN], bad[WIRE_CMD_LEN];

		make_cmd(ping, WIRE_VERSION, WCMD_PING, 0U, 1U, 0U);
		make_cmd(bad, WIRE_VERSION, 14U, 0U, 2U, 0U);
		comm_get_stats(&before);
		comm_bridge_rx_command(ping, WIRE_CMD_LEN);
		comm_bridge_rx_command(bad, WIRE_CMD_LEN);
		tk_dly_tsk(300);
		comm_get_stats(&after);
		TEST_CHECK("S1-U08",
			   (after.rx_ok == (UH)(before.rx_ok + 1U)) &&
			   (after.rx_rejected == (UH)(before.rx_rejected + 1U)),
			   "received commands validated (1 ok, 1 rejected)");
	}

	/* S1-U20: the shared fault mask -- setting and clearing one bit
	   never disturbs another subsystem's bit */
	{
		UB	saved = fault_get();
		BOOL	ok = TRUE;
		INT	i;

		fault_clear(FAULT_ALL);
		for (i = 0; i < FAULT_ROUNDS; i++) {
			fault_raise(FAULT_IMU);
			fault_raise(FAULT_NO_ECHO);
			fault_clear(FAULT_IMU);
			if (fault_get() != FAULT_NO_ECHO) {
				ok = FALSE;
			}
			fault_clear(FAULT_NO_ECHO);
		}
		fault_raise(saved);
		TEST_CHECK("S1-U20", ok && (flg_faults > 0),
			   "fault bitmask set/clear keeps other bits");
	}

#if TM_MQTT
	/* S1-U09: reaches CONNECTED (needs the AP and broker running) */
	{
		UW t0 = s1_now_ms();

		while (!comm_is_connected() &&
		       ((s1_now_ms() - t0) < CONNECT_WAIT_MS)) {
			tk_dly_tsk(250);
		}
		tm_printf((UB*)"[TEST] S1-U09 info: connected after %u ms\n",
			  s1_now_ms() - t0);
		TEST_CHECK("S1-U09", comm_is_connected(),
			   "WiFi + MQTT connected within 30 s");
	}

	/* S1-U10: telemetry is actually leaving the queue while connected */
	{
		COMM_STATS before, after;

		comm_get_stats(&before);
		tk_dly_tsk(3000);
		comm_get_stats(&after);
		TEST_CHECK("S1-U10",
			   ((UH)(after.tx_queued - before.tx_queued) >= 4U) &&
			   (after.tx_dropped == before.tx_dropped),
			   "telemetry published for 3 s with no drops");
	}
#endif

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
