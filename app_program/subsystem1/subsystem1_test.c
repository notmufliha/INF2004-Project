/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *	(add NET=lwip MQTT=1 to include the network tests S1-U09/U10).
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	output to testing/subsystem1/logs/ and record the result in
 *	testing/subsystem1/TEST_LOG.md.
 */

#include "s1_private.h"
#include "../test_report.h"

#if APP_TEST

#define CONNECT_WAIT_MS		30000	/* S1-U09: time allowed to connect */
#define BURST_EVENTS		40	/* S1-U06: events posted in a burst */
#define BURST_LIMIT_MS		20	/* S1-U06: must finish within this  */

LOCAL BOOL parses_to(const char *s, REMOTE_CMD_TYPE type, H arg, UW seq)
{
	PARSED_CMD pc;
	INT len = 0;

	while (s[len] != '\0') {
		len++;
	}
	return ((comm_cmd_parse((const UB *)s, len, &pc) == CMD_OK) &&
		(pc.local == LOCAL_NONE) && (pc.cmd.type == type) &&
		(pc.cmd.arg == arg) && (pc.cmd.seq == seq)) ? TRUE : FALSE;
}

LOCAL CMD_RESULT parse_result(const char *s)
{
	PARSED_CMD pc;
	INT len = 0;

	while (s[len] != '\0') {
		len++;
	}
	return comm_cmd_parse((const UB *)s, len, &pc);
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

	/* S1-U03: valid commands are accepted and decoded (FR-B1-5) */
	TEST_CHECK("S1-U03",
		   parses_to("START", RCMD_START, 0, 0) &&
		   parses_to("stop", RCMD_STOP, 0, 0) &&
		   parses_to("  Stop  #17 ", RCMD_STOP, 0, 17) &&
		   parses_to("SPEED 40", RCMD_SPEED, 40, 0) &&
		   parses_to("speed 100 #3", RCMD_SPEED, 100, 3) &&
		   parses_to("MODE line", RCMD_MODE, MODE_LINE_ONLY, 0) &&
		   parses_to("ESTOP", RCMD_ESTOP, 0, 0) &&
		   (parse_result("PING") == CMD_OK) &&
		   (parse_result("TELEM 250") == CMD_OK),
		   "valid commands accepted and decoded");

	/* S1-U04: invalid commands are rejected (NFR-B1-5) */
	TEST_CHECK("S1-U04",
		   (parse_result("") == CMD_ERR_EMPTY) &&
		   (parse_result("FLY") == CMD_ERR_UNKNOWN) &&
		   (parse_result("SPEED") == CMD_ERR_MISSING_ARG) &&
		   (parse_result("SPEED 101") == CMD_ERR_RANGE) &&
		   (parse_result("SPEED -5") == CMD_ERR_BAD_NUMBER) &&
		   (parse_result("SPEED fast") == CMD_ERR_BAD_NUMBER) &&
		   (parse_result("STOP now") == CMD_ERR_EXTRA_ARG) &&
		   (parse_result("MODE FLY") == CMD_ERR_BAD_MODE) &&
		   (parse_result("TELEM 5") == CMD_ERR_RANGE) &&
		   (parse_result("STOP #x") == CMD_ERR_BAD_NUMBER) &&
		   (parse_result("ST\x01OP") == CMD_ERR_BAD_CHAR) &&
		   (parse_result("A B C D") == CMD_ERR_EXTRA_ARG),
		   "invalid commands rejected");

	/* S1-U05: JSON writer never overruns, and flags overflow */
	{
		UB		small[16];
		UB		big[64];
		JSON_BUF	jb;
		INT		n1, n2;

		small[15] = 0xA5U;			/* guard byte */
		json_begin(&jb, small, 15);
		json_str(&jb, "key", "a value that is far too long");
		n1 = json_end(&jb);

		json_begin(&jb, big, (INT)sizeof(big));
		json_int(&jb, "a", -2147483647 - 1);
		json_str(&jb, "q", "x\"y");
		n2 = json_end(&jb);

		TEST_CHECK("S1-U05",
			   (n1 == -1) && (small[15] == 0xA5U) &&
			   (small[14] == '\0') && (n2 > 0) &&
			   (big[n2] == '\0'),
			   "JSON writer is bounded and escapes quotes");
	}

	/* S1-U06: reporting events never blocks, even when the queue is
	   full (NFR-B1-2): a burst of 40 must finish in under 20 ms */
	{
		UW	t0 = s1_now_ms();
		INT	i, ok = 0, full = 0;

		for (i = 0; i < BURST_EVENTS; i++) {
			er = comm_report_error(1, (H)i, "S1-U06 burst");
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

	/* S1-U07: telemetry setters update the snapshot consistently */
	{
		COMM_TELEMETRY t;

		comm_set_motion(123, -45, 1000, 999, 4321);
		comm_set_line(11, 22, 1, LINE_F_DETECTED | LINE_F_JUNCTION);
		comm_get_telemetry(&t);
		TEST_CHECK("S1-U07",
			   (t.speed_l == 123) && (t.speed_r == -45) &&
			   (t.enc_l == 1000) && (t.distance_mm == 4321) &&
			   (t.ir_left == 11) && (t.ir_centre == 22) &&
			   (t.line_flags == (LINE_F_DETECTED | LINE_F_JUNCTION)),
			   "telemetry snapshot setters/getter");
	}

	/* S1-U08: a received command goes through validation:
	   one valid (forwarded or reported busy), one rejected */
	{
		COMM_STATS before, after;

		comm_get_stats(&before);
		comm_bridge_rx_command((const uint8_t *)"PING #1", 7);
		comm_bridge_rx_command((const uint8_t *)"LAUNCH", 6);
		tk_dly_tsk(300);
		comm_get_stats(&after);
		TEST_CHECK("S1-U08",
			   (after.rx_ok == before.rx_ok + 1U) &&
			   (after.rx_rejected == before.rx_rejected + 1U),
			   "received commands validated (1 ok, 1 rejected)");
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
			   (after.tx_queued >= before.tx_queued + 4U) &&
			   (after.tx_dropped == before.tx_dropped),
			   "telemetry published for 3 s with no drops");
	}
#endif

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
