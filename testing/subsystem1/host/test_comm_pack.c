/*
 *	test_comm_pack.c
 *	Host-side test of subsystem 1's binary wire format (comm_pack.c),
 *	built with the laptop's gcc against the real kernel headers.  No
 *	Pico needed.  Run everything with:  ./run_host_tests.sh
 *
 *	  ./test_comm_pack vectors   pack min / middle / max / invalid values
 *	                             of every field; print one line per
 *	                             frame:  kind hex field=value ...
 *	                             test_wire.py decodes the hex with the
 *	                             laptop decoder and compares each field.
 *	  ./test_comm_pack decode    read hex command frames from stdin (one
 *	                             per line), print "result id arg seq".
 *	                             test_wire.py feeds it frames made by the
 *	                             laptop encoder.
 *	  ./test_comm_pack sizes     message and struct sizes, before/after
 *
 *	Exit status 0 = every C-side check passed.
 */

#include <stdio.h>
#include <string.h>
#include "../../../app_program/subsystem1/s1_private.h"

LOCAL INT failures;

#define CHECK_EQ(what, got, want)					\
	do {								\
		if ((W)(got) != (W)(want)) {				\
			printf("FAIL %s: got %ld want %ld\n", (what),	\
			       (long)(got), (long)(want));		\
			failures++;					\
		}							\
	} while (0)

LOCAL void print_hex(const char *kind, const UB *b, INT n)
{
	INT i;

	printf("%s ", kind);
	for (i = 0; i < n; i++) {
		printf("%02x", b[i]);
	}
}

/* ------------------------------------------------------------------ */

LOCAL void telemetry_vector(W spd_l, W spd_r, W enc_l, W enc_r, UH ir_l,
			    UH ir_c, UB ir_r, UB line, UB motion, H pitch,
			    UB hump_valid, UB hump_last, UB hump_peak,
			    UH d, UH w, UH lc, UH rc, UH seq, UW tds,
			    UB state, UB faults, UB barcode, UB nav)
{
	COMM_TELEMETRY	t = {0};
	UB		buf[COMM_PAYLOAD_MAX];
	INT		n;
	BITBUF		b;

	/* Store exactly as comm_bridge.c does (saturate, then bit-field) */
	t.robot_state  = state;
	t.speed_l      = (INT)sat_s(spd_l, COMM_SPEED_MAX);
	t.speed_r      = (INT)sat_s(spd_r, COMM_SPEED_MAX);
	t.enc_l        = (UINT)enc_l & 0xFFFFFFU;
	t.enc_r        = (UINT)enc_r & 0xFFFFFFU;
	t.ir_left      = sat_u(ir_l, COMM_IR_MAX);
	t.ir_centre    = sat_u(ir_c, COMM_IR_MAX);
	t.ir_right     = ir_r;
	t.line_flags   = line;
	t.barcode      = barcode;
	t.nav          = nav;
	t.motion       = motion;
	t.pitch_ddeg   = (INT)sat_s(pitch, COMM_PITCH_MAX);
	t.hump_valid   = hump_valid;
	t.hump_last_mm = hump_last;
	t.hump_peak_mm = hump_peak;
	t.obs_dist_cm  = d;
	t.obs_width_cm = w;
	t.obs_left_cm  = lc;
	t.obs_right_cm = rc;

	n = pack_telemetry(buf, (INT)sizeof(buf), seq, tds, faults, &t);
	CHECK_EQ("telemetry length", n, WIRE_TELEM_LEN);

	/* C-side round trip of a few fields with the bit reader */
	b.buf = buf; b.cap_bits = (UH)(n * 8); b.pos = 0U; b.overflow = FALSE;
	CHECK_EQ("telem ver", bits_get(&b, 3), WIRE_VERSION);
	CHECK_EQ("telem seq", bits_get(&b, 16), seq);
	CHECK_EQ("telem time", bits_get(&b, 20), tds & 0xFFFFFU);
	(void)bits_get(&b, 4 + 8);
	CHECK_EQ("telem spd_l", bits_get_signed(&b, 14), t.speed_l);
	CHECK_EQ("telem spd_r", bits_get_signed(&b, 14), t.speed_r);

	print_hex("telemetry", buf, n);
	printf(" seq=%u t_ds=%lu state=%u faults=%u spd_l=%d spd_r=%d "
	       "enc_l=%ld enc_r=%ld ir_l=%u ir_c=%u ir_r=%u line=%u "
	       "barcode=%u nav=%u motion=%u pitch_dd=%d hump_valid=%u "
	       "hump_last_mm=%u hump_peak_mm=%u obs_d_cm=%u obs_w_cm=%u "
	       "obs_lc_cm=%u obs_rc_cm=%u\n",
	       seq, (unsigned long)tds, state, faults, (int)t.speed_l,
	       (int)t.speed_r,
	       /* sign-extend the 24-bit totals, as the decoder does */
	       (long)(((W)t.enc_l ^ 0x800000L) - 0x800000L),
	       (long)(((W)t.enc_r ^ 0x800000L) - 0x800000L),
	       t.ir_left, t.ir_centre, t.ir_right, line, barcode, nav,
	       motion, (int)t.pitch_ddeg, hump_valid, hump_last, hump_peak,
	       d, w, lc, rc);
}

LOCAL void event_vector(UB type, UB seq, UW tds, UW a, UW bb, UW c, UW d,
			UW e, UW flag)
{
	COMM_EVENT	ev = {0};
	UB		buf[COMM_PAYLOAD_MAX];
	INT		n;
	static const INT want_len[4] = { 5, 6, 9, 7 };

	ev.type = type; ev.time_ds = tds; ev.a = a; ev.b = bb; ev.c = c;
	ev.d = d; ev.e = e; ev.flag = flag;
	n = pack_event(buf, (INT)sizeof(buf), seq, &ev);
	CHECK_EQ("event length", n, want_len[type]);

	print_hex("event", buf, n);
	printf(" type=%u seq=%u t_ds=%lu", type, seq, (unsigned long)tds);
	switch (type) {
	case EVT_BARCODE:
		printf(" code=%lu nav=%lu\n", (unsigned long)a, (unsigned long)e);
		break;
	case EVT_HUMP:
		printf(" valid=%lu height_mm=%lu peak_mm=%lu\n",
		       (unsigned long)flag, (unsigned long)a, (unsigned long)bb);
		break;
	case EVT_OBSTACLE:
		printf(" d_cm=%lu w_cm=%lu lc_cm=%lu rc_cm=%lu action=%lu\n",
		       (unsigned long)a, (unsigned long)bb, (unsigned long)c,
		       (unsigned long)d, (unsigned long)e);
		break;
	default:
		printf(" err=%lu detail=%lu\n", (unsigned long)a,
		       (unsigned long)bb);
		break;
	}
}

LOCAL void status_vector(UB link, BOOL demo, UH seq, UW tds, UW ip,
			 UB telem_ds, UB faults, UH cnt)
{
	STATUS_INFO	s = {0};
	UB		buf[COMM_PAYLOAD_MAX];
	INT		n;

	s.link = link; s.demo = demo; s.seq = seq; s.time_ds = tds;
	s.ip = ip; s.telem_ds = telem_ds; s.faults = faults;
	s.st.tx_queued = cnt; s.st.tx_dropped = (UH)(cnt ^ 0x5555U);
	s.st.evt_dropped = 1U; s.st.rx_ok = (UH)(cnt >> 1);
	s.st.rx_rejected = 2U; s.st.reconnects = (UH)(cnt & 0xFFU);
	n = pack_status(buf, (INT)sizeof(buf), &s);
	CHECK_EQ("status length", n, WIRE_STATUS_LEN);
	CHECK_EQ("status byte 0 = online byte", buf[0] & 0xF0U,
		 (UB)COMM_STATUS_ONLINE[0]);

	print_hex("status", buf, n);
	printf(" online=1 link=%u demo=%u seq=%u t_ds=%lu ip=%lu "
	       "telem_ds=%u faults=%u tx=%u drop=%u evdrop=1 rx=%u rej=2 "
	       "reconn=%u\n", link, demo ? 1U : 0U, seq, (unsigned long)tds,
	       (unsigned long)ip, telem_ds, faults, cnt,
	       (UH)(cnt ^ 0x5555U), (UH)(cnt >> 1), cnt & 0xFFU);
}

LOCAL void ack_vector(UB seq, UB id, UB result)
{
	UB	buf[COMM_PAYLOAD_MAX];
	INT	n = pack_ack(buf, (INT)sizeof(buf), seq, id, result);

	CHECK_EQ("ack length", n, WIRE_ACK_LEN);
	print_hex("cmd/ack", buf, n);
	printf(" seq=%u cmd=%u ok=%u result=%u\n", seq, id,
	       (result < CMD_ERR_FIRST) ? 1U : 0U, result);
}

LOCAL void vectors(void)
{
	/* minimum / zero */
	telemetry_vector(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	/* maximum of every field, B2 max speed 5000 counts/s */
	telemetry_vector(5000, -5000, 8388607, -8388608, 4095, 4095, 1, 7,
			 7, 511, 1, 126, 126, 510, 126, 510, 510, 65535,
			 0xFFFFFU, ROBOT_SAFE_STOP, 0xFF, 4, NAV_UTURN);
	/* saturation and invalid markers */
	telemetry_vector(20000, -20000, 1000, -1, 9999, 4096, 1, 1, 3,
			 -900, 0, 0, 0, 0x1FF, 0x7F, 0x1FF, 0x1FF, 1234,
			 98765, ROBOT_LINE_FOLLOW, FAULT_WIFI_LOSS, 1, NAV_LEFT);
	/* typical, with a wrapped encoder total (> 2^24 counts) */
	telemetry_vector(3000, 2987, 16777216 + 42, 123456, 812, 2400, 0,
			 LINE_F_DETECTED, 1, -45, 1, 23, 31, 250, 12, 30, 9,
			 42, 2100, ROBOT_HUMP, FAULT_NO_ECHO | FAULT_IMU, 2,
			 NAV_RIGHT);

	event_vector(EVT_BARCODE, 0, 0, 1, 0, 0, 0, NAV_LEFT, 0);
	event_vector(EVT_BARCODE, 255, 0xFFFFFU, 4, 0, 0, 0, NAV_UTURN, 0);
	event_vector(EVT_HUMP, 7, 160, 23, 31, 0, 0, 0, 1);
	event_vector(EVT_HUMP, 8, 161, 0, 0, 0, 0, 0, 0);
	event_vector(EVT_OBSTACLE, 9, 420, 25, 12, 30, 9, AVOID_LEFT, 0);
	event_vector(EVT_OBSTACLE, 10, 421, 0x1FF, 0x7F, 510, 0, AVOID_REVERSE, 0);
	event_vector(EVT_ERROR, 11, 430, ERR_S2_ENCODER, 17, 0, 0, 0, 0);
	event_vector(EVT_ERROR, 12, 431, ERR_S5(9), 255, 0, 0, 0, 0);

	status_vector(COMM_CONNECTED, FALSE, 12, 600, 0x1701A8C0U, 5, 0, 151);
	status_vector(COMM_DISABLED, TRUE, 65535, 0xFFFFFU, 0xFFFFFFFFU, 100,
		      0xFF, 65535);

	ack_vector(5, RCMD_SPEED, CMD_OK);
	ack_vector(0, 0, CMD_ERR_LENGTH);
	ack_vector(255, WCMD_PING, CMD_OK_PONG);
	ack_vector(7, RCMD_STOP, CMD_ERR_BUSY);
}

/* ------------------------------------------------------------------ */

LOCAL INT hex_to_bytes(const char *s, UB *out, INT cap)
{
	INT		n = 0;
	unsigned	v;

	while ((s[0] != '\0') && (s[0] != '\n') && (s[0] != '\r')) {
		if ((n >= cap) || (sscanf(s, "%2x", &v) != 1)) {
			return -1;
		}
		out[n++] = (UB)v;
		s += 2;
	}
	return n;
}

LOCAL void decode_stdin(void)
{
	char		line[64];
	UB		f[8];
	PARSED_CMD	pc;

	while (fgets(line, (int)sizeof(line), stdin) != NULL) {
		INT n = hex_to_bytes(line, f, (INT)sizeof(f));
		CMD_RESULT r = comm_cmd_decode(f, (n < 0) ? 0 : n, &pc);
		printf("%d %u %u %u\n", (int)r, pc.id, pc.arg, pc.seq);
	}
}

/* ------------------------------------------------------------------ */

LOCAL void sizes(void)
{
	/* "before" = the JSON/text version (commit ec0b859): MQTT.md's
	 * example messages (typical) and every field at its widest value
	 * (worst), as produced by the old comm_json.c */
	printf("%-28s %12s %8s\n", "item", "before", "after");
	printf("%-28s %12s %8d\n", "telemetry frame (B)", "290 / 374",
	       WIRE_TELEM_LEN);
	printf("%-28s %12s %8s\n", "event frame (B)", "55-86 / 117", "5-9");
	printf("%-28s %12s %8d\n", "status frame (B)", "157 / 233",
	       WIRE_STATUS_LEN);
	printf("%-28s %12s %8d\n", "command frame (B)", "11 / 64",
	       WIRE_CMD_LEN);
	printf("%-28s %12s %8d\n", "ack frame (B)", "60 / 143", WIRE_ACK_LEN);
	printf("%-28s %8d %8d\n", "COMM_TELEMETRY struct", 52,
	       (int)sizeof(COMM_TELEMETRY));
	printf("%-28s %8d %8d\n", "COMM_EVENT struct", 44,
	       (int)sizeof(COMM_EVENT));
	printf("%-28s %8d %8d\n", "REMOTE_CMD struct", 8,
	       (int)sizeof(REMOTE_CMD));
	printf("%-28s %8d %8d\n", "COMM_STATS struct", 32,
	       (int)sizeof(COMM_STATS));
	CHECK_EQ("sizeof COMM_TELEMETRY", sizeof(COMM_TELEMETRY), 24);
	CHECK_EQ("sizeof COMM_EVENT", sizeof(COMM_EVENT), 8);
	CHECK_EQ("sizeof REMOTE_CMD", sizeof(REMOTE_CMD), 4);
}

int main(int argc, char **argv)
{
	const char *mode = (argc > 1) ? argv[1] : "vectors";

	if (strcmp(mode, "decode") == 0) {
		decode_stdin();
	} else if (strcmp(mode, "sizes") == 0) {
		sizes();
	} else {
		vectors();
	}
	if (failures != 0) {
		fprintf(stderr, "%d C-side check(s) FAILED\n", failures);
	}
	return (failures != 0) ? 1 : 0;
}
