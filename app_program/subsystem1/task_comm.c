/*
 *	task_comm.c
 *	Subsystem 1 -- the WiFi & Telemetry task (comm_task).
 *
 *	This task implements the Buddy 1 subsystem statechart:
 *
 *	   INITIALISING -> CONNECTING_WIFI -> CONNECTING_MQTT -> CONNECTED
 *	                        ^                   ^               |
 *	                        |   RECONNECTING <--+---------------+  (link lost)
 *	                        |        |
 *	                        +---- OFFLINE  (WiFi down > 30 s, or MQTT
 *	                                        retries exhausted)
 *
 *	The radio and MQTT session are driven by the kernel's WiFi task
 *	(cyw43_utk_task + mqtt_utk.c).  This task only OBSERVES their state,
 *	and does all the application-level work, at low priority:
 *
 *	  every loop (<= 50 ms):
 *	    - work out the link state, report changes        (FR-B1-1/2/7)
 *	    - validate received commands, forward/ack them    (FR-B1-5, NFR-B1-5)
 *	    - turn queued events into JSON                    (FR-B1-4)
 *	  every COMM_TELEM_PERIOD_MS while CONNECTED:
 *	    - publish the telemetry snapshot                  (FR-B1-3)
 *	  every COMM_HEARTBEAT_MS while CONNECTED:
 *	    - publish a heartbeat/status message              (FR-B1-6)
 *
 *	Telemetry is PAUSED while not connected (it would only be stale when
 *	it finally arrived).  Events are still queued, up to the size of
 *	mbf_tx, and flushed on reconnect.  Nothing here ever stops the robot:
 *	a network fault only affects telemetry and remote commands (NFR-B1-1).
 *
 *	APIs shown: tk_wai_flg (with timeout), tk_rcv_mbf/tk_snd_mbf TMO_POL,
 *	            tk_loc_mtx/tk_unl_mtx
 */

#include "s1_private.h"

#if TM_MQTT
#include "cyw43_utk.h"		/* cyw43_utk_get_status(): safe from any task */
#define S1_CYW43_LINK_UP	3	/* CYW43_LINK_UP in cyw43.h */
#endif

#define COMM_LOOP_MS		50	/* longest sleep between checks */

LOCAL UW	telem_period_ms = COMM_TELEM_PERIOD_MS;
LOCAL UW	telem_seq;
LOCAL UW	event_seq;
LOCAL UW	hb_seq;
LOCAL BOOL	ever_connected;
LOCAL UW	wifi_down_since;	/* 0 = WiFi is up */

LOCAL UB	json_out[COMM_PAYLOAD_MAX + 1];	/* this task only */

/* ---------------------------------------------------------------------
 *  Link state (statechart)
 * --------------------------------------------------------------------- */

LOCAL COMM_STATE compute_state(UW now)
{
#if TM_MQTT
	T_CYW43_UTK_STATUS st;

	cyw43_utk_get_status(&st);

	if (st.init_complete == 0U) {
		return COMM_INITIALISING;
	}
	if (st.ready == 0U) {
		return COMM_DISABLED;		/* radio hardware failed */
	}

	if (st.link_status != S1_CYW43_LINK_UP) {
		/* The kernel re-joins the AP every 5 s by itself. */
		if (wifi_down_since == 0U) {
			wifi_down_since = now;
		}
		if ((now - wifi_down_since) >= COMM_WIFI_OFFLINE_MS) {
			return COMM_OFFLINE;
		}
		return ever_connected ? COMM_RECONNECTING : COMM_CONNECTING_WIFI;
	}
	wifi_down_since = 0U;

	switch (s1_mqtt_state) {
	case COMM_MQTT_CONNECTED:
		return COMM_CONNECTED;
	case COMM_MQTT_BACKOFF:
		return COMM_RECONNECTING;
	case COMM_MQTT_OFFLINE:
		return COMM_OFFLINE;
	default:
		return COMM_CONNECTING_MQTT;
	}
#else
	(void)now;
	return COMM_DISABLED;		/* built without NET=lwip MQTT=1 */
#endif
}

/* ---------------------------------------------------------------------
 *  Message builders
 * --------------------------------------------------------------------- */

/* Finish a JSON message and queue it.  Oversize messages are dropped
 * (and counted) rather than published half-written. */
LOCAL void send_json(JSON_BUF *jb, COMM_TOPIC_ID topic)
{
	INT len = json_end(jb);

	if (len < 0) {
		s1_stats.tx_dropped++;
		tm_printf((UB*)"s1: message too long for topic %d, dropped\n",
			  (INT)topic);
		return;
	}
	if (s1_comm_state == COMM_DISABLED) {
		return;			/* nobody would ever send it */
	}
	(void)s1_tx_enqueue(topic, FALSE, json_out, len);
}

LOCAL void publish_status(UW now)
{
	JSON_BUF	jb;
	COMM_STATS	st;
	UB		ip[16];
	UW		ip_raw = 0U;

#if TM_MQTT
	T_CYW43_UTK_STATUS ws;
	cyw43_utk_get_status(&ws);
	ip_raw = ws.ip_addr;
#endif
	/* lwIP keeps the address in network order: first octet lowest */
	tm_sprintf(ip, (const UB*)"%d.%d.%d.%d",
		   (INT)(ip_raw & 0xffU), (INT)((ip_raw >> 8) & 0xffU),
		   (INT)((ip_raw >> 16) & 0xffU), (INT)((ip_raw >> 24) & 0xffU));

	comm_get_stats(&st);
	hb_seq++;

	json_begin(&jb, json_out, (INT)sizeof(json_out));
	json_bool(&jb, "online", TRUE);
	json_uint(&jb, "seq", hb_seq);
	json_uint(&jb, "up", now);
	json_str(&jb, "link", comm_state_name((COMM_STATE)s1_comm_state));
	json_str(&jb, "ip", (const char *)ip);
	json_uint(&jb, "telem_ms", telem_period_ms);
	json_uint(&jb, "tx", st.tx_queued);
	json_uint(&jb, "drop", st.tx_dropped);
	json_uint(&jb, "evdrop", st.evt_dropped);
	json_uint(&jb, "rx", st.rx_ok);
	json_uint(&jb, "rej", st.rx_rejected);
	json_uint(&jb, "reconn", st.reconnects);
	json_bool(&jb, "demo", S1_STANDALONE_DEMO ? TRUE : FALSE);
	send_json(&jb, COMM_TOPIC_STATUS);

	tm_printf((UB*)"s1: heartbeat #%u  link=%s  tx=%u drop=%u rx=%u rej=%u\n",
		  hb_seq, comm_state_name((COMM_STATE)s1_comm_state),
		  st.tx_queued, st.tx_dropped, st.rx_ok, st.rx_rejected);
}

LOCAL void publish_telemetry(UW now)
{
	JSON_BUF	jb;
	COMM_TELEMETRY	t;
	UB		bc[2];

	comm_get_telemetry(&t);		/* consistent copy, under the mutex */
	telem_seq++;
	bc[0] = (t.last_barcode != 0U) ? t.last_barcode : (UB)'-';
	bc[1] = '\0';

	json_begin(&jb, json_out, (INT)sizeof(json_out));
	json_uint(&jb, "seq", telem_seq);
	json_uint(&jb, "t", now);
	json_str(&jb, "state", robot_state_name((ROBOT_STATE)t.robot_state));
	json_pair(&jb, "spd", t.speed_l, t.speed_r);		/* mm/s */
	json_pair(&jb, "enc", t.enc_l, t.enc_r);		/* counts */
	json_int(&jb, "dist", t.distance_mm);			/* mm */

	json_obj_open(&jb, "line");
	json_int(&jb, "l", t.ir_left);
	json_int(&jb, "c", t.ir_centre);
	json_int(&jb, "r", t.ir_right);
	json_bool(&jb, "det", (t.line_flags & LINE_F_DETECTED) != 0U);
	json_bool(&jb, "lost", (t.line_flags & LINE_F_LOST) != 0U);
	json_bool(&jb, "jct", (t.line_flags & LINE_F_JUNCTION) != 0U);
	json_obj_close(&jb);

	json_obj_open(&jb, "imu");
	json_str(&jb, "ev", imu_event_name((IMU_EVENT)t.imu_event));
	json_int(&jb, "pitch", t.pitch_ddeg);			/* 0.1 deg */
	json_obj_close(&jb);

	json_obj_open(&jb, "hump");
	json_int(&jb, "last", t.hump_last_mm);
	json_int(&jb, "peak", t.hump_peak_mm);
	json_obj_close(&jb);

	json_obj_open(&jb, "bc");
	json_str(&jb, "code", (const char *)bc);
	json_str(&jb, "nav", nav_cmd_name((NAV_CMD)t.last_nav));
	json_obj_close(&jb);

	json_obj_open(&jb, "obs");
	json_int(&jb, "d", t.obs_dist_mm);
	json_int(&jb, "w", t.obs_width_mm);
	json_int(&jb, "lc", t.obs_left_mm);
	json_int(&jb, "rc", t.obs_right_mm);
	json_obj_close(&jb);

	send_json(&jb, COMM_TOPIC_TELEMETRY);
}

LOCAL void publish_event(const COMM_EVENT *ev)
{
	JSON_BUF	jb;
	UB		code[2];

	event_seq++;
	json_begin(&jb, json_out, (INT)sizeof(json_out));
	json_uint(&jb, "seq", event_seq);
	json_uint(&jb, "t", ev->time_ms);

	switch (ev->type) {
	case EVT_BARCODE:
		code[0] = ev->code;
		code[1] = '\0';
		json_str(&jb, "type", "barcode");
		json_str(&jb, "code", (const char *)code);
		json_str(&jb, "nav", nav_cmd_name((NAV_CMD)ev->nav));
		tm_printf((UB*)"s1: EVENT barcode '%c' -> %s\n",
			  (INT)ev->code, nav_cmd_name((NAV_CMD)ev->nav));
		break;

	case EVT_HUMP:
		json_str(&jb, "type", "hump");
		json_int(&jb, "height", ev->v[0]);
		json_int(&jb, "peak", ev->v[1]);
		tm_printf((UB*)"s1: EVENT hump %d mm (peak %d mm)\n",
			  (INT)ev->v[0], (INT)ev->v[1]);
		break;

	case EVT_OBSTACLE:
		json_str(&jb, "type", "obstacle");
		json_int(&jb, "d", ev->v[0]);
		json_int(&jb, "w", ev->v[1]);
		json_int(&jb, "lc", ev->v[2]);
		json_int(&jb, "rc", ev->v[3]);
		json_str(&jb, "action", avoid_action_name((AVOID_ACTION)ev->code));
		tm_printf((UB*)"s1: EVENT obstacle at %d mm, width %d -> %s\n",
			  (INT)ev->v[0], (INT)ev->v[1],
			  avoid_action_name((AVOID_ACTION)ev->code));
		break;

	case EVT_ERROR:
		json_str(&jb, "type", "error");
		json_int(&jb, "sub", ev->code);
		json_int(&jb, "code", ev->v[0]);
		json_str(&jb, "text", ev->text);
		tm_printf((UB*)"s1: EVENT error S%d code %d \"%s\"\n",
			  (INT)ev->code, (INT)ev->v[0], ev->text);
		break;

	default:
		return;			/* unknown type: ignore */
	}
	send_json(&jb, COMM_TOPIC_EVENT);
}

/* ---------------------------------------------------------------------
 *  Commands
 * --------------------------------------------------------------------- */

LOCAL void send_ack(const UB *echo, UW seq, BOOL ok, const char *detail)
{
	JSON_BUF jb;

	json_begin(&jb, json_out, (INT)sizeof(json_out));
	json_str(&jb, "cmd", (const char *)echo);
	json_uint(&jb, "seq", seq);
	json_bool(&jb, "ok", ok);
	json_str(&jb, ok ? "result" : "reason", detail);
	send_json(&jb, COMM_TOPIC_ACK);
}

LOCAL void handle_command(UB *text, INT len, UW now)
{
	PARSED_CMD	pc;
	CMD_RESULT	r;
	INT		i;

	/* Make a printable, NUL-terminated copy for logs and the ack */
	for (i = 0; i < len; i++) {
		if ((text[i] < 0x20U) || (text[i] > 0x7eU)) {
			text[i] = (UB)'?';
		}
	}
	text[len] = '\0';

	r = comm_cmd_parse(text, len, &pc);
	if (r != CMD_OK) {
		s1_stats.rx_rejected++;
		tm_printf((UB*)"s1: REJECTED command \"%s\": %s\n",
			  text, cmd_result_text(r));
		send_ack(text, pc.cmd.seq, FALSE, cmd_result_text(r));
		return;
	}

	s1_stats.rx_ok++;
	switch (pc.local) {
	case LOCAL_PING:
		send_ack(text, pc.cmd.seq, TRUE, "pong");
		break;

	case LOCAL_STATUS:
		send_ack(text, pc.cmd.seq, TRUE, "status sent");
		publish_status(now);
		break;

	case LOCAL_TELEM:
		telem_period_ms = (UW)pc.local_arg;
		tm_printf((UB*)"s1: telemetry period now %u ms\n",
			  telem_period_ms);
		send_ack(text, pc.cmd.seq, TRUE, "telemetry period set");
		break;

	default:
		/* For the Vehicle Controller.  TMO_POL: if its queue is
		 * full we say so in the ack rather than wait. */
		if (tk_snd_mbf(mbf_remote_cmd, &pc.cmd, (INT)sizeof(pc.cmd),
			       TMO_POL) == E_OK) {
			tm_printf((UB*)"s1: command %s (arg %d, #%u) -> controller\n",
				  remote_cmd_name((REMOTE_CMD_TYPE)pc.cmd.type),
				  (INT)pc.cmd.arg, pc.cmd.seq);
			send_ack(text, pc.cmd.seq, TRUE, "forwarded");
		} else {
			s1_stats.fwd_failed++;
			tm_printf((UB*)"s1: command %s NOT delivered: "
				  "controller queue full\n",
				  remote_cmd_name((REMOTE_CMD_TYPE)pc.cmd.type));
			send_ack(text, pc.cmd.seq, FALSE, "controller busy");
		}
		break;
	}
}

LOCAL void drain_commands(UW now)
{
	UB	buf[COMM_CMD_MAX + 2];	/* +1 over-long marker, +1 NUL */
	INT	n;

	while ((n = tk_rcv_mbf(mbf_rx, buf, TMO_POL)) > 0) {
		handle_command(buf, n, now);
	}
}

LOCAL void drain_events(void)
{
	COMM_EVENT ev;

	while (tk_rcv_mbf(mbf_events, &ev, TMO_POL) == (INT)sizeof(ev)) {
		publish_event(&ev);
	}
}

/* ---------------------------------------------------------------------
 *  The task
 * --------------------------------------------------------------------- */

EXPORT void comm_task(INT stacd, void *exinf)
{
	COMM_STATE	state = COMM_INITIALISING;
	UW		last_telem;
	UW		last_hb;
	UINT		ptn;

	(void)stacd; (void)exinf;

	tm_printf((UB*)"s1: WiFi & Telemetry task started%s\n",
		  S1_STANDALONE_DEMO ? " (STANDALONE DEMO MODE)" : "");
#if !TM_MQTT
	tm_printf((UB*)"s1: built without NET=lwip MQTT=1 -> no network, "
		  "events are printed only\n");
#endif

	last_telem = s1_now_ms();
	last_hb    = last_telem;

	while (1) {
		COMM_STATE	next;
		UW		now;

		/* Sleep until something is queued, or at most COMM_LOOP_MS.
		 * E_TMOUT is the normal "nothing happened" case. */
		(void)tk_wai_flg(flg_comm, FLG_COMM_ALL, TWF_ORW | TWF_BITCLR,
				 &ptn, COMM_LOOP_MS);
		now = s1_now_ms();

		/* 1. Link state machine */
		next = compute_state(now);
		if (next != state) {
			tm_printf((UB*)"s1: link %s -> %s\n",
				  comm_state_name(state), comm_state_name(next));
			if (next == COMM_CONNECTED) {
				if (ever_connected) {
					s1_stats.reconnects++;
				}
				ever_connected = TRUE;
				last_telem = now;
				last_hb    = now;
			}
			state = next;
			s1_comm_state = (INT)next;
			if (next == COMM_CONNECTED) {
				publish_status(now);	/* "I'm back" */
			}
		}

		/* 2. Commands and events */
		drain_commands(now);
		drain_events();

		/* 3. Periodic messages, only while connected */
		if (state == COMM_CONNECTED) {
			if ((now - last_telem) >= telem_period_ms) {
				last_telem = now;
				publish_telemetry(now);
			}
			if ((now - last_hb) >= COMM_HEARTBEAT_MS) {
				last_hb = now;
				publish_status(now);
			}
		}
	}
}
