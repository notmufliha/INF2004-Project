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
 *	    - keep FAULT_WIFI_LOSS in the shared fault mask in step
 *	    - validate received commands, forward/ack them    (FR-B1-5, NFR-B1-5)
 *	    - pack queued events into binary frames           (FR-B1-4)
 *	  every telemetry period while CONNECTED:
 *	    - publish the telemetry snapshot (28 bytes)       (FR-B1-3)
 *	  every COMM_HEARTBEAT_MS while CONNECTED:
 *	    - publish a heartbeat/status frame (23 bytes)     (FR-B1-6)
 *
 *	Telemetry is PAUSED while not connected (it would only be stale when
 *	it finally arrived).  Events are still queued, up to the size of
 *	mbf_tx, and flushed on reconnect.  Nothing here ever stops the robot:
 *	a network fault only affects telemetry and remote commands (NFR-B1-1).
 *
 *	APIs shown: tk_wai_flg (with timeout), tk_rcv_mbf/tk_snd_mbf TMO_POL,
 *	            tk_set_flg/tk_clr_flg (fault mask)
 */

#include "s1_private.h"

#if TM_MQTT
#include "cyw43_utk.h"		/* cyw43_utk_get_status(): safe from any task */
#define S1_CYW43_LINK_UP	3	/* CYW43_LINK_UP in cyw43.h */
#endif

#if S1_POLL_B2_ENCODERS
#include "../subsystem2/encoder_control.h"	/* encoder_get_counts() */
#endif

#define COMM_LOOP_MS		50	/* longest sleep between checks */

LOCAL UB	telem_period_ds = COMM_TELEM_PERIOD_DS;	/* 7 bits used */
LOCAL UH	telem_seq;
LOCAL UB	event_seq;
LOCAL UH	hb_seq;
LOCAL BOOL	ever_connected;
LOCAL UW	wifi_down_since;	/* 0 = WiFi is up */

LOCAL UB	frame[COMM_PAYLOAD_MAX];	/* this task only */

LOCAL UW time_ds(UW now_ms)
{
	return (now_ms / 100U) & 0xFFFFFU;	/* 20 bits */
}

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

/* WiFi-loss bit of the shared fault mask: set once a link we had is
 * lost (or we gave up), cleared when connected again. */
LOCAL void update_wifi_fault(COMM_STATE st)
{
	if (st == COMM_CONNECTED) {
		fault_clear(FAULT_WIFI_LOSS);
	} else if ((st == COMM_OFFLINE) ||
		   (ever_connected && (st == COMM_RECONNECTING))) {
		fault_raise(FAULT_WIFI_LOSS);
	}
}

/* ---------------------------------------------------------------------
 *  B2 encoders (S1_POLL_B2_ENCODERS): speed = change in count over the
 *  telemetry period, in counts/s like B2's own speed task.
 * --------------------------------------------------------------------- */
#if S1_POLL_B2_ENCODERS
LOCAL void poll_encoders(UW now)
{
	static INT	prev_l, prev_r;
	static UW	prev_t;
	static BOOL	have_prev;
	INT		l, r;
	W		sl = 0, sr = 0;
	UW		dt = now - prev_t;

	encoder_get_counts(&l, &r);	/* consistent pair, ISR-safe snapshot */
	if (have_prev && (dt > 0U)) {
		sl = ((W)(l - prev_l) * 1000) / (W)dt;
		sr = ((W)(r - prev_r) * 1000) / (W)dt;
	}
	prev_l = l;
	prev_r = r;
	prev_t = now;
	have_prev = TRUE;
	comm_set_motion(sl, sr, l, r);
}
#endif

/* ---------------------------------------------------------------------
 *  Frame builders
 * --------------------------------------------------------------------- */

/* Queue one packed frame.  A packing overflow cannot happen with the
 * fixed layouts, but is still dropped and counted rather than sent. */
LOCAL void send_frame(INT len, COMM_TOPIC_ID topic)
{
	if (len < 0) {
		s1_stats.tx_dropped++;
		tm_printf((UB*)"s1: frame too long for topic %d, dropped\n",
			  (INT)topic);
		return;
	}
	if (s1_comm_state == COMM_DISABLED) {
		return;			/* nobody would ever send it */
	}
	(void)s1_tx_enqueue(topic, FALSE, frame, len);
}

LOCAL void publish_status(UW now)
{
	STATUS_INFO	s;

#if TM_MQTT
	T_CYW43_UTK_STATUS ws;
	cyw43_utk_get_status(&ws);
	s.ip = ws.ip_addr;
#else
	s.ip = 0U;
#endif
	comm_get_stats(&s.st);
	hb_seq++;
	s.link     = s1_comm_state;
	s.demo     = S1_STANDALONE_DEMO ? TRUE : FALSE;
	s.seq      = hb_seq;
	s.time_ds  = time_ds(now);
	s.telem_ds = telem_period_ds;
	s.faults   = fault_get();

	send_frame(pack_status(frame, (INT)sizeof(frame), &s),
		   COMM_TOPIC_STATUS);

	tm_printf((UB*)"s1: heartbeat #%d  link=%s  faults=0x%02x  tx=%d "
		  "drop=%d rx=%d rej=%d\n",
		  (INT)hb_seq, comm_state_name((COMM_STATE)s1_comm_state),
		  (INT)s.faults, (INT)s.st.tx_queued, (INT)s.st.tx_dropped,
		  (INT)s.st.rx_ok, (INT)s.st.rx_rejected);
}

LOCAL void publish_telemetry(UW now)
{
	COMM_TELEMETRY t;

#if S1_POLL_B2_ENCODERS
	poll_encoders(now);
#endif
	comm_get_telemetry(&t);		/* consistent copy, under the mutex */
	telem_seq++;
	send_frame(pack_telemetry(frame, (INT)sizeof(frame), telem_seq,
				  time_ds(now), fault_get(), &t),
		   COMM_TOPIC_TELEMETRY);
}

LOCAL void publish_event(const COMM_EVENT *ev)
{
	event_seq++;

	switch (ev->type) {
	case EVT_BARCODE:
		tm_printf((UB*)"s1: EVENT barcode '%c' -> %s\n",
			  (INT)(COMM_BARCODE_FIRST - 1 + ev->a),
			  nav_cmd_name((NAV_CMD)ev->e));
		break;
	case EVT_HUMP:
		tm_printf((UB*)"s1: EVENT hump %d mm (peak %d mm, valid %d)\n",
			  (INT)ev->a, (INT)ev->b, (INT)ev->flag);
		break;
	case EVT_OBSTACLE:
		tm_printf((UB*)"s1: EVENT obstacle at %d cm, width %d -> %s\n",
			  (INT)ev->a, (INT)ev->b,
			  avoid_action_name((AVOID_ACTION)ev->e));
		break;
	default:
		tm_printf((UB*)"s1: EVENT error 0x%02x (S%d #%d) detail %d\n",
			  (INT)ev->a, (INT)ERR_SUBSYS(ev->a),
			  (INT)ERR_NUMBER(ev->a), (INT)ev->b);
		break;
	}
	send_frame(pack_event(frame, (INT)sizeof(frame), event_seq, ev),
		   COMM_TOPIC_EVENT);
}

/* ---------------------------------------------------------------------
 *  Commands
 * --------------------------------------------------------------------- */

LOCAL void send_ack(const PARSED_CMD *pc, CMD_RESULT r)
{
	send_frame(pack_ack(frame, (INT)sizeof(frame), pc->seq, pc->id,
			    (UB)r),
		   COMM_TOPIC_ACK);
}

LOCAL void handle_command(const UB *data, INT len, UW now)
{
	PARSED_CMD	pc;
	CMD_RESULT	r;

	r = comm_cmd_decode(data, len, &pc);
	if (r != CMD_OK) {
		s1_stats.rx_rejected++;
		tm_printf((UB*)"s1: REJECTED command (%d bytes, id %d, #%d): %s\n",
			  len, (INT)pc.id, (INT)pc.seq, cmd_result_text(r));
		send_ack(&pc, r);
		return;
	}

	s1_stats.rx_ok++;
	switch (pc.id) {
	case WCMD_PING:
		send_ack(&pc, CMD_OK_PONG);
		break;

	case WCMD_STATUS:
		send_ack(&pc, CMD_OK_STATUS);
		publish_status(now);
		break;

	case WCMD_TELEM:
		telem_period_ds = pc.arg;
		tm_printf((UB*)"s1: telemetry period now %d ms\n",
			  (INT)telem_period_ds * 100);
		send_ack(&pc, CMD_OK_TELEM);
		break;

	default:
		/* For the Vehicle Controller.  TMO_POL: if its queue is
		 * full we say so in the ack rather than wait. */
		if (tk_snd_mbf(mbf_remote_cmd, &pc.cmd, (INT)sizeof(pc.cmd),
			       TMO_POL) == E_OK) {
			tm_printf((UB*)"s1: command %s (arg %d, #%d) -> controller\n",
				  remote_cmd_name((REMOTE_CMD_TYPE)pc.cmd.type),
				  (INT)pc.cmd.arg, (INT)pc.cmd.seq);
			send_ack(&pc, CMD_OK);
		} else {
			s1_stats.fwd_failed++;
			tm_printf((UB*)"s1: command %s NOT delivered: "
				  "controller queue full\n",
				  remote_cmd_name((REMOTE_CMD_TYPE)pc.cmd.type));
			send_ack(&pc, CMD_ERR_BUSY);
		}
		break;
	}
}

LOCAL void drain_commands(UW now)
{
	UB	buf[COMM_CMD_MAX + 1];	/* +1 over-long marker */
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
			if (next == COMM_CONNECTED) {
				if (ever_connected) {
					s1_stats.reconnects++;
				}
				ever_connected = TRUE;
				last_telem = now;
				last_hb    = now;
			}
			s1_comm_state = (UB)next;
			update_wifi_fault(next);
			tm_printf((UB*)"s1: link %s -> %s  faults=0x%02x\n",
				  comm_state_name(state), comm_state_name(next),
				  (INT)fault_get());
			state = next;
			if (next == COMM_CONNECTED) {
				publish_status(now);	/* "I'm back" */
			}
		}

		/* 2. Commands and events */
		drain_commands(now);
		drain_events();

		/* 3. Periodic messages, only while connected */
		if (state == COMM_CONNECTED) {
			if ((now - last_telem) >= ((UW)telem_period_ds * 100U)) {
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
