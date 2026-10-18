/*
 *	comm_bridge.c
 *	Subsystem 1 -- the two "edges" of the communication subsystem:
 *
 *	  1. The public, non-blocking API other subsystems call
 *	     (comm_set_xxx, comm_report_xxx, comm_get_xxx).
 *	  2. The three hooks the kernel's WiFi task calls (comm_bridge_xxx,
 *	     declared in comm_bridge.h).
 *
 *	Every kernel call here either polls (TMO_POL) or waits at most
 *	TELEM_LOCK_TMO ms for a mutex that is only ever held for a short
 *	memcpy.  Nothing here ever waits for the network (NFR-B1-2).
 *
 *	APIs shown: tk_snd_mbf/tk_rcv_mbf with TMO_POL, tk_loc_mtx with a
 *	            timeout, tk_set_flg, tk_get_otm
 */

#include "s1_private.h"

EXPORT volatile INT	s1_mqtt_state = COMM_MQTT_IDLE;
EXPORT volatile INT	s1_comm_state = COMM_INITIALISING;
EXPORT COMM_STATS	s1_stats;
EXPORT COMM_TELEMETRY	s1_telem;

/* Written only by the WiFi task, read by anyone */
LOCAL volatile UW	bridge_rx_dropped;

/* Milliseconds since boot (wraps after ~49 days, fine for a demo run) */
EXPORT UW s1_now_ms(void)
{
	SYSTIM	tim;

	tk_get_otm(&tim);	/* operating time: never set back */
	return tim.lo;
}

/* Lock the snapshot.  FALSE = could not get it in time, skip the update.
 * The holder only copies ~60 bytes, and TA_INHERIT raises a low-priority
 * holder, so in practice this never times out. */
LOCAL BOOL telem_lock(void)
{
	return (tk_loc_mtx(mtx_telem, TELEM_LOCK_TMO) == E_OK) ? TRUE : FALSE;
}

LOCAL void telem_unlock(void)
{
	tk_unl_mtx(mtx_telem);
}

/* =====================================================================
 *  1a. Telemetry snapshot setters (IF-S1-TLM)
 * ===================================================================== */

EXPORT void comm_set_robot_state(ROBOT_STATE state)
{
	if (telem_lock()) {
		s1_telem.robot_state = (UB)state;
		telem_unlock();
	}
}

EXPORT void comm_set_motion(H speed_l, H speed_r, W enc_l, W enc_r,
			    W distance_mm)
{
	if (telem_lock()) {
		s1_telem.speed_l     = speed_l;
		s1_telem.speed_r     = speed_r;
		s1_telem.enc_l       = enc_l;
		s1_telem.enc_r       = enc_r;
		s1_telem.distance_mm = distance_mm;
		telem_unlock();
	}
}

EXPORT void comm_set_line(UH ir_left, UH ir_centre, UB ir_right, UB flags)
{
	if (telem_lock()) {
		s1_telem.ir_left    = ir_left;
		s1_telem.ir_centre  = ir_centre;
		s1_telem.ir_right   = ir_right;
		s1_telem.line_flags = flags;
		telem_unlock();
	}
}

EXPORT void comm_set_imu(IMU_EVENT ev, H pitch_ddeg, H hump_peak_mm)
{
	if (telem_lock()) {
		s1_telem.imu_event  = (UB)ev;
		s1_telem.pitch_ddeg = pitch_ddeg;
		if (hump_peak_mm > s1_telem.hump_peak_mm) {
			s1_telem.hump_peak_mm = hump_peak_mm;
		}
		telem_unlock();
	}
}

EXPORT void comm_set_obstacle(H dist_mm, H width_mm, H left_mm, H right_mm)
{
	if (telem_lock()) {
		s1_telem.obs_dist_mm  = dist_mm;
		s1_telem.obs_width_mm = width_mm;
		s1_telem.obs_left_mm  = left_mm;
		s1_telem.obs_right_mm = right_mm;
		telem_unlock();
	}
}

EXPORT void comm_get_telemetry(COMM_TELEMETRY *out)
{
	if (telem_lock()) {
		*out = s1_telem;
		telem_unlock();
	}
}

/* =====================================================================
 *  1b. One-off events (IF-S1-EVT)
 * ===================================================================== */

LOCAL ER post_event(COMM_EVENT *ev)
{
	ER er;

	ev->time_ms = s1_now_ms();
	er = tk_snd_mbf(mbf_events, ev, (INT)sizeof(*ev), TMO_POL);
	if (er == E_OK) {
		tk_set_flg(flg_comm, FLG_COMM_EVENT);	/* wake comm_task */
	} else {
		if (telem_lock()) {			/* several writers */
			s1_stats.evt_dropped++;
			telem_unlock();
		}
	}
	return er;
}

EXPORT ER comm_report_barcode(UB code, NAV_CMD nav)
{
	COMM_EVENT ev = {0};

	ev.type = EVT_BARCODE;
	ev.code = code;
	ev.nav  = (UB)nav;

	if (telem_lock()) {			/* also kept in telemetry */
		s1_telem.last_barcode = code;
		s1_telem.last_nav     = (UB)nav;
		telem_unlock();
	}
	return post_event(&ev);
}

EXPORT ER comm_report_hump(H height_mm, H peak_mm)
{
	COMM_EVENT ev = {0};

	ev.type = EVT_HUMP;
	ev.v[0] = height_mm;
	ev.v[1] = peak_mm;

	if (telem_lock()) {
		s1_telem.hump_last_mm = height_mm;
		if (peak_mm > s1_telem.hump_peak_mm) {
			s1_telem.hump_peak_mm = peak_mm;
		}
		telem_unlock();
	}
	return post_event(&ev);
}

EXPORT ER comm_report_obstacle(H dist_mm, H width_mm, H left_mm,
			       H right_mm, AVOID_ACTION action)
{
	COMM_EVENT ev = {0};

	ev.type = EVT_OBSTACLE;
	ev.code = (UB)action;
	ev.v[0] = dist_mm;
	ev.v[1] = width_mm;
	ev.v[2] = left_mm;
	ev.v[3] = right_mm;

	comm_set_obstacle(dist_mm, width_mm, left_mm, right_mm);
	return post_event(&ev);
}

EXPORT ER comm_report_error(UB subsystem, H code, const char *text)
{
	COMM_EVENT	ev = {0};
	INT		i = 0;

	ev.type = EVT_ERROR;
	ev.code = subsystem;
	ev.v[0] = code;
	if (text != NULL) {
		while ((i < COMM_ERR_TEXT_MAX) && (text[i] != '\0')) {
			ev.text[i] = text[i];
			i++;
		}
	}
	ev.text[i] = '\0';
	return post_event(&ev);
}

/* =====================================================================
 *  1c. Link status
 * ===================================================================== */

EXPORT COMM_STATE comm_get_state(void)
{
	return (COMM_STATE)s1_comm_state;
}

EXPORT BOOL comm_is_connected(void)
{
	return (s1_comm_state == COMM_CONNECTED) ? TRUE : FALSE;
}

EXPORT void comm_get_stats(COMM_STATS *out)
{
	if (telem_lock()) {
		*out = s1_stats;
		telem_unlock();
	}
	out->rx_dropped = bridge_rx_dropped;
}

/* =====================================================================
 *  1d. Outgoing queue, filled by comm_task only
 * ===================================================================== */

EXPORT BOOL s1_tx_enqueue(COMM_TOPIC_ID topic, BOOL retain,
			  const UB *payload, INT len)
{
	static UB	rec[TX_RECORD_MAX];	/* comm_task only: not re-entrant */
	INT		i;
	ER		er;

	if ((len < 0) || (len > COMM_PAYLOAD_MAX)) {
		s1_stats.tx_dropped++;
		return FALSE;
	}

	rec[0] = (UB)topic;
	rec[1] = retain ? 1U : 0U;
	for (i = 0; i < len; i++) {
		rec[TX_RECORD_HDR + i] = payload[i];
	}

	er = tk_snd_mbf(mbf_tx, rec, TX_RECORD_HDR + len, TMO_POL);
	if (er == E_OK) {
		s1_stats.tx_queued++;
		return TRUE;
	}
	s1_stats.tx_dropped++;		/* network slower than we produce */
	return FALSE;
}

/* =====================================================================
 *  2. Hooks called by the kernel's WiFi task (comm_bridge.h)
 * ===================================================================== */

int32_t comm_bridge_tx_pop(uint8_t *topic_id, uint8_t *retain,
			   uint8_t *payload, uint16_t payload_max)
{
	static UB	rec[TX_RECORD_MAX];	/* WiFi task only */
	INT		n;
	INT		i;

	if (mbf_tx <= 0) {
		return -1;		/* subsystem1_init() has not run yet */
	}
	n = tk_rcv_mbf(mbf_tx, rec, TMO_POL);
	if (n < TX_RECORD_HDR) {
		return -1;		/* E_TMOUT: nothing queued */
	}

	n -= TX_RECORD_HDR;
	if (n > (INT)payload_max) {
		n = (INT)payload_max;
	}
	*topic_id = rec[0];
	*retain   = rec[1];
	for (i = 0; i < n; i++) {
		payload[i] = rec[TX_RECORD_HDR + i];
	}
	return n;
}

void comm_bridge_rx_command(const uint8_t *data, uint16_t len)
{
	INT n = (INT)len;

	if (mbf_rx <= 0) {
		return;
	}
	/* Over-long commands are cut to COMM_CMD_MAX + 1 bytes: still one
	 * byte too long, so the parser rejects them as "too long". */
	if (n > (COMM_CMD_MAX + 1)) {
		n = COMM_CMD_MAX + 1;
	}
	if (n == 0) {
		n = 1;		/* an empty message still gets an ack */
		data = (const uint8_t *)" ";
	}
	if (tk_snd_mbf(mbf_rx, data, n, TMO_POL) == E_OK) {
		tk_set_flg(flg_comm, FLG_COMM_RX);
	} else {
		bridge_rx_dropped++;
	}
}

void comm_bridge_mqtt_state(int32_t state)
{
	s1_mqtt_state = (INT)state;
	if (flg_comm > 0) {
		tk_set_flg(flg_comm, FLG_COMM_LINK);
	}
}
