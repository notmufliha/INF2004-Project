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
 *	copy.  Nothing here ever waits for the network (NFR-B1-2).
 *
 *	PACKED SHARED DATA: s1_telem is a set of C bit-fields.  Writing one
 *	bit-field reads, modifies and writes back the whole 32-bit word that
 *	also holds other tasks' fields, so every write AND every read is done
 *	while holding mtx_telem.  Values are range-checked (saturated) before
 *	the lock is taken, so the lock is held only for the stores.
 *
 *	APIs shown: tk_snd_mbf/tk_rcv_mbf with TMO_POL, tk_loc_mtx with a
 *	            timeout, tk_set_flg, tk_get_otm
 */

#include "s1_private.h"

EXPORT volatile UB	s1_mqtt_state = COMM_MQTT_IDLE;
EXPORT volatile UB	s1_comm_state = COMM_INITIALISING;
EXPORT COMM_STATS	s1_stats;
EXPORT COMM_TELEMETRY	s1_telem;

/* Written only by the WiFi task, read by anyone */
LOCAL volatile UH	bridge_rx_dropped;

/* Milliseconds since boot (wraps after ~49 days, fine for a demo run) */
EXPORT UW s1_now_ms(void)
{
	SYSTIM	tim;

	tk_get_otm(&tim);	/* operating time: never set back */
	return tim.lo;
}

/* Lock the snapshot.  FALSE = could not get it in time, skip the update.
 * The holder only copies 24 bytes, and TA_INHERIT raises a low-priority
 * holder, so in practice this never times out. */
LOCAL BOOL telem_lock(void)
{
	return (tk_loc_mtx(mtx_telem, TELEM_LOCK_TMO) == E_OK) ? TRUE : FALSE;
}

LOCAL void telem_unlock(void)
{
	tk_unl_mtx(mtx_telem);
}

/* cm value for a 9-bit / 7-bit field; all-ones = not measured */
LOCAL UW dist_field(UH cm)
{
	return (cm == COMM_CM_INVALID) ? 0x1FFU : sat_u(cm, COMM_DIST_MAX_CM);
}

LOCAL UW width_field(UH cm)
{
	return (cm == COMM_CM_INVALID) ? 0x7FU : sat_u(cm, COMM_WIDTH_MAX_CM);
}

/* =====================================================================
 *  1a. Telemetry snapshot setters (IF-S1-TLM)
 * ===================================================================== */

EXPORT void comm_set_robot_state(ROBOT_STATE state)
{
	if ((UINT)state >= (UINT)ROBOT_STATE_COUNT) {
		return;
	}
	if (telem_lock()) {
		s1_telem.robot_state = (UINT)state;
		telem_unlock();
	}
}

/* Speeds in encoder counts/s (B2's unit).  Encoder totals keep only the
 * low 24 bits: at B2's 5000 counts/s maximum that is 28 min of driving
 * before wrapping, and the laptop decoder unwraps it anyway. */
EXPORT void comm_set_motion(W speed_l_cps, W speed_r_cps, W enc_l, W enc_r)
{
	INT	sl = (INT)sat_s(speed_l_cps, COMM_SPEED_MAX);
	INT	sr = (INT)sat_s(speed_r_cps, COMM_SPEED_MAX);
	UINT	el = (UINT)enc_l & 0xFFFFFFU;
	UINT	er = (UINT)enc_r & 0xFFFFFFU;

	if (telem_lock()) {
		s1_telem.speed_l = sl;
		s1_telem.speed_r = sr;
		s1_telem.enc_l   = el;
		s1_telem.enc_r   = er;
		telem_unlock();
	}
}

EXPORT void comm_set_line(UH ir_left, UH ir_centre, UB ir_right, UB flags)
{
	UINT l = sat_u(ir_left, COMM_IR_MAX);
	UINT c = sat_u(ir_centre, COMM_IR_MAX);

	if (telem_lock()) {
		s1_telem.ir_left    = l;
		s1_telem.ir_centre  = c;
		s1_telem.ir_right   = (ir_right != 0U) ? 1U : 0U;
		s1_telem.line_flags = flags & 0x07U;
		telem_unlock();
	}
}

EXPORT void comm_set_imu(UB motion_state, H pitch_ddeg)
{
	INT p = (INT)sat_s(pitch_ddeg, COMM_PITCH_MAX);

	if (motion_state >= COMM_MOTION_COUNT) {
		return;
	}
	if (telem_lock()) {
		s1_telem.motion     = motion_state;
		s1_telem.pitch_ddeg = p;
		telem_unlock();
	}
}

EXPORT void comm_set_obstacle_dist(UH dist_cm)
{
	UINT d = dist_field(dist_cm);

	if (telem_lock()) {
		s1_telem.obs_dist_cm = d;
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

	ev->time_ds = (s1_now_ms() / 100U) & 0xFFFFFU;
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

EXPORT ER comm_report_barcode(UB letter, NAV_CMD nav)
{
	COMM_EVENT	ev = {0};
	UINT		code;

	if ((letter < COMM_BARCODE_FIRST) || (letter > COMM_BARCODE_LAST) ||
	    ((UINT)nav >= (UINT)NAV_COUNT)) {
		return E_PAR;
	}
	code = (UINT)(letter - COMM_BARCODE_FIRST) + 1U;

	ev.type = EVT_BARCODE;
	ev.a    = code;
	ev.e    = (UINT)nav;

	if (telem_lock()) {			/* also kept in telemetry */
		s1_telem.barcode = code;
		s1_telem.nav     = (UINT)nav;
		telem_unlock();
	}
	return post_event(&ev);
}

/* Heights in mm (0.1 cm resolution), 0..126.  valid = FALSE while B4
 * has no height estimator: the laptop then shows "n/a", not 0 mm. */
EXPORT ER comm_report_hump(BOOL valid, H height_mm, H peak_mm)
{
	COMM_EVENT	ev = {0};
	UINT		h = valid ? sat_u(height_mm, COMM_HUMP_MAX_MM) : 0U;
	UINT		p = valid ? sat_u(peak_mm, COMM_HUMP_MAX_MM) : 0U;

	ev.type = EVT_HUMP;
	ev.flag = valid ? 1U : 0U;
	ev.a    = h;
	ev.b    = p;

	if (telem_lock()) {
		s1_telem.hump_valid   = ev.flag;
		s1_telem.hump_last_mm = h;
		if (p > s1_telem.hump_peak_mm) {
			s1_telem.hump_peak_mm = p;
		}
		telem_unlock();
	}
	return post_event(&ev);
}

EXPORT ER comm_report_obstacle(UH dist_cm, UH width_cm, UH left_cm,
			       UH right_cm, AVOID_ACTION action)
{
	COMM_EVENT ev = {0};

	if ((UINT)action >= (UINT)AVOID_COUNT) {
		return E_PAR;
	}
	ev.type = EVT_OBSTACLE;
	ev.a    = dist_field(dist_cm);
	ev.b    = width_field(width_cm);
	ev.c    = dist_field(left_cm);
	ev.d    = dist_field(right_cm);
	ev.e    = (UINT)action;

	if (telem_lock()) {
		s1_telem.obs_dist_cm  = ev.a;
		s1_telem.obs_width_cm = ev.b;
		s1_telem.obs_left_cm  = ev.c;
		s1_telem.obs_right_cm = ev.d;
		telem_unlock();
	}
	return post_event(&ev);
}

EXPORT ER comm_report_error(UB err_code, UB detail)
{
	COMM_EVENT ev = {0};

	ev.type = EVT_ERROR;
	ev.a    = err_code;
	ev.b    = detail;
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
 *  1d. Shared fault bitmask (fault_flags.h).  flg_faults is created in
 *  subsystem1.c.  The kernel updates the pattern atomically, so these
 *  are safe from any task or interrupt handler.
 * ===================================================================== */

EXPORT void fault_raise(UINT bits)
{
	if (flg_faults > 0) {
		tk_set_flg(flg_faults, bits & FAULT_ALL);
	}
}

EXPORT void fault_clear(UINT bits)
{
	if (flg_faults > 0) {
		tk_clr_flg(flg_faults, ~(bits & FAULT_ALL));	/* AND mask */
	}
}

EXPORT UB fault_get(void)
{
	T_RFLG rflg;

	if ((flg_faults <= 0) || (tk_ref_flg(flg_faults, &rflg) != E_OK)) {
		return 0U;
	}
	return (UB)(rflg.flgptn & FAULT_ALL);
}

/* =====================================================================
 *  1e. Outgoing queue, filled by comm_task only
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

	rec[0] = (UB)(((UINT)topic & 0x03U) | (retain ? 0x04U : 0U));
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
	*topic_id = rec[0] & 0x03U;
	*retain   = (rec[0] >> 2) & 0x01U;
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
	/* Over-long frames are cut to COMM_CMD_MAX + 1 bytes: still one
	 * byte too long, so the decoder rejects them as the wrong length. */
	if (n > (COMM_CMD_MAX + 1)) {
		n = COMM_CMD_MAX + 1;
	}
	if (n == 0) {
		n = 1;		/* an empty message still gets an ack */
		data = (const uint8_t *)"";
	}
	if (tk_snd_mbf(mbf_rx, data, n, TMO_POL) == E_OK) {
		tk_set_flg(flg_comm, FLG_COMM_RX);
	} else {
		bridge_rx_dropped++;
	}
}

void comm_bridge_mqtt_state(int32_t state)
{
	s1_mqtt_state = (UB)state;
	if (flg_comm > 0) {
		tk_set_flg(flg_comm, FLG_COMM_LINK);
	}
}
