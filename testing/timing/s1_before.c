/* s1_before.c - B1 telemetry build BEFORE Change Requirement 1 (JSON text).
 * Compiled against a checkout of 3e34f57; the body is publish_telemetry()
 * from that commit's task_comm.c without the final send.  Typical values
 * match MQTT.md's example telemetry. */
#include "subsystem1/s1_private.h"

static UB json_out[401];
static COMM_TELEMETRY t = {
	2U, 1U, 0U, 'A', 300, 296, 41230, 40987, 0,
	812U, 3010U, 1U, 3U, -12, 23, 31, 250, 120, 300, 90 };
volatile INT result;

void s1_before_telemetry(void)
{
	JSON_BUF jb;
	UB bc[2] = { t.last_barcode, '\0' };

	json_begin(&jb, json_out, (INT)sizeof(json_out));
	json_uint(&jb, "seq", 42U);
	json_uint(&jb, "t", 210000U);
	json_str(&jb, "state", robot_state_name((ROBOT_STATE)t.robot_state));
	json_pair(&jb, "spd", t.speed_l, t.speed_r);
	json_pair(&jb, "enc", t.enc_l, t.enc_r);
	json_int(&jb, "dist", t.distance_mm);
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
	json_int(&jb, "pitch", t.pitch_ddeg);
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
	result = json_end(&jb);
}
