/*
 *	demo_sim.c
 *	Subsystem 1 -- STANDALONE DEMO ONLY (S1_STANDALONE_DEMO == 1).
 *
 *	A fake "robot" so the whole WiFi/MQTT path can be tested and
 *	demonstrated before the other subsystems exist:
 *
 *	  - plays the role of B2/B3/B4/B5: updates the telemetry snapshot
 *	    every 100 ms with plausible, changing values, and reports a
 *	    barcode, hump and obstacle event every few seconds;
 *	  - plays the role of the Vehicle Controller: takes commands from
 *	    mbf_remote_cmd and reacts (START/STOP/PAUSE/SPEED...), so you can
 *	    see a command sent from the laptop change the telemetry.
 *
 *	It uses ONLY the public interface in subsystem1.h, exactly as the
 *	real subsystems will, so it doubles as a usage example for them.
 *
 *	Set S1_STANDALONE_DEMO to 0 in subsystem1.h before integration,
 *	otherwise this task would steal the Vehicle Controller's commands.
 */

#include "s1_private.h"

#if S1_STANDALONE_DEMO

#define DEMO_STEP_MS		100
#define DEMO_FULL_SPEED		200	/* mm/s at SPEED 100 */
#define DEMO_COUNTS_PER_MM	2	/* fake encoder resolution */
#define DEMO_BARCODE_MS		8000
#define DEMO_HUMP_MS		13000
#define DEMO_OBSTACLE_MS	21000

/* Small pseudo-random generator so the values look alive */
LOCAL UW demo_rand(void)
{
	static UW x = 2463534242U;

	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

EXPORT void demo_task(INT stacd, void *exinf)
{
	REMOTE_CMD	cmd;
	BOOL		running   = TRUE;
	ROBOT_STATE	state     = ROBOT_LINE_FOLLOW;
	W		speed_pct = 50;
	W		dist_mm   = 0;
	H		peak_mm   = 0;
	UW		t         = 0U;
	UB		barcode   = 'A';

	(void)stacd; (void)exinf;

	tm_printf((UB*)"s1-demo: fake robot running.  Set S1_STANDALONE_DEMO "
		  "to 0 before integration!\n");

	while (1) {
		H spd;

		tk_dly_tsk(DEMO_STEP_MS);
		t += DEMO_STEP_MS;

		/* ---- act as the Vehicle Controller ---------------------- */
		while (tk_rcv_mbf(mbf_remote_cmd, &cmd, TMO_POL) > 0) {
			tm_printf((UB*)"s1-demo: controller got %s arg=%d seq=%u\n",
				  remote_cmd_name((REMOTE_CMD_TYPE)cmd.type),
				  (INT)cmd.arg, cmd.seq);
			switch (cmd.type) {
			case RCMD_START:
			case RCMD_RESUME:
				running = TRUE;
				state   = ROBOT_LINE_FOLLOW;
				break;
			case RCMD_STOP:
			case RCMD_ESTOP:
				running = FALSE;
				state   = ROBOT_SAFE_STOP;
				break;
			case RCMD_PAUSE:
				running = FALSE;
				state   = ROBOT_IDLE;
				break;
			case RCMD_SPEED:
				speed_pct = cmd.arg;
				break;
			case RCMD_RESET_PEAK:
				peak_mm = 0;
				break;
			default:
				break;
			}
		}

		/* ---- act as B2 (motion) --------------------------------- */
		spd = running ? (H)((speed_pct * DEMO_FULL_SPEED) / 100) : 0;
		dist_mm += ((W)spd * DEMO_STEP_MS) / 1000;
		comm_set_motion(spd, (H)(spd - (H)(demo_rand() % 4U)),
				dist_mm * DEMO_COUNTS_PER_MM,
				dist_mm * DEMO_COUNTS_PER_MM - 3,
				dist_mm);
		comm_set_robot_state(state);

		/* ---- act as B3 (line sensors) --------------------------- */
		comm_set_line((UH)(800U + (demo_rand() % 200U)),
			      (UH)(2300U + (demo_rand() % 400U)),
			      0U, LINE_F_DETECTED);

		/* ---- act as B4 (IMU) ------------------------------------ */
		comm_set_imu(running ? IMU_EV_MOVING : IMU_EV_STATIONARY,
			     (H)((W)(demo_rand() % 21U) - 10), peak_mm);

		if (!running) {
			continue;	/* no new events while stopped */
		}

		/* ---- periodic events ------------------------------------ */
		if ((t % DEMO_BARCODE_MS) == 0U) {
			NAV_CMD nav = (NAV_CMD)(NAV_LEFT + (barcode - 'A'));
			(void)comm_report_barcode(barcode, nav);
			barcode = (barcode >= 'D') ? (UB)'A' : (UB)(barcode + 1);
		}
		if ((t % DEMO_HUMP_MS) == 0U) {
			H h = (H)(10U + (demo_rand() % 30U));
			if (h > peak_mm) {
				peak_mm = h;
			}
			(void)comm_report_hump(h, peak_mm);
		}
		if ((t % DEMO_OBSTACLE_MS) == 0U) {
			(void)comm_report_obstacle(250, 120, 300, 90, AVOID_LEFT);
		}
	}
}

#endif /* S1_STANDALONE_DEMO */
