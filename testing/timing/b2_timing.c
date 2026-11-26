/* b2_timing.c - B2 hot paths, compiled against the tree before
 * (7bad6fb, -DB2_BEFORE) and after (dc2cc5a) the minimum-bit change.
 *  b2_isr_4edges : encoder GPIO ISR for one full forward quadrature cycle
 *                  on the left wheel (4 edges)
 *  b2_cmd_store_load : write a DRIVE command into the pending slot and read
 *                  it back, as motion_send_command / motion_receive_command */
#include "subsystem2/encoder_control.c"
#include "subsystem2/motion_command.h"
#ifndef B2_BEFORE
#include "subsystem2/motion_storage.h"
#endif

#define INTS0 (*(volatile UW *)(ENC_IO_BASE + 0x120U))
#define SIO_IN (*(volatile UW *)0xD0000004U)

void b2_isr_4edges(void)
{
	static const UW levels[4] = { 2U, 3U, 1U, 0U }; /* A,B: 10 11 01 00 */
	for (int i = 0; i < 4; i++) {
		INTS0 = ENC_EDGE_MASK(ENC_L_A);
		SIO_IN = levels[i];
		encoder_gpio_isr(0);
	}
}

static motion_command_t cmd_in = {
	.type = MOTION_CMD_DRIVE, .direction = MOTION_DIR_FORWARD,
	.target_speed_cps = 1200, .steering_permille = -350,
	.timeout_ms = 500U, .command_id = 7U, .safety_generation = 3U };
static motion_command_t cmd_out;
#ifdef B2_BEFORE
static motion_command_t slot;
#else
static motion_stored_command_t slot;
#endif
volatile int result;

void b2_cmd_store_load(void)
{
#ifdef B2_BEFORE
	slot = cmd_in;
	cmd_out = slot;
#else
	slot = motion_pack_command(&cmd_in);
	slot.command_id = cmd_in.command_id;
	cmd_out.type = slot.type;
	cmd_out.direction = slot.direction;
	cmd_out.turn_direction = slot.turn_direction;
	cmd_out.target_speed_cps = slot.target_speed_cps;
	cmd_out.target_angle_deg = slot.target_angle_deg;
	cmd_out.steering_permille = slot.steering_permille;
	cmd_out.timeout_ms = slot.timeout_ms;
	cmd_out.run_ms = slot.run_ms;
	cmd_out.target_distance_counts = slot.target_distance_counts;
	cmd_out.command_id = slot.command_id;
	cmd_out.safety_generation = slot.safety_generation;
#endif
	result = cmd_out.steering_permille;
}
