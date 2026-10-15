/*
 *	task_led.c
 *	Subsystem 1 -- link-status LED on GP19.
 *
 *	You can see the WiFi/MQTT state at a glance, without a laptop:
 *
 *	    CONNECTING_WIFI          fast blink   (2 Hz)
 *	    CONNECTING_MQTT          slow blink   (1 Hz)
 *	    RECONNECTING             slow blink   (1 Hz)
 *	    CONNECTED                solid on
 *	    OFFLINE / DISABLED       short blip every 2 s
 *	    INITIALISING             off
 *
 *	The task sleeps until a cyclic handler wakes it every S1_TICK_MS, so
 *	the pattern timing stays exact (same technique as the template).
 *
 *	APIs shown: tk_slp_tsk, tk_wup_tsk, tk_can_wup, cyclic handler,
 *	            gpio_set_pin / gpio_set_val
 */

#include "s1_private.h"

#define BLIP_PERIOD_TICKS	8	/* 8 x 250 ms = 2 s */

/*
 * Cyclic handler: runs in interrupt context, NOT as a task, so it must
 * be short and must never wait.  It only wakes the LED task.
 */
EXPORT void led_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_led);
}

LOCAL UINT led_level(COMM_STATE st, UW tick)
{
	switch (st) {
	case COMM_CONNECTED:
		return 1U;
	case COMM_CONNECTING_WIFI:
		return tick & 1U;			/* toggle every tick */
	case COMM_CONNECTING_MQTT:
	case COMM_RECONNECTING:
		return (tick >> 1) & 1U;		/* toggle every 2 ticks */
	case COMM_OFFLINE:
	case COMM_DISABLED:
		return ((tick % BLIP_PERIOD_TICKS) == 0U) ? 1U : 0U;
	default:
		return 0U;
	}
}

EXPORT void led_task(INT stacd, void *exinf)
{
	UW tick = 0U;

	(void)stacd; (void)exinf;

	gpio_set_pin(S1_LED_PIN, GPIO_MODE_OUT);

	while (1) {
		tk_slp_tsk(TMO_FEVR);		/* until led_cyclic_handler */
		tk_can_wup(TSK_SELF);		/* drop wake-ups that piled up */

		gpio_set_val(S1_LED_PIN,
			     led_level((COMM_STATE)s1_comm_state, tick));
		tick++;
	}
}
