/** @file s1_led.c
 * @brief Subsystem 1 link-status LED on GP19 (design element D1.4).
 *
 * The WiFi/MQTT state can be read at a glance, without a laptop:
 *
 *     CONNECTING_WIFI          fast blink   (2 Hz)
 *     CONNECTING_MQTT          slow blink   (1 Hz)
 *     RECONNECTING             slow blink   (1 Hz)
 *     CONNECTED                solid on
 *     OFFLINE / DISABLED       short blip every 2 s
 *     INITIALISING             off
 *
 * The task sleeps until a cyclic handler wakes it every S1_TICK_MS, so
 * the pattern timing stays exact.
 *
 * APIs shown: tk_slp_tsk, tk_wup_tsk, tk_can_wup, cyclic handler,
 *             gpio_set_pin / gpio_set_val
 */

#include "s1_led.h"
#include "s1_private.h"

#define BLIP_PERIOD_TICKS 8U /* 8 x 250 ms = 2 s */

/** @brief LED task: one pattern step per cyclic wake-up, forever. */
void
s1_led_task (INT stacd, void * p_exinf)
{
    uint32_t tick = 0U;

    (void)stacd;
    (void)p_exinf;

    (void)gpio_set_pin(S1_LED_PIN, GPIO_MODE_OUT);

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR); /* until s1_led_cyclic_isr()     */
        (void)tk_can_wup(TSK_SELF); /* drop wake-ups that piled up    */

        /* g_s1_link_state only ever holds comm_link_state_t values. */
        (void)gpio_set_val(S1_LED_PIN,
                           s1_led_level((comm_link_state_t)g_s1_link_state,
                                        tick));
        tick++;
    }
}

/** @brief LED level for @p state at @p tick (250 ms steps). */
uint32_t
s1_led_level (comm_link_state_t state, uint32_t tick)
{
    uint32_t level = 0U;

    switch (state)
    {
        case COMM_CONNECTED:
            level = 1U;
        break;

        case COMM_CONNECTING_WIFI:
            level = tick & 1U; /* toggle every tick */
        break;

        case COMM_CONNECTING_MQTT:
        case COMM_RECONNECTING:
            level = (tick >> 1U) & 1U; /* toggle every 2 ticks */
        break;

        case COMM_OFFLINE:
        case COMM_DISABLED:
            level = (0U == (tick % BLIP_PERIOD_TICKS)) ? 1U : 0U;
        break;

        default:
            level = 0U;
        break;
    }

    return (level);
}

/** @brief Cyclic handler: interrupt context, so short and never waits.
 * It only wakes the LED task. */
void
s1_led_cyclic_isr (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(tid_led);
}

/*** end of file ***/

