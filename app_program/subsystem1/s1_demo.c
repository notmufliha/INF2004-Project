/** @file s1_demo.c
 * @brief Subsystem 1 STANDALONE DEMO ONLY (S1_STANDALONE_DEMO == 1).
 *
 * A fake "robot" so the whole WiFi/MQTT path can be shown on the bench
 * without the other subsystems:
 *   - plays B2/B3/B4/B5: updates the telemetry snapshot every 100 ms
 *     with plausible, changing values and reports a barcode, hump and
 *     obstacle event every few seconds;
 *   - plays the Vehicle Controller: takes commands from mbf_remote_cmd
 *     and reacts (START/STOP/PAUSE/SPEED...), so a command sent from
 *     the laptop visibly changes the telemetry.
 *
 * It uses ONLY the public interface in subsystem1.h, exactly as the
 * real subsystems do.  Off by default: the Vehicle Controller refuses to
 * build with it, because this task would steal its commands.
 */

#include "s1_demo.h"
#include "s1_private.h"
#include "comm_names.h"

#if S1_STANDALONE_DEMO

#define DEMO_STEP_MS      100U
#define DEMO_FULL_SPEED   5000 /* counts/s at SPEED 100 (B2's max) */
#define DEMO_PERCENT      100
#define DEMO_BARCODE_MS   8000U
#define DEMO_HUMP_MS      13000U
#define DEMO_OBSTACLE_MS  21000U
#define DEMO_SEED         2463534242UL

/** @brief Fake controller and sensor state. */
typedef struct
{
    bool               b_running;
    comm_robot_state_t state;
    int32_t            speed_pct;
    int32_t            enc;
    int16_t            peak_mm;
    uint32_t           time_ms;
    uint8_t            barcode;
} demo_state_t;

static uint32_t demo_rand (void);
static void     demo_commands (demo_state_t * p_demo);
static void     demo_sensors (demo_state_t * p_demo);
static void     demo_events (demo_state_t * p_demo);
#endif /* S1_STANDALONE_DEMO */

/** @brief Start the demo task (nothing to do unless it is built in). */
ER
s1_demo_start (void)
{
    ER result = E_OK;
#if S1_STANDALONE_DEMO
    ID task   = app_start_task((FP)s1_demo_task, S1_PRI_DEMO, 0U,
                               "demo_task");

    result = (task < E_OK) ? task : E_OK;
#endif

    return (result);
}

/** @brief The fake robot: one step every DEMO_STEP_MS, forever. */
void
s1_demo_task (INT stacd, void * p_exinf)
{
#if S1_STANDALONE_DEMO
    demo_state_t demo = {0};
#endif

    (void)stacd;
    (void)p_exinf;

#if S1_STANDALONE_DEMO
    demo.b_running = true;
    demo.state     = ROBOT_LINE_FOLLOW;
    demo.speed_pct = 50;
    demo.barcode   = (uint8_t)'A';

    tm_printf((UB *)"s1-demo: fake robot running.  Build without "
                    "S1_STANDALONE_DEMO=1 to use the real subsystems!\n");

    for (;;)
    {
        (void)tk_dly_tsk(DEMO_STEP_MS);
        demo.time_ms += DEMO_STEP_MS;
        demo_commands(&demo);
        demo_sensors(&demo);
        if (demo.b_running)
        {
            demo_events(&demo); /* no new events while stopped */
        }
    }
#endif
}

#if S1_STANDALONE_DEMO

/** @brief xorshift32: values that look alive, no division. */
static uint32_t
demo_rand (void)
{
    static uint32_t state = DEMO_SEED;

    state ^= state << 13U;
    state ^= state >> 17U;
    state ^= state << 5U;

    return (state);
}

/** @brief Act as the Vehicle Controller for every queued command. */
static void
demo_commands (demo_state_t * p_demo)
{
    comm_remote_cmd_t cmd;
    uint32_t          count;
    bool              b_more = true;

    for (count = 0U; (count < REMOTE_CMD_QUEUE_LEN) && b_more; count++)
    {
        b_more = (tk_rcv_mbf(mbf_remote_cmd, &cmd, TMO_POL) > 0);
        if (b_more)
        {
            tm_printf((UB *)"s1-demo: controller got %s arg=%d seq=%d\n",
                      comm_names_remote((comm_remote_cmd_type_t)cmd.type),
                      (INT)cmd.arg, (INT)cmd.seq);

            switch (cmd.type)
            {
                case RCMD_START:
                case RCMD_RESUME:
                    p_demo->b_running = true;
                    p_demo->state     = ROBOT_LINE_FOLLOW;
                break;

                case RCMD_STOP:
                case RCMD_ESTOP:
                    p_demo->b_running = false;
                    p_demo->state     = ROBOT_SAFE_STOP;
                break;

                case RCMD_PAUSE:
                    p_demo->b_running = false;
                    p_demo->state     = ROBOT_IDLE;
                break;

                case RCMD_SPEED:
                    p_demo->speed_pct = (int32_t)cmd.arg;
                break;

                case RCMD_RESET_PEAK:
                    p_demo->peak_mm = 0;
                    comm_reset_hump_peak();
                break;

                default:
                break;
            }
        }
    }
}

/** @brief Act as B2/B3/B4/B5: refresh every telemetry field. */
static void
demo_sensors (demo_state_t * p_demo)
{
    int32_t speed = 0;

    if (p_demo->b_running)
    {
        speed = (p_demo->speed_pct * DEMO_FULL_SPEED) / DEMO_PERCENT;
    }

    p_demo->enc += (speed * (int32_t)DEMO_STEP_MS) / 1000;
    /* demo_rand() % 40 is 0..39: fits int32_t. */
    comm_set_motion(speed, speed - (int32_t)(demo_rand() % 40U),
                    p_demo->enc, p_demo->enc - 3);
    comm_set_robot_state(p_demo->state);

    /* Every modulus below keeps the value well inside 16 bits. */
    comm_set_line((uint16_t)(800U + (demo_rand() % 200U)),
                  (uint16_t)(2300U + (demo_rand() % 400U)), 0U,
                  LINE_F_DETECTED);
    comm_set_imu(p_demo->b_running ? 1U : 0U,
                 (int16_t)((int32_t)(demo_rand() % 21U) - 10));
    comm_set_obstacle_dist((uint16_t)(60U + (demo_rand() % 100U)));
}

/** @brief Report a barcode, hump and obstacle event now and then. */
static void
demo_events (demo_state_t * p_demo)
{
    int16_t height;

    if (0U == (p_demo->time_ms % DEMO_BARCODE_MS))
    {
        /* barcode is 'A'..'D', so NAV_LEFT + 0..3 is a valid command. */
        (void)comm_report_barcode(p_demo->barcode,
                                  (comm_nav_cmd_t)((uint32_t)NAV_LEFT +
                                                   ((uint32_t)p_demo->barcode -
                                                    (uint32_t)'A')));
        p_demo->barcode = (p_demo->barcode >= (uint8_t)'D')
                              ? (uint8_t)'A'
                              : (uint8_t)(p_demo->barcode + 1U);
    }

    if (0U == (p_demo->time_ms % DEMO_HUMP_MS))
    {
        height = (int16_t)(10U + (demo_rand() % 30U)); /* 10..39 mm */
        if (height > p_demo->peak_mm)
        {
            p_demo->peak_mm = height;
        }
        (void)comm_report_hump(true, height, p_demo->peak_mm);
    }

    if (0U == (p_demo->time_ms % DEMO_OBSTACLE_MS))
    {
        (void)comm_report_obstacle(25U, 12U, 30U, 9U, AVOID_LEFT);
    }
}

#endif /* S1_STANDALONE_DEMO */

/*** end of file ***/

