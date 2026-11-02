/** @file comm.c
 * @brief Subsystem 1 public, non-blocking API (design element D1.1):
 * telemetry setters, one-off events and link status (see comm.h).
 *
 * Every kernel call here either polls (TMO_POL) or waits at most
 * TELEM_LOCK_TMO ms for a mutex that is only ever held for a short
 * copy.  Nothing here ever waits for the network (NFR-B1-2).
 *
 * PACKED SHARED DATA: g_s1_telem is a set of C bit-fields.  Writing one
 * bit-field reads, modifies and writes back the whole 32-bit word that
 * also holds other tasks' fields, so every write AND every read is done
 * while holding gh_s1_telem_mtx.  Values are range-checked (saturated)
 * before the lock is taken, so the lock is held only for the stores.
 */

#include "comm.h"
#include "s1_private.h"
#include "s1_comm.h"

#define TIME_DS_MASK   0xFFFFFU /* 20-bit event time stamp, 0.1 s */
#define MS_PER_DS      100U
#define DIST_INVALID   0x1FFU   /* 9-bit field all ones           */
#define WIDTH_INVALID  0x7FU    /* 7-bit field all ones           */
#define BARCODE_MASK   0x07U
#define NAV_MASK       0x07U
#define ACTION_MASK    0x07U
#define ROBOT_MASK     0x0FU
#define MOTION_MASK    0x07U
#define HUMP_MASK      0x7FU
#define DIST_MASK      0x1FFU
#define WIDTH_MASK     0x7FU
#define IR_MASK        0xFFFU
#define SPEED_MASK     0x3FFFU  /* 14 bits */
#define PITCH_MASK     0x3FFU   /* 10 bits */
#define ERR_MASK       0xFFU

comm_telemetry_t     g_s1_telem;
comm_stats_t         g_s1_stats;
volatile uint8_t     g_s1_link_state = (uint8_t)COMM_INITIALISING;

static bool     telem_lock (void);
static void     telem_unlock (void);
static uint32_t dist_field (uint16_t dist_cm);
static uint32_t width_field (uint16_t width_cm);
static ER       post_event (comm_pack_event_t * p_event);

/** @brief Set the robot state shown in telemetry (VC). */
void
comm_set_robot_state (comm_robot_state_t state)
{
    if (((uint32_t)state < (uint32_t)ROBOT_STATE_COUNT) && (telem_lock()))
    {
        g_s1_telem.robot_state = (UINT)state & ROBOT_MASK;
        telem_unlock();
    }
}

/** @brief Wheel speeds (counts/s, B2's unit) and encoder totals (B2).
 *
 * Encoder totals keep only the low 24 bits: at B2's 5000 counts/s
 * maximum that is 28 min of driving before wrapping, and the laptop
 * decoder unwraps it anyway.
 */
void
comm_set_motion (int32_t speed_l_cps, int32_t speed_r_cps, int32_t enc_l,
                 int32_t enc_r)
{
    uint32_t left  = comm_pack_twos(comm_pack_sat_s(speed_l_cps,
                                                    COMM_SPEED_MAX),
                                    COMM_SPEED_BITS);
    uint32_t right = comm_pack_twos(comm_pack_sat_s(speed_r_cps,
                                                    COMM_SPEED_MAX),
                                    COMM_SPEED_BITS);
    /* Totals wrap by design: modulo 2^24 via two's complement. */
    uint32_t total_l = (uint32_t)enc_l & COMM_ENC_MASK;
    uint32_t total_r = (uint32_t)enc_r & COMM_ENC_MASK;

    if (telem_lock())
    {
        g_s1_telem.speed_left  = left & SPEED_MASK;
        g_s1_telem.speed_right = right & SPEED_MASK;
        g_s1_telem.enc_left    = total_l & COMM_ENC_MASK;
        g_s1_telem.enc_right   = total_r & COMM_ENC_MASK;
        telem_unlock();
    }
}

/** @brief Line sensors: raw ADC 0..4095, right sensor digital (B3). */
void
comm_set_line (uint16_t ir_left, uint16_t ir_centre, uint8_t ir_right,
               uint8_t flags)
{
    uint32_t left   = comm_pack_sat_u(ir_left, COMM_IR_MAX);
    uint32_t centre = comm_pack_sat_u(ir_centre, COMM_IR_MAX);

    if (telem_lock())
    {
        g_s1_telem.ir_left    = left & IR_MASK;
        g_s1_telem.ir_centre  = centre & IR_MASK;
        g_s1_telem.ir_right   = (0U != ir_right) ? 1U : 0U;
        g_s1_telem.line_flags = (UINT)flags & LINE_F_ALL;
        telem_unlock();
    }
}

/** @brief IMU motion state (0..7) and pitch in 0.1 degree (B4). */
void
comm_set_imu (uint8_t motion_state, int16_t pitch_ddeg)
{
    uint32_t pitch = comm_pack_twos(comm_pack_sat_s(pitch_ddeg,
                                                    COMM_PITCH_MAX),
                                    COMM_PITCH_BITS);

    if ((motion_state < COMM_MOTION_COUNT) && (telem_lock()))
    {
        g_s1_telem.motion     = (UINT)motion_state & MOTION_MASK;
        g_s1_telem.pitch_ddeg = pitch & PITCH_MASK;
        telem_unlock();
    }
}

/** @brief Front ultrasonic distance in cm, or COMM_CM_INVALID (B5). */
void
comm_set_obstacle_dist (uint16_t dist_cm)
{
    uint32_t dist = dist_field(dist_cm);

    if (telem_lock())
    {
        g_s1_telem.obs_dist_cm = dist & DIST_MASK;
        telem_unlock();
    }
}

/** @brief Consistent copy of the whole snapshot (under the mutex). */
void
comm_get_telemetry (comm_telemetry_t * p_out)
{
    if ((NULL != p_out) && (telem_lock()))
    {
        *p_out = g_s1_telem;
        telem_unlock();
    }
}

/** @brief Report a decoded barcode 'A'..'D' and its command (B3). */
ER
comm_report_barcode (uint8_t letter, comm_nav_cmd_t nav)
{
    comm_pack_event_t event  = {0};
    ER                result = E_PAR;
    uint32_t          code;

    if ((letter >= (uint8_t)COMM_BARCODE_FIRST) &&
        (letter <= (uint8_t)COMM_BARCODE_LAST) &&
        ((uint32_t)nav < (uint32_t)NAV_COUNT))
    {
        code        = ((uint32_t)letter - (uint32_t)COMM_BARCODE_FIRST) + 1U;
        event.type  = EVT_BARCODE;
        event.val_a = code & BARCODE_MASK;
        event.val_e = (UINT)nav & NAV_MASK;

        if (telem_lock()) /* also kept in the telemetry snapshot */
        {
            g_s1_telem.barcode = code & BARCODE_MASK;
            g_s1_telem.nav     = (UINT)nav & NAV_MASK;
            telem_unlock();
        }

        result = post_event(&event);
    }

    return (result);
}

/** @brief Report a hump: heights in mm, 0..126 (B4).
 *
 * b_valid = false while B4 has no height estimate: the laptop then
 * shows "n/a", not 0 mm.  The telemetry peak only ever rises until
 * comm_reset_hump_peak().
 */
ER
comm_report_hump (bool b_valid, int16_t height_mm, int16_t peak_mm)
{
    comm_pack_event_t event  = {0};
    uint32_t          height = 0U;
    uint32_t          peak   = 0U;

    if (b_valid)
    {
        height = comm_pack_sat_u(height_mm, COMM_HUMP_MAX_MM);
        peak   = comm_pack_sat_u(peak_mm, COMM_HUMP_MAX_MM);
    }

    event.type  = EVT_HUMP;
    event.flag  = b_valid ? 1U : 0U;
    event.val_a = height & HUMP_MASK;
    event.val_b = peak & HUMP_MASK;

    if (telem_lock())
    {
        g_s1_telem.hump_valid   = event.flag;
        g_s1_telem.hump_last_mm = height & HUMP_MASK;
        if (peak > g_s1_telem.hump_peak_mm)
        {
            g_s1_telem.hump_peak_mm = peak & HUMP_MASK;
        }
        telem_unlock();
    }

    return (post_event(&event));
}

/** @brief Forget the highest hump (RESET_PEAK, via the VC). */
void
comm_reset_hump_peak (void)
{
    if (telem_lock())
    {
        g_s1_telem.hump_peak_mm = 0U;
        telem_unlock();
    }
}

/** @brief Report an obstacle and the avoidance decision, in cm (B5). */
ER
comm_report_obstacle (uint16_t dist_cm, uint16_t width_cm, uint16_t left_cm,
                      uint16_t right_cm, comm_avoid_action_t action)
{
    comm_pack_event_t event  = {0};
    ER                result = E_PAR;

    if ((uint32_t)action < (uint32_t)AVOID_COUNT)
    {
        event.type  = EVT_OBSTACLE;
        event.val_a = dist_field(dist_cm) & DIST_MASK;
        event.val_b = width_field(width_cm) & WIDTH_MASK;
        event.val_c = dist_field(left_cm) & DIST_MASK;
        event.val_d = dist_field(right_cm) & DIST_MASK;
        event.val_e = (UINT)action & ACTION_MASK;

        if (telem_lock())
        {
            g_s1_telem.obs_dist_cm  = event.val_a;
            g_s1_telem.obs_width_cm = event.val_b & WIDTH_MASK;
            g_s1_telem.obs_left_cm  = event.val_c;
            g_s1_telem.obs_right_cm = event.val_d;
            telem_unlock();
        }

        result = post_event(&event);
    }

    return (result);
}

/** @brief Report a 1-byte error code (error_codes.h) and a detail. */
ER
comm_report_error (uint8_t err_code, uint8_t detail)
{
    comm_pack_event_t event = {0};

    event.type  = EVT_ERROR;
    event.val_a = (UINT)err_code & ERR_MASK;
    event.val_b = (UINT)detail & ERR_MASK;

    return (post_event(&event));
}

/** @brief Current link state (comm_link_state_t). */
comm_link_state_t
comm_get_state (void)
{
    /* Only ever written with comm_link_state_t values (s1_comm.c). */
    return ((comm_link_state_t)g_s1_link_state);
}

/** @brief True while WiFi and MQTT are both up. */
bool
comm_is_connected (void)
{
    return ((uint8_t)COMM_CONNECTED == g_s1_link_state);
}

/** @brief Copy of the link counters. */
void
comm_get_stats (comm_stats_t * p_out)
{
    if (NULL != p_out)
    {
        if (telem_lock())
        {
            *p_out = g_s1_stats;
            telem_unlock();
        }
        p_out->rx_dropped = comm_bridge_rx_dropped();
    }
}

/** @brief Lock the snapshot; false = not in time, skip the update.
 *
 * The holder only copies 24 bytes and TA_INHERIT raises a low-priority
 * holder, so in practice this never times out.
 */
static bool
telem_lock (void)
{
    return (E_OK == tk_loc_mtx(gh_s1_telem_mtx, TELEM_LOCK_TMO));
}

/** @brief Release the snapshot mutex. */
static void
telem_unlock (void)
{
    (void)tk_unl_mtx(gh_s1_telem_mtx);
}

/** @brief cm value for a 9-bit field; all ones = not measured. */
static uint32_t
dist_field (uint16_t dist_cm)
{
    uint32_t field = DIST_INVALID;

    if (COMM_CM_INVALID != dist_cm)
    {
        field = comm_pack_sat_u(dist_cm, COMM_DIST_MAX_CM) & DIST_MASK;
    }

    return (field);
}

/** @brief cm value for a 7-bit field; all ones = not measured. */
static uint32_t
width_field (uint16_t width_cm)
{
    uint32_t field = WIDTH_INVALID;

    if (COMM_CM_INVALID != width_cm)
    {
        field = comm_pack_sat_u(width_cm, COMM_WIDTH_MAX_CM) & WIDTH_MASK;
    }

    return (field);
}

/** @brief Time-stamp an event and queue it for the comm task. */
static ER
post_event (comm_pack_event_t * p_event)
{
    ER result;

    p_event->time_ds = (s1_comm_now_ms() / MS_PER_DS) & TIME_DS_MASK;
    /* sizeof(comm_pack_event_t) is 8: fits the kernel's INT. */
    result = tk_snd_mbf(gh_s1_event_mbf, p_event, (INT)sizeof(*p_event),
                        TMO_POL);

    if (E_OK == result)
    {
        (void)tk_set_flg(gh_s1_comm_flg, FLG_COMM_EVENT); /* wake task */
    }
    else if (telem_lock()) /* several writers: count under the lock */
    {
        g_s1_stats.evt_dropped++;
        telem_unlock();
    }
    else
    {
        /* Could not even count it; the drop is still reported E_TMOUT. */
    }

    return (result);
}

/*** end of file ***/

