/** @file imu_lift.h
 * @brief Buddy 4 lift detection: the car picked up off the floor.
 *
 * Uses the size of the measured acceleration, which does not depend on how
 * the car is held: on the floor (still, driving or crossing a hump) it stays
 * near 1 g; lifting by hand first accelerates the car upward.  The excess
 * over a slowly learned 1 g baseline is integrated (with leaks) into an
 * upward speed and a rise.  Both above their thresholds is a lift.  The
 * flag then stays set for hold_ms so the Vehicle Controller sees it and
 * stops the motors (it needs START to run again).
 *
 * The learned baseline absorbs accelerometer scale error and slow drift,
 * so a constant offset never integrates into a false rise.
 */
#ifndef IMU_LIFT_H
#define IMU_LIFT_H

#include <stdbool.h>
#include <stdint.h>

#include "imu.h"

/* Samples (10 ms each) averaged into the 1 g baseline before detection. */
#define IMU_LIFT_SETTLE_SAMPLES 50U
/* The recent upward speed peak fades this fast (m/s per s): a 0.3 m/s
 * pick-up peak still counts ~0.3 s later, when the rise has built up. */
#define IMU_LIFT_PEAK_FADE_MPS2 1.0f
/* Largest difference one sample may teach the 1 g baseline (m/s^2). */
#define IMU_LIFT_LEARN_CLIP_MS2 0.5f

/* Provisional thresholds: validate by lifting the real car. */
typedef struct
{
    float32_t rise_speed_mps; /* upward speed that counts as lifting      */
    float32_t rise_m;         /* upward rise that counts as lifting       */
    float32_t speed_leak_s;   /* speed integrator time constant           */
    float32_t rise_leak_s;    /* rise integrator time constant            */
    float32_t baseline_s;     /* 1 g baseline learning time constant      */
    uint32_t  hold_ms;        /* report lifted this long after detection  */
    uint32_t  confirm_ms;     /* both thresholds held this long, no break */
    uint32_t  max_gap_ms;     /* larger sample gaps restart integration   */
} imu_lift_config_t;

typedef struct
{
    imu_lift_config_t config;
    bool              b_have_sample;
    uint32_t          settle_count; /* samples averaged into baseline    */
    uint32_t          last_ms;
    float32_t         baseline_ms2;     /* learned |a| on the floor          */
    float32_t         speed_mps;        /* leaky upward speed                */
    float32_t         rise_m;           /* leaky upward rise                 */
    float32_t         recent_speed_mps; /* fading peak of speed_mps      */
    bool              b_rising;         /* both thresholds met now           */
    uint32_t          rising_ms;        /* ...since this sample              */
    bool              b_lifted;
    bool              b_was_lifted;   /* b_lifted before the last step     */
    uint32_t          lifted_ms;      /* when the lift was detected        */
    float32_t         peak_speed_mps; /* at detection, for the log       */
    float32_t         peak_rise_m;
} imu_lift_t;

void imu_lift_default_config (imu_lift_config_t * p_config);
bool imu_lift_init (imu_lift_t * p_lift, const imu_lift_config_t * p_config);
/* One accelerometer sample (robot axes, m/s^2, gravity included).  Returns
 * true while the car counts as lifted.  Invalid samples restart the
 * integration (the lifted hold still runs out on time). */
bool imu_lift_step (imu_lift_t * p_lift, const imu_data_t * p_data);
/* True on the sample that detected a lift (for a single log line). */
bool imu_lift_started (const imu_lift_t * p_lift, const imu_data_t * p_data);
/* True when the last imu_lift_step changed the lifted flag either way
 * (e.g. to update telemetry only on a change). */
bool imu_lift_changed (const imu_lift_t * p_lift);

#endif

/*** end of file ***/

