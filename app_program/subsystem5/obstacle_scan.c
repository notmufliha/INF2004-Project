#include "subsystem5.h"
#include "obstacle_scan.h"
#include "s5_logic.h"
#include "drv_ultrasonic.h"
#include "s5_measure.h"
#include "drv_servo.h"
#include "../robot_config.h"

/* Calibrate against the actual servo and sensor mounting. Servo must settle
 * before measuring; avoid ultrasonic cross-talk by spacing trigger pulses. */
#ifndef S5_SERVO_SETTLE_MS
#define S5_SERVO_SETTLE_MS ROBOT_SCAN_SERVO_SETTLE_MS
#endif
#ifndef S5_ECHO_GAP_MS
#define S5_ECHO_GAP_MS 65
#endif

static const uint16_t coarse_angles[S5_SCAN_COUNT] = {70u, 80u, 90u, 100u, 110u};

int s5_find_nearest(const S5ScanPoint *points, int count)
{
    int best = -1;
    int i;
    if (!points || count <= 0)
    {
        return -1;
    }
    for (i = 0; i < count; ++i) {
        if (points[i].valid && points[i].distance_cm > 0.0f &&
            points[i].distance_cm <= S5_MAX_RANGE_CM &&
            (best < 0 || points[i].distance_cm < points[best].distance_cm))
            best = i;
    }
    return best;
}
static int scan_angles(S5ScanPoint *points, const uint16_t *angles, int count, bool probe)
{
    int i;
    (void)probe;
    for (i = 0; i < count; ++i)
        points[i] = (S5ScanPoint){angles[i], 0.0f, false};
    for (i = 0; i < count; ++i) {
#if S5_SERVO_ENABLED && S5_HARDWARE_ENABLED && !S5_SIMULATION
#if ROBOT_VERBOSE_SENSOR_LOG
        tm_printf((UB*)"[S5-SCAN] move servo to %d deg; settling %d ms\n",
                  (int)angles[i], (int)S5_SERVO_SETTLE_MS);
#endif
        if (!s5_servo_angle(angles[i])) {
            int k;
            for (k = 0; k < count; ++k) points[k].valid = false;
            (void)s5_servo_angle(90u);
            return -1;
        }
        tk_dly_tsk(S5_SERVO_SETTLE_MS);
        points[i].valid = probe ? s5_measure_probe_cm(&points[i].distance_cm) :
                                  s5_measure_scan_cm(&points[i].distance_cm);
        /* A failed three-ping measurement is unknown. Do not repeatedly
         * trigger the transducer or invent a distance during a scan. */
#if ROBOT_VERBOSE_SENSOR_LOG
        tm_printf((UB*)"[S5-SCAN] angle=%d echo=%s distance_cm_x100=%d\n",
                  (int)angles[i], points[i].valid ? "OK" : "TIMEOUT/INVALID",
                  (int)(points[i].distance_cm * 100.0f));
#endif
        /* Servo travel/settling already spaces rays. The driver also
         * enforces the 65 ms minimum at every actual trigger. */
#endif
    }
#if S5_SERVO_ENABLED && S5_HARDWARE_ENABLED && !S5_SIMULATION
    if (!s5_servo_angle(90u)) {
        for (i = 0; i < count; ++i) points[i].valid = false;
    }
    tk_dly_tsk(S5_SERVO_SETTLE_MS);
#endif
    return s5_find_nearest(points, count);
}
static void scan_five(S5Scan *scan, const uint16_t *angles, bool probe)
{
    if (scan) scan->nearest_index = scan_angles(scan->points, angles, S5_SCAN_COUNT, probe);
}
void s5_coarse_scan(S5Scan *scan) { scan_five(scan, coarse_angles, false); }
void s5_width_scan(S5Scan *scan, S5ScanPoint points[S5_WIDTH_SCAN_COUNT])
{
    static const uint16_t angles[S5_WIDTH_SCAN_COUNT] = {
        ROBOT_WIDTH_SWEEP_MIN_DEG, ROBOT_WIDTH_SCAN_MIN_DEG, 60u, 75u,
        90u, 105u, ROBOT_WIDTH_SWEEP_MAX_DEG
    };
    static const int route_rays[S5_SCAN_COUNT] = {1, 2, 4, 5, 6};
    (void)scan_angles(points, angles, S5_WIDTH_SCAN_COUNT, true);
    /* Use the left turn bearing and the rightmost available ray. The
     * live sensor checks the actual right turn heading before driving. */
    for (int i = 0; i < S5_SCAN_COUNT; ++i)
        scan->points[i] = points[route_rays[i]];
    scan->nearest_index = s5_find_nearest(scan->points, S5_SCAN_COUNT);
}
void s5_probe_scan(S5Scan *scan) { scan_five(scan, coarse_angles, true); }
void s5_fine_scan(S5Scan *scan, uint16_t center_deg)
{
    uint16_t angles[S5_SCAN_COUNT];
    s5_make_fine_angles(center_deg, angles);
    scan_five(scan, angles, false);
}
