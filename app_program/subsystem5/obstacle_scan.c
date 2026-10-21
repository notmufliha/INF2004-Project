#include "subsystem5.h"
#include "obstacle_scan.h"
#include "s5_logic.h"
#include "drv_ultrasonic.h"
#include "s5_measure.h"
#include "drv_servo.h"

/* Calibrate against the actual servo and sensor mounting. Servo must settle
 * before measuring; avoid ultrasonic cross-talk by spacing trigger pulses. */
#ifndef S5_SERVO_SETTLE_MS
#define S5_SERVO_SETTLE_MS 180
#endif
#ifndef S5_ECHO_GAP_MS
#define S5_ECHO_GAP_MS 65
#endif

static const uint16_t coarse_angles[S5_SCAN_COUNT] = {30u, 60u, 90u, 120u, 150u};

int s5_find_nearest(const S5ScanPoint *points, int count)
{
    int best = -1;
    int i;
    if (!points || count <= 0) return -1;
    for (i = 0; i < count; ++i) {
        if (points[i].valid && points[i].distance_cm > 0.0f &&
            points[i].distance_cm <= S5_MAX_RANGE_CM &&
            (best < 0 || points[i].distance_cm < points[best].distance_cm))
            best = i;
    }
    return best;
}
static void scan_angles(S5Scan *scan, const uint16_t angles[S5_SCAN_COUNT])
{
    int i;
    if (!scan) return;
    for (i = 0; i < S5_SCAN_COUNT; ++i) {
        scan->points[i].angle_deg = angles[i];
        scan->points[i].distance_cm = 0.0f;
        scan->points[i].valid = false;
#if S5_SERVO_ENABLED && S5_HARDWARE_ENABLED && !S5_SIMULATION
        tm_printf((UB*)"[S5-SCAN] move servo to %d deg; settling %d ms\n",
                  (int)angles[i], (int)S5_SERVO_SETTLE_MS);
        s5_servo_angle(angles[i]);
        tk_dly_tsk(S5_SERVO_SETTLE_MS);
        scan->points[i].valid = s5_measure_stable_cm(&scan->points[i].distance_cm);
        tm_printf((UB*)"[S5-SCAN] angle=%d echo=%s distance_cm_x100=%d\n",
                  (int)angles[i], scan->points[i].valid ? "OK" : "TIMEOUT/INVALID",
                  (int)(scan->points[i].distance_cm * 100.0f));
        tk_dly_tsk(S5_ECHO_GAP_MS);
#endif
    }
#if S5_SERVO_ENABLED && S5_HARDWARE_ENABLED && !S5_SIMULATION
    s5_servo_angle(90u);
    tk_dly_tsk(S5_SERVO_SETTLE_MS);
#endif
    scan->nearest_index = s5_find_nearest(scan->points, S5_SCAN_COUNT);
}
void s5_coarse_scan(S5Scan *scan) { scan_angles(scan, coarse_angles); }
void s5_fine_scan(S5Scan *scan, uint16_t center_deg)
{
    uint16_t angles[S5_SCAN_COUNT];
    s5_make_fine_angles(center_deg, angles);
    scan_angles(scan, angles);
}
