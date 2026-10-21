#ifndef S5_OBSTACLE_SCAN_H
#define S5_OBSTACLE_SCAN_H
#include <stdbool.h>
#include <stdint.h>
#define S5_SCAN_COUNT 5

typedef struct {
    uint16_t angle_deg;
    float distance_cm;
    bool valid;
} S5ScanPoint;

typedef struct {
    S5ScanPoint points[S5_SCAN_COUNT];
    int nearest_index; /* -1 when all readings invalid */
} S5Scan;

void s5_coarse_scan(S5Scan *scan);
void s5_fine_scan(S5Scan *scan, uint16_t center_deg);
int s5_find_nearest(const S5ScanPoint *points, int count);
#endif
