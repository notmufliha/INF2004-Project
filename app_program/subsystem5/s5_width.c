/**
 * @file s5_width.c
 * @brief Obstacle width from a fine servo sweep (B5). See s5_width.h.
 *
 * Algorithm adapted, with permission, from the AAI03 team's INF2004
 * obs_profile.c ((c) 2026 AAI03 team): find_extent(), beam_corrected()
 * and face_reach_mm(), rewritten for this project's units and bearings.
 */
#include "s5_width.h"

#include <math.h>
#include <stddef.h>

#define DEG_TO_RAD 0.01745329252f

static bool on_obstacle(const S5ScanPoint *p, float closest_cm, float depth_cm,
                        float range_limit_cm)
{
    return p->valid && p->distance_cm > 0.0f &&
           p->distance_cm <= range_limit_cm &&
           p->distance_cm <= closest_cm + depth_cm;
}

/* Furthest sideways an echo at range r can lie when the obstacle is no
 * nearer than face: sqrt(r^2 - face^2), 0 if r is shorter. */
static float face_reach_cm(float range_cm, float face_cm)
{
    float face = face_cm > 0.0f ? face_cm : 0.0f;
    return range_cm > face ? sqrtf(range_cm * range_cm - face * face) : 0.0f;
}

S5WidthResult s5_width_profile(const S5ScanPoint *points, int count,
                               float beam_half_deg, float depth_cm,
                               float range_limit_cm, float max_width_cm)
{
    S5WidthResult r = {0};
    int closest = -1;
    int i;

    r.first = r.last = -1;
    if (points == NULL || count <= 0) return r;
    for (i = 0; i < count; ++i) {
        if (points[i].valid && points[i].distance_cm > 0.0f &&
            points[i].distance_cm <= range_limit_cm &&
            (closest < 0 || points[i].distance_cm < points[closest].distance_cm))
            closest = i;
    }
    if (closest < 0) return r;

    {
        const float near_cm = points[closest].distance_cm;
        const float centre_deg = (float)points[closest].angle_deg;
        int low = closest, high = closest;

        while (low > 0 && on_obstacle(&points[low - 1], near_cm, depth_cm, range_limit_cm))
            --low;
        while (high + 1 < count && on_obstacle(&points[high + 1], near_cm, depth_cm, range_limit_cm))
            ++high;

        r.valid = true;
        r.first = low;
        r.last = high;
        r.nearest_cm = near_cm;
        r.face_cm = near_cm * cosf((centre_deg - 90.0f) * DEG_TO_RAD);
        r.bracketed = low > 0 && high + 1 < count;
        r.left_x_cm = r.right_x_cm = near_cm * sinf((centre_deg - 90.0f) * DEG_TO_RAD);

        for (i = low; i <= high; ++i) {
            float angle = (float)points[i].angle_deg;
            float shift = centre_deg - angle;
            float x, reach;
            /* The cone still hears the edge after the servo has turned past
             * it: move the point toward the closest one, never past it. */
            if (shift > beam_half_deg) shift = beam_half_deg;
            if (shift < -beam_half_deg) shift = -beam_half_deg;
            angle += shift;
            x = points[i].distance_cm * sinf((angle - 90.0f) * DEG_TO_RAD);
            reach = face_reach_cm(points[i].distance_cm, r.face_cm);
            if (x > reach) x = reach;
            if (x < -reach) x = -reach;
            if (x < r.left_x_cm) r.left_x_cm = x;
            if (x > r.right_x_cm) r.right_x_cm = x;
        }
    }
    r.width_cm = r.right_x_cm - r.left_x_cm;
    if (max_width_cm > 0.0f && r.width_cm > max_width_cm) {
        /* Wider than the course allows is sonar spread: keep the centre. */
        float centre = (r.left_x_cm + r.right_x_cm) * 0.5f;
        r.width_cm = max_width_cm;
        r.left_x_cm = centre - max_width_cm * 0.5f;
        r.right_x_cm = centre + max_width_cm * 0.5f;
        r.capped = true;
    }
    r.center_x_cm = (r.left_x_cm + r.right_x_cm) * 0.5f;
    return r;
}
