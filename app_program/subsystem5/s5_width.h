/**
 * @file s5_width.h
 * @brief Obstacle width from a fine servo sweep (B5).
 *
 * Adapted, with permission, from the AAI03 team's INF2004 obstacle profile
 * (obs_profile.c, (c) 2026 AAI03 team): the obstacle is the run of scan
 * points around the closest echo whose range stays within a depth
 * tolerance of it; each point is moved toward the closest point by up to
 * the sonar's half-beam (the cone keeps hearing a corner after the servo
 * has turned past it); and each edge is bounded by sqrt(r^2 - face^2),
 * the furthest sideways an echo at range r can come from when the face
 * is no nearer than its closest point.
 *
 * Changes for this robot: centimetres and float (B5's units), this
 * project's bearings (90 = ahead, below 90 = left, above = right), right
 * positive lateral offsets, and a bracketed flag (both edges seen inside
 * the sweep) for the log and the MQTT width report.
 */
#ifndef S5_WIDTH_H
#define S5_WIDTH_H

#include <stdbool.h>
#include "obstacle_scan.h"

typedef struct {
    bool  valid;        /* an obstacle within range_limit_cm was seen     */
    bool  bracketed;    /* free space was seen beyond both of its edges   */
    bool  capped;       /* width was limited to max_width_cm              */
    float width_cm;     /* right edge - left edge                         */
    float center_x_cm;  /* lateral centre, right positive                 */
    float left_x_cm;    /* left edge (negative = left of the sensor axis) */
    float right_x_cm;
    float face_cm;      /* forward distance to the closest point          */
    float nearest_cm;   /* range of the closest echo                      */
    int   first, last;  /* run of points on the obstacle (indices)        */
} S5WidthResult;

/* points: sorted by increasing angle. beam_half_deg: sonar half-beam
 * correction (0 = none). depth_cm: an echo this much further than the
 * closest one still belongs to the obstacle. range_limit_cm: ignore echoes
 * beyond this. max_width_cm: cap (0 = none). */
S5WidthResult s5_width_profile(const S5ScanPoint *points, int count,
                               float beam_half_deg, float depth_cm,
                               float range_limit_cm, float max_width_cm);

#endif
