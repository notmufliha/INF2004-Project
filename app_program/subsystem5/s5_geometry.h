#ifndef S5_GEOMETRY_H
#define S5_GEOMETRY_H
#include <math.h>
#include "../robot_config.h"

/* Conservative axle-relative radial clearance: project the measured sensor
 * offset along the ray. This is <= the true axle-to-hit distance. Bearings
 * remain robot-relative: 90 is front. Never fabricate a missing echo. */
static inline float
s5_turn_range_cm (float sensor_cm, unsigned bearing_deg)
{
    float projection = cosf(((float)bearing_deg - 90.0f) * 0.01745329252f);
    if (projection < 0.0f)
    {
        projection = 0.0f;
    }
    return sensor_cm + ROBOT_SENSOR_TURN_OFFSET_CM * projection;
}
#endif

/*** end of file ***/

