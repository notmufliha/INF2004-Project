/* Host test: s5_width_profile() (fine-sweep obstacle width). A simulated
 * sonar returns the nearest point of a flat box face inside its beam cone,
 * which is what makes a plain angle-based width read too wide. */
#include "../../app_program/subsystem5/s5_width.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define DEG 0.01745329252f

/* Box face from x=left..right (cm, right positive) at forward y=face.
 * Ray at bearing a (90 = ahead) with beam half-angle b. */
static S5ScanPoint ping(uint16_t a, float left, float right, float face, float b)
{
    S5ScanPoint p = {a, 0.0f, false};
    float lo = 90.0f + atanf(left / face) / DEG;
    float hi = 90.0f + atanf(right / face) / DEG;
    float c_lo = (float)a - b, c_hi = (float)a + b;
    if (c_hi < lo || c_lo > hi) return p;          /* cone misses the box */
    /* Nearest face point in the cone: closest to straight-on (x = 0). */
    float best = c_lo > 90.0f ? c_lo : (c_hi < 90.0f ? c_hi : 90.0f);
    if (best < lo) best = lo;
    if (best > hi) best = hi;
    p.distance_cm = face / cosf((best - 90.0f) * DEG);
    p.valid = true;
    return p;
}

static S5WidthResult sweep(float left, float right, float face, float beam,
                           float correction)
{
    S5ScanPoint pts[13];
    for (int i = 0; i < 13; ++i)
        pts[i] = ping((uint16_t)(60 + 5 * i), left, right, face, beam);
    return s5_width_profile(pts, 13, correction, 8.0f, 60.0f, 20.0f);
}

int main(void)
{
    /* The course box: 14 cm wide, centred, sonar 15 cm from its face. */
    /* At 15 cm a 14 cm box spans +/-25 deg; with the beam spread the rays
     * at 60 and 120 deg (the servo's limit) still hear it, so the run is
     * not bracketed. The range bound still places each edge at a corner,
     * so a width is reported (the earlier midpoint method gave none). */
    for (float beam = 5.0f; beam <= 15.0f; beam += 2.5f) {
        S5WidthResult r = sweep(-7.0f, 7.0f, 15.0f, beam, 0.0f);
        printf("   beam %4.1f deg: width %.1f cm%s\n", beam, r.width_cm,
               r.bracketed ? "" : " (edges at the sweep limit)");
        assert(r.valid);
        assert(fabsf(r.width_cm - 14.0f) <= 3.5f);
        assert(fabsf(r.center_x_cm) <= 1.5f);
    }
    /* Beam correction only narrows the reading. */
    assert(sweep(-7.0f, 7.0f, 15.0f, 7.5f, 10.0f).width_cm <=
           sweep(-7.0f, 7.0f, 15.0f, 7.5f, 0.0f).width_cm);
    /* Off-centre box: the centre moves toward it. Its right edge (12 cm
     * at 15 cm = 39 deg) is past the servo's 120 deg limit, so that side
     * reads short: a known limit of the sweep, not of the method. */
    {
        S5WidthResult r = sweep(-2.0f, 12.0f, 15.0f, 7.5f, 0.0f);
        printf("   off-centre (right edge past the sweep): width %.1f centre %.1f\n",
               r.width_cm, r.center_x_cm);
        assert(r.valid && r.center_x_cm > 1.0f);
    }
    /* Nothing in range: not valid. */
    {
        S5ScanPoint pts[3] = {{80u, 90.0f, true}, {90u, 0.0f, false}, {100u, 75.0f, true}};
        assert(!s5_width_profile(pts, 3, 10.0f, 8.0f, 60.0f, 20.0f).valid);
    }
    /* A wall far behind the box is not part of it (depth tolerance). */
    {
        S5ScanPoint pts[5] = {{70u, 40.0f, true}, {80u, 16.0f, true}, {90u, 15.0f, true},
                              {100u, 16.0f, true}, {110u, 40.0f, true}};
        S5WidthResult r = s5_width_profile(pts, 5, 0.0f, 8.0f, 60.0f, 0.0f);
        assert(r.valid && r.first == 1 && r.last == 3 && r.bracketed);
    }
    /* Cap: a reading wider than the course allows keeps its centre. */
    {
        S5ScanPoint pts[3] = {{60u, 30.0f, true}, {90u, 10.0f, true}, {120u, 30.0f, true}};
        S5WidthResult r = s5_width_profile(pts, 3, 0.0f, 25.0f, 60.0f, 20.0f);
        assert(r.capped && fabsf(r.width_cm - 20.0f) < 0.01f);
        assert(fabsf(r.center_x_cm) < 0.01f);
    }
    puts("PASS B5 fine-sweep width: beam correction, face bound, off-centre, depth run, cap");
    return 0;
}
