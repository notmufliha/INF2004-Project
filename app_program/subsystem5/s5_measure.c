#include "subsystem5.h"
#include "s5_measure.h"
#include "drv_ultrasonic.h"

bool s5_measure_stable_cm(float *out_cm)
{
    float values[3], t;
    int n = 0, i, j;
    if (!out_cm) return false;
    for (i = 0; i < 3; ++i) {
        float v = 0.0f;
        if (s5_ultrasonic_read_cm(&v) && v >= 2.0f && v <= 400.0f)
            values[n++] = v;
        if (i != 2) tk_dly_tsk(S5_ECHO_GAP_MS);
    }
    if (n < 2) return false;
    for (i = 0; i < n; ++i)
        for (j = i + 1; j < n; ++j)
            if (values[j] < values[i]) {
                t = values[i]; values[i] = values[j]; values[j] = t;
            }
    *out_cm = n == 3 ? values[1] : (values[0] + values[1]) * 0.5f;
    return true;
}
