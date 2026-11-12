#ifndef S5_NAVIGATION_H
#define S5_NAVIGATION_H
#include <stdbool.h>
#include "s5_logic.h"

/* B5 owns bypass/recovery commands while the mission handoff is BUSY.
 * VC owns normal DRIVE; B3 alone owns ADC/line sampling. */
bool s5_navigation_init(void);
void s5_navigation_pause(void);
void s5_navigation_tick(bool front_valid, float front_cm, bool follow);
S5Error s5_navigation_error(void);
bool s5_navigation_stopped(void);
bool s5_navigation_has_line_reference(void);
/* Encoder-controlled stationary viewpoint changes, servo remains bounded. */
bool s5_navigation_probe_begin(int turn_deg);
bool s5_navigation_probe_complete(void);
bool s5_navigation_probe_bypass(const S5Profile *profile);
/* Optional non-blocking rear-sensor provider, installed before task startup.
 * True means fresh independently verified clearance for the requested travel.
 * No provider means that reverse is rejected, never assumed safe. */
bool s5_navigation_set_rear_sensor(bool (*rear_clear)(float travel_cm));
#endif
