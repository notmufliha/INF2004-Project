#ifndef S5_MISSION_H
#define S5_MISSION_H
#include <stdbool.h>
typedef enum { S5_MISSION_FOLLOW, S5_MISSION_BUSY, S5_MISSION_FAULT } S5MissionState;
void s5_mission_init(void);
void s5_mission_enable(bool enabled);
bool s5_mission_enabled(void);
void s5_mission_publish(S5MissionState state);
S5MissionState s5_mission_state(void);
void s5_mission_front(bool valid, float cm);
bool s5_mission_approach_slow(void);
/* VC: the car is on (or just past) a ramp: shorter obstacle stop. */
void s5_mission_set_ramp(bool on_ramp);
bool s5_mission_ramp(void);
/* VC -> B5: the armed barcode turn for the next crossing (-1 left,
 * +1 right, 0 none). B5 bypasses on that side when it is open. */
void s5_mission_set_turn_hint(int hint);
int s5_mission_turn_hint(void);
/* B5 -> VC: the rejoin already made that turn at the crossing next to the
 * obstacle; the VC drops its pending action (read once). */
void s5_mission_mark_turn_done(void);
bool s5_mission_take_turn_done(void);
#endif
