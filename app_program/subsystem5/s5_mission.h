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
#endif
