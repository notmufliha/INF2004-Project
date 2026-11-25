#ifndef S5_MANEUVER_H
#define S5_MANEUVER_H
#include "s5_logic.h"

/* Portable B5 executor. The VC/B2 adapter owns command IDs and PWM. */
typedef enum {
    S5_MAN_IDLE, S5_MAN_TURN_OUT, S5_MAN_OFFSET, S5_MAN_FORWARD,
    S5_MAN_PASS, S5_MAN_TURN_IN, S5_MAN_SEARCH, S5_MAN_LINE_STOP,
    S5_MAN_ALIGN, S5_MAN_COMPLETE, S5_MAN_FAILED, S5_MAN_BACK,
    S5_MAN_HANDOFF_STOP, S5_MAN_ALIGN_CHECK, S5_MAN_REPLAN_STOP, S5_MAN_REPLAN_TURN,
    S5_MAN_REPLAN_CHECK, S5_MAN_REPLAN_ADVANCE,
    S5_MAN_REJOIN_ADVANCE, S5_MAN_REJOIN_ALIGN
} S5ManeuverState;
typedef enum { S5_ACT_NONE, S5_ACT_STOP, S5_ACT_MOVE,
               S5_ACT_TURN } S5ActionType;
typedef struct {
    S5ActionType type;
    float distance_cm; /* negative means reverse */
    int turn_deg;      /* negative means left */
} S5Action;
typedef struct {
    uint32_t now_ms;
    bool front_valid;
    float front_cm;
    bool stopped;
    bool motion_terminal;
    bool motion_success;
    bool motion_cancelled;
    uint32_t result_id;
    bool line_valid;
    bool line_detected;
    bool line_junction;
    uint32_t line_sample_ms;
    bool rear_clear; /* independent rear observation, never a front echo */
    float motion_progress_cm;
} S5ManeuverInput;
typedef struct {
    S5ManeuverState state;
    S5Error error;
    S5Side side;
    float lateral_cm;
    float offset_cm;
    float minimum_offset_cm;
    unsigned turn_clear_hits;
    uint32_t last_turn_clear_ms;
    float pass_cm;
    float offset_done_cm, pass_done_cm, active_move_cm;
    float searched_cm;
    float search_limit_cm;
    uint32_t start_ms;
    uint32_t leg_ms;
    uint32_t pending_id;
    uint32_t last_line_ms;
    unsigned line_hits;
    unsigned alignment_step;
    int alignment_heading_deg;
    bool left_original_line;
    bool outward_turn_complete;
    bool equal_leg_route;
    bool guarded_probe_route;
    bool issuing;
    bool waiting_clear;
    unsigned clear_hits;
    uint32_t last_clear_sample_ms;
    bool reactive;
    unsigned reactive_turns;
    uint32_t invalid_since_ms;
    int reactive_heading_deg, reactive_target_deg;
    float reactive_progress_cm;
    unsigned reactive_exit_hits;
    float reactive_required_cm, reactive_forward_cm;
    int reactive_entry_heading_deg;
    bool forward_paused;
    bool reactive_align_parallel, reactive_parallel_lock;
} S5Maneuver;

void s5_maneuver_init(S5Maneuver *m);
/* depth_bound_cm is supplied by the validated course/VC, not measured width.
 * A reverse request requires independent rear clearance on every tick. */
bool s5_maneuver_begin(S5Maneuver *m, S5Command command,
                       const S5Profile *profile, float depth_bound_cm,
                       uint32_t now_ms);
bool s5_maneuver_plan_equal_legs(S5Maneuver *m, float obstacle_front_cm);
bool s5_maneuver_plan_probe_route(S5Maneuver *m, float obstacle_front_cm);
bool s5_maneuver_course_room(const S5Profile *profile, S5Side side, float obstacle_front_cm);
S5Action s5_maneuver_step(S5Maneuver *m, const S5ManeuverInput *in);
/* A rejected submission is a terminal STOP, not an assumed completion. */
bool s5_maneuver_submitted(S5Maneuver *m, uint32_t command_id);
void s5_maneuver_abort(S5Maneuver *m, S5Error error);
#endif
