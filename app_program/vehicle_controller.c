#include "vehicle_controller.h"
#include "robot_config.h"
#include "subsystem2/b2_motor.h"
#include "subsystem3/b3_line_barcode.h"
#include "subsystem4/imu_vc.h"
#include "subsystem4/subsystem4.h"
#include "subsystem5/s5_mission.h"
#include "run_log.h"
#include "subsystem3/b3_buzzer.h"
#include "subsystem4/imu_live.h"

#if VC_STEERING_DIAGNOSTICS
static void report_steering(VEHICLE_CONTROLLER *c, const VC_INPUT *in,
                            const motion_command_t *command);
#endif

#if ROBOT_MODE && (ROBOT_MOTION_TEST || ROBOT_WHEELS_UP_MOTOR_TEST || S1_STANDALONE_DEMO)
#error "The Vehicle Controller requires exclusive command ownership; disable demos/tests"
#endif

static void set_state(VEHICLE_CONTROLLER *c, ROBOT_STATE state)
{
    /* Always publish: B5 also writes the telemetry state (obstacle legs),
     * so a state equal to our cached one may still be stale on the
     * dashboard, e.g. IDLE after PAUSE while it still shows AVOID. */
    comm_set_robot_state(state);
    if (c->state != state) {
        c->state = state;
        tm_printf((UB *)"[VC] state=%d\n", (INT)state);
    }
}

static void stop_motion(VEHICLE_CONTROLLER *c)
{
    /* Immediate PWM disable even if B2's command queue is momentarily busy. */
    if (!c->stopped) {
        motion_command_t stop = {0};
        stop.type = MOTION_CMD_STOP;
        b2_request_stop();
        if (motion_send_command(&stop) == E_OK) c->stopped = true;
    }
    c->drive_sent = false;
}

static void safe_stop(VEHICLE_CONTROLLER *c)
{
    b2_request_stop(); /* Revoke B5 movement as well as VC movement. */
    c->running = false;
    c->paused = false;
    c->line_recovery_active = false;
    c->navigation = 0u;
    c->navigation_id = 0u;
    c->pending_navigation = B3_NAV_NONE;
    c->junction_enter_samples = 0u;
    c->junction_clear_samples = 0u;
    c->junction_latched = false;
    c->rescan_phase = 0u;
    c->rescan_pending = false;
    c->approach_active = false;
#if VC_FOLLOW_V4
    c->v4_phase = 0u;
    c->v4_motion_id = 0u;
    c->bar_phase = 0u;
#endif
    stop_motion(c);
    set_state(c, ROBOT_SAFE_STOP);
}

/* Line position as in Pololu QTRSensors readLine(), for two sensors:
 * weighted average of the calibrated values (index 0 = robot's left sensor
 * = ADC "centre", index 1 = robot's right sensor = ADC "left"), ignoring
 * values <= B3_LINE_NOISE_MAX.  When neither sensor is on the line it
 * returns the extreme on the side the line was last seen (0 or 1000), so
 * the controller keeps turning back toward it.  Updates the memory. */
static INT line_position(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    uint32_t left = in->centre;  /* physically on the robot's left  */
    uint32_t right = in->left;   /* physically on the robot's right */
    uint32_t sum = 0u;
    uint32_t weighted = 0u;

    if (left > B3_LINE_ACTIVE_MAX) left = B3_LINE_ACTIVE_MAX;
    if (right > B3_LINE_ACTIVE_MAX) right = B3_LINE_ACTIVE_MAX;
    if (left >= B3_LINE_ACTIVE_MIN || right >= B3_LINE_ACTIVE_MIN) {
        if (left > B3_LINE_NOISE_MAX) sum += left;
        if (right > B3_LINE_NOISE_MAX) {
            sum += right;
            weighted += right * 1000u;
        }
        c->line_pos = (INT)(weighted / sum); /* sum > 0: one value > 200 */
        c->last_line_search_direction =
            c->line_pos < B3_LINE_POS_CENTRE ? -1 : 1;
        return c->line_pos;
    }
    return c->line_pos < B3_LINE_POS_CENTRE ? 0 : 1000;
}

/* PD steering (Pololu 3pi line follower form): error = position - centre,
 * positive = line to the right = positive B2 steering (turn right). */
static INT line_follow_steering(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    /* ADC "left" is the robot's RIGHT sensor, "centre" its LEFT one.
     * Positive difference: the left sensor is coming off the line on its
     * side, so the line lies to the right: steer right (positive). */
    INT error = (INT)in->left - (INT)in->centre;
    INT magnitude;
    INT steering = 0;
    INT full_scale = c->line_straddle ? VC_STRADDLE_FULL_SCALE :
        B3_LINE_STEER_ERROR_FULL_SCALE;

    (void)line_position(c, in); /* keep the last-seen side for recovery */
#if VC_STRADDLE_QUADRATIC
    if (c->line_straddle) {
        INT mag = error < 0 ? -error : error;
        INT target = error * mag / 1000 * B3_LINE_STEER_MAX / 1000;

        if (c->error_filter_valid) {
            INT diff = target - c->filtered_error;
            INT move = diff / 2;
            if (move == 0) move = diff;  /* integer halving never reaches 0 */
            target = c->filtered_error + move;
        }
        c->filtered_error = target;
        c->error_filter_valid = true;
        c->last_error = error;
        return target;
    }
#endif
    /* Low-pass the difference (half old, half new: ~20 ms lag at 50 Hz).
     * GP26's black/white span is only ~100 ADC counts, so a few counts of
     * noise move its normalised value by ~100 and flicked the steering
     * across the deadband on straights. */
    if (c->error_filter_valid) {
        error = (c->filtered_error + error) / 2;
    }
    c->filtered_error = error;
    c->error_filter_valid = true;
    magnitude = error < 0 ? -error : error;
    if (magnitude > B3_LINE_STEER_DEADBAND) {
        magnitude -= B3_LINE_STEER_DEADBAND;
        if (magnitude > full_scale - B3_LINE_STEER_DEADBAND) {
            magnitude = full_scale - B3_LINE_STEER_DEADBAND;
        }
        steering = magnitude * B3_LINE_STEER_MAX /
            (full_scale - B3_LINE_STEER_DEADBAND);
        if (error < 0) steering = -steering;
    }
    c->last_error = error;
    return steering;
}

/* Spread sensor layout (VC_LINE_STRADDLE): is the car following the line?
 * Both sensors on floor is the CENTRED position here, so B3's LOST flag
 * cannot be used: treating it as lost made the car run the hard line
 * search between every correction (the jerky following).  The line counts
 * as lost only after it left past a sensor and was not found again within
 * VC_STRADDLE_OUTSIDE_MS, or when no sensor touched it for
 * VC_STRADDLE_BLIND_MS; then until a sensor sees black again. */
static bool straddle_following(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    UH left = in->centre;  /* physically on the robot's left  */
    UH right = in->left;   /* physically on the robot's right */
    bool left_on = B3_LINE_CENTRE_ACTIVE(left);
    bool right_on = B3_LINE_LEFT_ACTIVE(right);

    if (left_on || right_on) {
        c->straddle_black_ms = in->now_ms;
        c->straddle_outside = 0;
        c->straddle_lost = false;
        if (left_on != right_on) {
            INT side = left_on ? -1 : 1;
            if (side != c->straddle_side) {
                c->straddle_deep = false;
                c->straddle_side_ms = in->now_ms;
            }
            c->straddle_touch = side;
            c->straddle_side = side;
            if ((left_on ? left : right) >= VC_STRADDLE_DEEP) {
                c->straddle_deep = true;
            }
        } else {
            c->straddle_side = 0;   /* crossing tape: both black */
            c->straddle_deep = false;
        }
        if (left_on && right_on) {
            if (!c->straddle_both) {
                c->straddle_both = true;
                c->straddle_both_um = c->travel_um;
            } else if (c->travel_um - c->straddle_both_um >=
                       VC_LAYOUT_CLOSE_UM) {
                /* Far longer than a crossing tape: the line is under both
                 * sensors, so they are mounted close together. */
                c->line_straddle = false;
                tm_printf((UB *)"[VC] both line sensors on black for %d mm: close-together layout, using close-layout following\n",
                          (INT)(VC_LAYOUT_CLOSE_UM / 1000u));
            }
        } else {
            c->straddle_both = false;
        }
        return true;
    }
    if (c->straddle_deep && c->straddle_side != 0) {
        c->straddle_outside = c->straddle_side;
        c->straddle_outside_ms = in->now_ms;
        c->straddle_coast_ms =
            (c->straddle_last_exit != 0 &&
             c->straddle_last_exit != c->straddle_side) ?
            VC_STRADDLE_FLIP_COAST_MS : VC_STRADDLE_COAST_MS;
        c->straddle_arc_start = VC_STRADDLE_RECOVER_START;
        if (c->straddle_last_exit == c->straddle_side &&
            in->now_ms - c->straddle_exit_ms < VC_STRADDLE_CURVE_MS) {
            c->straddle_coast_ms = 0u;            /* in a bend */
            c->straddle_arc_start = VC_STRADDLE_CURVE_ARC;
        }
        {
            INT toward = c->follow_steer * c->straddle_side;
            if (toward >= VC_STRADDLE_BEND_STEER) {
                /* Already turning hard that way and the line still left:
                 * a curve tighter than the turn.  Keep turning. */
                c->straddle_coast_ms = 0u;
                c->straddle_arc_start = toward > VC_STRADDLE_CURVE_ARC ?
                    toward : VC_STRADDLE_CURVE_ARC;
            }
        }
#if VC_BEND_HOLD
        if (c->bend_side == c->straddle_side) {
            /* Known bend that way: keep turning, no coast (SITProjects:
             * white after a bend edge is seeked, never driven straight). */
            INT hold = c->bend_peak * VC_BEND_HOLD_PERMILLE / 1000;
            if (hold < VC_BEND_SEEK_MIN) hold = VC_BEND_SEEK_MIN;
            c->straddle_coast_ms = 0u;
            if (c->straddle_arc_start < hold) c->straddle_arc_start = hold;
        }
#endif
        c->straddle_last_exit = c->straddle_side;
        c->straddle_exit_ms = in->now_ms;
    }
    c->straddle_side = 0;
    c->straddle_deep = false;
    c->straddle_both = false;
    if (c->straddle_lost) return false;
    if (c->straddle_outside != 0) {
        if (in->now_ms - c->straddle_outside_ms < VC_STRADDLE_OUTSIDE_MS) {
            return true;
        }
        c->last_line_search_direction = c->straddle_outside;
        c->straddle_outside = 0;
        c->straddle_lost = true;
        return false;
    }
    if (in->now_ms - c->straddle_black_ms >= VC_STRADDLE_BLIND_MS) {
        c->straddle_lost = true;
        return false;
    }
    return true;   /* centred between the sensors */
}

#if VC_BEND_HOLD
/* Bend hold (see VC_BEND_HOLD): returns the steering to use.  steer is the
 * touch steering line_follow_steering() computed for this tick. */
static INT straddle_bend(VEHICLE_CONTROLLER *c, const VC_INPUT *in, INT steer)
{
    bool left_on = B3_LINE_CENTRE_ACTIVE(in->centre);
    bool right_on = B3_LINE_LEFT_ACTIVE(in->left);
    INT side = (left_on == right_on) ? 0 : (left_on ? -1 : 1);

    if (left_on && right_on) {
        /* Crossing tape or junction: no evidence either way. */
        c->bend_touching = false;
        c->bend_counter = false;
        return steer;
    }
    if (side != 0) {
        INT toward = steer * side;
        if (!c->bend_touching || side != c->bend_touch_now) {
            /* A touch begins (SITProjects note_touch / note_counter). */
            if (c->bend_side == -side) {
                /* The other sensor inside a bend: usually the car a little
                 * ahead of the bend, not the bend ending.  Keep the bend
                 * and steer back gently (their LINE_COUNTER_*). */
                c->bend_counter = true;
            } else if (side != c->bend_touch_side) {
                c->bend_side = 0;          /* other side: not that bend */
            } else if (c->travel_um - c->bend_touch_um <= VC_BEND_TOUCH_UM) {
                c->bend_side = side;       /* same side again soon: bend */
            }
            if (!c->bend_counter || c->bend_side != -side) {
                c->bend_touch_side = side;
                c->bend_touch_um = c->travel_um;
                c->bend_peak = 0;
            }
            c->bend_counter_um = c->travel_um;
            c->bend_touching = true;
            c->bend_touch_now = side;
        } else if (c->bend_counter) {
            if (c->travel_um - c->bend_counter_um >= VC_BEND_COUNTER_UM) {
                /* Black too long: a real turn the other way. */
                c->bend_counter = false;
                c->bend_side = side;
                c->bend_touch_side = side;
                c->bend_touch_um = c->bend_counter_um;
                c->bend_peak = 0;
            }
        } else if (c->travel_um - c->bend_touch_um >= VC_BEND_HELD_UM) {
            c->bend_side = side;           /* one long touch: bend */
        }
        c->bend_white_um = c->travel_um;
        if (c->bend_counter) return steer / 2;
        if (toward > c->bend_peak) c->bend_peak = toward;
        if (c->bend_side == side && toward < VC_BEND_SEEK_MIN) {
            /* Still turning with the bend while the edge only grazes. */
            return side * VC_BEND_SEEK_MIN;
        }
        return steer;
    }
    /* Both white. */
    if (c->bend_touching) {
        c->bend_touching = false;
        if (c->bend_counter) {
            c->bend_counter = false;       /* short counter touch: keep bend */
            c->bend_peak = 0;              /* ...but seek back gently */
        }
        c->bend_white_um = c->travel_um;
    }
    if (c->bend_side == 0) return steer;
    {
        uint32_t white = c->travel_um - c->bend_white_um;
        INT start = c->bend_peak * VC_BEND_HOLD_PERMILLE / 1000;
        INT seek;

        if (white >= VC_BEND_LOST_UM) {
            c->bend_side = 0;              /* hand over to outside/search */
            return steer;
        }
        if (start < VC_BEND_SEEK_MIN) start = VC_BEND_SEEK_MIN;
        if (start > VC_BEND_SEEK_MAX) start = VC_BEND_SEEK_MAX;
        seek = start + (INT)((int64_t)(VC_BEND_SEEK_MAX - start) *
                             (white > VC_BEND_RAMP_UM ? VC_BEND_RAMP_UM : white) /
                             VC_BEND_RAMP_UM);
        return c->bend_side * seek;
    }
}

static void bend_reset(VEHICLE_CONTROLLER *c)
{
    c->bend_side = 0;
    c->bend_touch_side = 0;
    c->bend_touching = false;
    c->bend_counter = false;
    c->bend_peak = 0;
}
#endif

/* Spread layout: hide a line sensor that is reading barcode bars (see
 * VC_BAR_EDGES).  Modifies the controller's copy of the input. */
static void bar_filter(VEHICLE_CONTROLLER *c, VC_INPUT *in)
{
    bool left_on = B3_LINE_CENTRE_ACTIVE(in->centre);
    bool right_on = B3_LINE_LEFT_ACTIVE(in->left);

    if ((!in->barcode_active && !c->strip_active) ||
        c->strip_edges < VC_BAR_STRIP_EDGES) {
        c->bars_active = false;
        return;
    }
    if (!c->bars_active) {
        c->bars_active = true;
        c->bar_edges_l = c->bar_edges_r = 0u;
        c->bar_ignore = 0;
        c->bar_prev_l = left_on;
        c->bar_prev_r = right_on;
    }
    if (left_on && !c->bar_prev_l && c->bar_edges_l < 255u) c->bar_edges_l++;
    if (right_on && !c->bar_prev_r && c->bar_edges_r < 255u) c->bar_edges_r++;
    c->bar_prev_l = left_on;
    c->bar_prev_r = right_on;
    if (c->bar_ignore == 0) {
        INT side = 0;

        if (c->bar_edges_l >= VC_BAR_EDGES && c->bar_edges_r == 0u) side = -1;
        if (c->bar_edges_r >= VC_BAR_EDGES && c->bar_edges_l == 0u) side = 1;
        if (side != 0) {
            c->bar_ignore = side;
            /* The bar touches were not the line: forget them. */
            if (c->straddle_side == side) {
                c->straddle_side = 0;
                c->straddle_deep = false;
            }
            if (c->straddle_outside == side) c->straddle_outside = 0;
            tm_printf((UB *)"[VC] %s line sensor is reading barcode bars: ignored on this strip\n",
                      side < 0 ? "left" : "right");
        }
    }
    if (c->bar_ignore < 0) in->centre = 0u;
    if (c->bar_ignore > 0) in->left = 0u;
}

/* Barcode re-read (see VC_BARCODE_RESCANS).  Returns true while it owns
 * the motors.  Phase 1: stop.  Phase 2: reverse, then line following
 * crosses the strip again. */
static bool barcode_rescan(VEHICLE_CONTROLLER *c, const VC_INPUT *in,
                           INT speed)
{
    if (c->rescan_phase == 1u) {
        stop_motion(c);
        set_state(c, ROBOT_NAV_COMMAND);
        if (in->now_ms - c->rescan_ms > VC_RESCAN_TIMEOUT_MS) {
            c->rescan_phase = 0u;
            return false;
        }
        if (!in->motion.physically_stopped) return true;
        if (motion_move_cm((float)c->rescan_back_um / 10000.0f, speed,
                           MOTION_DIR_REVERSE, &c->rescan_id) == E_OK) {
            c->stopped = false;
            c->rescan_phase = 2u;
            c->rescan_ms = in->now_ms;
        }
        return true;
    }
    if (c->rescan_phase == 2u) {
        motion_status_snapshot_t result;
        bool done = motion_get_result(c->rescan_id, &result) == E_OK;

        set_state(c, ROBOT_NAV_COMMAND);
        if (!done && in->now_ms - c->rescan_ms <= VC_RESCAN_TIMEOUT_MS) {
            return true;
        }
        if (!done) stop_motion(c);
        tm_printf((UB *)"[VC] barcode re-read: %s, driving over it again\n",
                  done ? "backed up" : "reverse timed out");
        c->rescan_phase = 0u;
        /* Junction evidence gathered while reversing over the barcode and
         * the line is not a junction ahead (it made the car turn left
         * 50 mm after the re-read, 2026-11-18). */
        c->junction_latched = false;
        c->junction_enter_samples = c->junction_clear_samples = 0u;
        c->jct_left_seen = c->jct_right_seen = false;
        c->approach_active = false;
        c->gp28_run_um = 0u;
        c->drive_sent = false;           /* fresh, gentle start */
        c->last_steer = c->last_speed = 0;
        c->error_filter_valid = false;
        c->straddle_black_ms = in->now_ms;
        c->straddle_lost = false;
        c->straddle_outside = 0;
        c->rescan_ms = in->now_ms;       /* start of the forget window */
        return false;
    }
    return false;
}

/* Spread layout line search: rotate on the spot in VC_SPIN_STEP_DEG
 * steps, widening sweeps (see VC_SPIN_STEP_DEG). */
static void straddle_spin_search(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    if (c->spin_id != 0u) {
        motion_status_snapshot_t result;

        if (motion_get_result(c->spin_id, &result) == E_OK) {
            c->spin_id = 0u;
            c->spin_steps++;
        } else if (in->now_ms - c->spin_ms > VC_SPIN_STEP_TIMEOUT_MS) {
            stop_motion(c);
            c->spin_id = 0u;
            c->spin_steps++;
        } else {
            return;                      /* step still turning */
        }
    }
    if (c->spin_done) {
        stop_motion(c);                  /* searched +-60: wait, armed */
        return;
    }
    if (c->spin_steps >= c->spin_target) {
        /* 60 one way, 120 back (60 the other side), 60 back to the start
         * heading, then stop: never turn the car round. */
        c->spin_dir = -c->spin_dir;
        c->spin_steps = 0u;
        c->spin_sweep++;
        if (c->spin_sweep >= 3u) {
            c->spin_done = true;
            stop_motion(c);
            tm_printf((UB *)"[VC] line not found within +-60 deg: stopped (put the car on the line)\n");
            return;
        }
        c->spin_target = c->spin_sweep == 1u ? 2u * VC_SPIN_FIRST_STEPS :
            VC_SPIN_FIRST_STEPS;
    }
    if (!in->motion.physically_stopped) {
        stop_motion(c);
        return;
    }
    if (motion_turn_degrees(VC_SPIN_STEP_DEG, VC_SPIN_SPEED_CPS,
                            c->spin_dir > 0 ? MOTION_TURN_RIGHT :
                            MOTION_TURN_LEFT, &c->spin_id) == E_OK) {
        c->stopped = false;
        c->drive_sent = false;
        c->spin_ms = in->now_ms;
    }
}

/* Widening search: sweep 600 ms toward the side the line was last seen,
 * then 1200 ms the other way, 1800 ms back, and so on.  Each reversal
 * crosses the starting heading and swings further out than the last. */
static INT line_search_direction(uint32_t elapsed_ms, INT first_direction)
{
    uint32_t segment_ms = VC_LINE_SEARCH_SWITCH_MS;
    INT direction = first_direction;

    while (elapsed_ms >= segment_ms) {
        elapsed_ms -= segment_ms;
        segment_ms += VC_LINE_SEARCH_SWITCH_MS;
        direction = -direction;
    }
    return direction;
}

/* Convert the requested curve into wheel speeds the motors can hold.
 * B2 drives left = v(1+s), right = v(1-s) with s in permille.
 * allow_pivot: a hard turn may stop the inner wheel (line search only).
 * While following, a stopped inner wheel made the car pivot and lurch at
 * every hard correction, so there the inner wheel keeps the minimum. */
static void apply_wheel_minimum(motion_command_t *command, bool allow_pivot)
{
    INT steering = command->steering_permille;
    INT magnitude = steering < 0 ? -steering : steering;
    int64_t outer = (int64_t)command->target_speed_cps * (1000 + magnitude);
    int64_t inner = (int64_t)command->target_speed_cps * (1000 - magnitude);

    if (magnitude == 0) {
        /* Straight: both wheels at least the sustainable minimum. */
        if (command->target_speed_cps < VC_MIN_WHEEL_CPS) {
            command->target_speed_cps = VC_MIN_WHEEL_CPS;
        }
        return;
    }
    if (inner >= (int64_t)VC_MIN_WHEEL_CPS * 1000) return;
    if (magnitude <= 500) {
        /* Preserve gentle corrections at startup. The old limiter made
         * v=100, s=-25 into a full pivot because both wheels were below
         * their sustainable speed. Scale the base instead of the radius. */
        command->target_speed_cps =
            (VC_MIN_WHEEL_CPS * 1000 + (1000 - magnitude) - 1) /
            (1000 - magnitude);
        return;
    }
    outer /= 1000;
    if (outer < VC_MIN_WHEEL_CPS) outer = VC_MIN_WHEEL_CPS;
    if (!allow_pivot) {
        /* Never slower outside than a 500-permille turn after the scaling
         * above, so wheel speeds have no step at 500: outer >= 3x min. */
        if (outer < (int64_t)VC_MIN_WHEEL_CPS * VC_FOLLOW_MIN_OUTER_RATIO) {
            outer = (int64_t)VC_MIN_WHEEL_CPS * VC_FOLLOW_MIN_OUTER_RATIO;
        }
        inner = VC_MIN_WHEEL_CPS;
    } else {
        /* Hard turn where pivoting is allowed: an inner wheel below its
         * sustainable minimum stops, the car pivots around it.  (Raising
         * it to the minimum instead capped the turn at ~570 permille.) */
        inner = 0;
    }
    if (outer <= inner) {
        /* Both wheels would sit at the minimum and the turn would be lost
         * (or the inner wheel left in the stall zone): pivot instead. */
        inner = 0;
    }
    command->target_speed_cps = (INT)((outer + inner) / 2);
    magnitude = (INT)((outer - inner) * 1000 / (outer + inner));
    command->steering_permille = steering < 0 ? -magnitude : magnitude;
}

/* Spread layout following: wheel speeds with the inner wheel down to
 * VC_STRADDLE_MIN_INNER_CPS and an outer:inner ratio up to
 * VC_STRADDLE_MAX_RATIO (see there); pivot (inner 0) only if allowed. */
static void straddle_wheels(motion_command_t *command, bool allow_pivot)
{
    INT steering = command->steering_permille;
    INT magnitude = steering < 0 ? -steering : steering;
    int64_t base = command->target_speed_cps;
    int64_t outer = base * (1000 + magnitude) / 1000;
    int64_t inner = base * (1000 - magnitude) / 1000;

    if (magnitude == 0) {
        if (command->target_speed_cps < VC_MIN_WHEEL_CPS) {
            command->target_speed_cps = VC_MIN_WHEEL_CPS;
        }
        return;
    }
    if (allow_pivot) {
        inner = 0;
        if (outer < VC_MIN_WHEEL_CPS) outer = VC_MIN_WHEEL_CPS;
    } else {
        if (inner * VC_STRADDLE_MAX_RATIO < outer) {
            inner = outer / VC_STRADDLE_MAX_RATIO;     /* ratio cap */
        }
        if (inner < VC_STRADDLE_MIN_INNER_CPS) {
            /* Keep the requested ratio (capped) at the wheel minimum. */
            int64_t num = 1000 + magnitude;
            int64_t den = 1000 - magnitude;
            if (num > den * VC_STRADDLE_MAX_RATIO) {
                num = VC_STRADDLE_MAX_RATIO;
                den = 1;
            }
            inner = VC_STRADDLE_MIN_INNER_CPS;
            outer = inner * num / den;
        }
    }
    command->target_speed_cps = (INT)((outer + inner) / 2);
    magnitude = (INT)((outer - inner) * 1000 / (outer + inner));
    command->steering_permille = steering < 0 ? -magnitude : magnitude;
}

/* True while the barcode sensor has been on black for a crossing tape's
 * worth of travel.  Distance comes from the measured wheel speeds. */
static bool crossing_tape(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    uint32_t dt_ms = c->travel_time_valid ? in->now_ms - c->travel_last_ms : 0u;
    INT left = in->motion.left_speed_cps;
    INT right = in->motion.right_speed_cps;
    uint32_t counts_per_s;

    c->travel_last_ms = in->now_ms;
    c->travel_time_valid = true;
    if (!in->motion_valid || dt_ms > 100u) dt_ms = 0u; /* paused or stale */
    if (left < 0) left = -left;
    if (right < 0) right = -right;
    counts_per_s = (uint32_t)(left + right) / 2u;
    c->travel_um += counts_per_s * dt_ms * VC_UM_PER_COUNT / 1000u;
    if (in->right != c->gp28_level) {
        if (c->floor_known && c->gp28_level != c->floor_level &&
            c->gp28_run_um >= 2000u) {
            /* Barcode sensor back on floor: how long was it off it? */
            run_log_event(in->now_ms, RUN_EV_GP28_RUN,
                          (uint16_t)(c->gp28_run_um / 1000u));
        }
        c->gp28_level = in->right;
        c->gp28_run_um = 0u;
    } else if (c->gp28_run_um < 1000000u) {
        c->gp28_run_um += counts_per_s * dt_ms * VC_UM_PER_COUNT / 1000u;
    }
    if (c->gp28_run_um >= VC_FLOOR_LEARN_UM) {
        c->floor_level = c->gp28_level;
        c->floor_known = true;
    }
    return c->floor_known && c->gp28_level != c->floor_level &&
        c->gp28_run_um >= VC_JUNCTION_MIN_UM;
}

/* Spread layout: crossing or branch under the line sensors. */
static bool straddle_junction(VEHICLE_CONTROLLER *c, const VC_INPUT *in,
                              bool tape)
{
    bool left_recent, right_recent;
    bool left_on = B3_LINE_CENTRE_ACTIVE(in->centre);  /* robot's left */
    bool right_on = B3_LINE_LEFT_ACTIVE(in->left);     /* robot's right */

    left_recent = c->jct_left_seen &&
        c->travel_um - c->jct_left_um <= VC_STRADDLE_JCT_WINDOW_UM;
    right_recent = c->jct_right_seen &&
        c->travel_um - c->jct_right_um <= VC_STRADDLE_JCT_WINDOW_UM;
    if ((left_on && !right_recent) || (right_on && !left_recent)) {
        /* One side only (so far): a junction would start here.  Was the
         * car following steadily up to now?  Frozen once the second side
         * also touches. */
        c->jct_candidate_steady =
            c->travel_um - c->jct_block_um >= VC_JCT_SETTLE_UM;
    }
    if (left_on) {
        c->jct_left_seen = true;
        c->jct_left_um = c->travel_um;
    }
    if (right_on) {
        c->jct_right_seen = true;
        c->jct_right_um = c->travel_um;
    }
    left_recent = c->jct_left_seen &&
        c->travel_um - c->jct_left_um <= VC_STRADDLE_JCT_WINDOW_UM;
    right_recent = c->jct_right_seen &&
        c->travel_um - c->jct_right_um <= VC_STRADDLE_JCT_WINDOW_UM;
    if (in->left >= VC_STRADDLE_JCT_BOTH && in->centre >= VC_STRADDLE_JCT_BOTH) {
        if (c->jct_both_ticks < 255u) c->jct_both_ticks++;
    } else {
        c->jct_both_ticks = 0u;
    }
    if (c->jct_both_ticks >= VC_STRADDLE_JCT_BOTH_TICKS) {
        return true;       /* both under a crossing at once: certain */
    }
    return c->jct_candidate_steady &&
        ((left_recent && right_recent) ||
         (tape && (left_recent || right_recent)));
}

static void update_junction_state(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    UB line_flags = in->line_flags;
    bool tape = crossing_tape(c, in);
    bool junction;

    c->tape_now = tape;
    if (c->line_straddle) {
        junction = straddle_junction(c, in, tape);
    } else {
        /* Both line sensors sit on the line all the time here, so only
         * the barcode sensor crossing the tape tells a junction.  B3's
         * JUNCTION flag (both >= 800 and within 150) is not required: a
         * car slightly off-centre at the tape failed it. */
        junction = tape && (line_flags & LINE_F_DETECTED) != 0u &&
            c->curve_steer < VC_JUNCTION_MAX_CURVE_STEER;
        if (tape && !junction && c->pending_navigation != B3_NAV_NONE &&
            !c->jct_waiting_logged) {
            c->jct_waiting_logged = true;
            run_log_event(in->now_ms, RUN_EV_NAV_WAITING,
                          (uint16_t)c->curve_steer);
            tm_printf((UB *)"[VC] tape under GP28 with a turn pending, not a junction: curve_steer=%d flags=%d\n",
                      (INT)c->curve_steer, (INT)line_flags);
        }
        if (!tape) c->jct_waiting_logged = false;
    }

    if (junction) {
        c->junction_clear_samples = 0u;
        if (c->junction_enter_samples < VC_JUNCTION_CONFIRM_SAMPLES) {
            c->junction_enter_samples++;
        }
        if (c->junction_enter_samples >= VC_JUNCTION_CONFIRM_SAMPLES) {
            if (!c->junction_latched) {
                run_log_event(in->now_ms, RUN_EV_JUNCTION,
                              (uint16_t)c->pending_navigation);
                tm_printf((UB *)"[VC] junction (pending turn %d)\n",
                          (INT)c->pending_navigation);
                b3_buzzer_request(B3_BEEP_JUNCTION);
            }
            c->junction_latched = true;
        }
    } else {
        c->junction_enter_samples = 0u;
        if (c->junction_latched) {
            if (c->junction_clear_samples < VC_JUNCTION_RELEASE_SAMPLES) {
                c->junction_clear_samples++;
            }
            if (c->junction_clear_samples >= VC_JUNCTION_RELEASE_SAMPLES) {
                c->junction_latched = false;
                c->junction_clear_samples = 0u;
            }
        }
    }
}

#if VC_FOLLOW_V4
/* ---- Line following v4 (see VC_FOLLOW_V4 in vehicle_controller.h) ---- */
enum { V4_FOLLOW = 0, V4_SEARCH, V4_CROSS, V4_BARCODE };
enum { BAR_IDLE = 0, BAR_BRAKE, BAR_REVERSE, BAR_SETTLE, BAR_READ };

static void v4_enter(VEHICLE_CONTROLLER *c, UB phase, uint32_t now_ms)
{
    c->v4_phase = phase;
    c->v4_sub = 0u;
    c->v4_phase_ms = now_ms;
    c->v4_motion_id = 0u;
}

/* Result of the motion command v4 is waiting for: 1 done, 0 running,
 * -1 failed or timed out (the motors are then stopped). */
static INT v4_motion_done(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    motion_status_snapshot_t result;

    if (c->v4_motion_id == 0u) return 1;
    if (motion_get_result(c->v4_motion_id, &result) == E_OK) {
        c->v4_motion_id = 0u;
        return (result.status == MOTION_STATUS_COMPLETED) ? 1 : -1;
    }
    if (in->now_ms - c->v4_phase_ms > VC4_MOTION_TIMEOUT_MS) {
        stop_motion(c);
        c->v4_motion_id = 0u;
        return -1;
    }
    return 0;
}

static bool v4_turn(VEHICLE_CONTROLLER *c, const VC_INPUT *in, INT deg,
                    bool left)
{
    if (!in->motion.physically_stopped) {
        stop_motion(c);
        return false;
    }
    if (motion_turn_degrees(deg, vc4_mm_s_to_cps(VC4_TURN_MM_S),
                            left ? MOTION_TURN_LEFT : MOTION_TURN_RIGHT,
                            &c->v4_motion_id) != E_OK) {
        return false;
    }
    c->stopped = false;
    c->drive_sent = false;
    c->v4_phase_ms = in->now_ms;
    return true;
}

static bool v4_move(VEHICLE_CONTROLLER *c, const VC_INPUT *in, float cm,
                    INT mm_s, bool forward)
{
    if (!in->motion.physically_stopped) {
        stop_motion(c);
        return false;
    }
    if (motion_move_cm(cm, vc4_mm_s_to_cps(mm_s),
                       forward ? MOTION_DIR_FORWARD : MOTION_DIR_REVERSE,
                       &c->v4_motion_id) != E_OK) {
        return false;
    }
    c->stopped = false;
    c->drive_sent = false;
    c->v4_phase_ms = in->now_ms;
    return true;
}

/* End of the run (line not found, FINISH barcode): stopped, START again. */
static void v4_end_run(VEHICLE_CONTROLLER *c, const char *why)
{
    stop_motion(c);
    c->running = false;
    c->paused = false;
    c->pending_navigation = B3_NAV_NONE;
    c->navigation = 0u;
    set_state(c, ROBOT_IDLE);
    tm_printf((UB *)"[VC4] run ended: %s\n", why);
}

static void v4_reset_follow(VEHICLE_CONTROLLER *c)
{
    vc4_reset(&c->trk);
    c->drive_sent = false;
    c->last_speed = c->last_steer = 0;
}

/* Line search: 60 toward the side the line was last seen, 120 back, 60
 * again; then wait.  Found = either sensor on the tape. */
static void v4_search(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    INT done;

    set_state(c, ROBOT_LINE_RECOVERY);
    if (vc4_black(in->centre, in->left)) {
        stop_motion(c);
        c->v4_motion_id = 0u;
        tm_printf((UB *)"[VC4] line found again after %d ms of search\n",
                  (INT)(in->now_ms - c->recovery_ms));
        run_log_event(in->now_ms, RUN_EV_NAV_DONE, 0u);
        v4_reset_follow(c); /* discard the arc that caused the search */
        v4_enter(c, V4_FOLLOW, in->now_ms);
        c->drive_sent = false;
        return;
    }
    if (in->now_ms - c->recovery_ms >= VC4_SEARCH_MS) {
        v4_end_run(c, "line lost, not found within 9 s of search");
        return;
    }
    done = v4_motion_done(c, in);
    if (done == 0) return;
    switch (c->v4_sub) {
    case 0u:
        if (c->v4_search_back_mm <= 0 ||
            v4_move(c, in, (float)c->v4_search_back_mm / 10.0f,
                    VC4_REVERSE_MM_S, false)) {
            c->v4_sub = 1u;
        }
        break;
    case 1u:
        if (v4_turn(c, in, VC4_SEARCH_SWEEP_DEG, c->v4_search_first > 0)) {
            c->v4_sub = 2u;
        }
        break;
    case 2u:
        if (v4_turn(c, in, 2 * VC4_SEARCH_SWEEP_DEG, c->v4_search_first < 0)) {
            c->v4_sub = 3u;
        }
        break;
    case 3u:
        if (v4_turn(c, in, VC4_SEARCH_SWEEP_DEG, c->v4_search_first > 0)) {
            c->v4_sub = 4u;
        }
        break;
    default:
        stop_motion(c);   /* swept: wait for the line or the timeout */
        break;
    }
}

static void v4_start_search(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    /* + steering = line left: sweep left first. */
    c->v4_search_first = c->trk.filt < 0 ? -1 : 1;
    if (c->trk.filt == 0 && c->trk.last_side != 0) {
        c->v4_search_first = c->trk.last_side;
    }
    {
        uint32_t back_mm = vc4_white_um(&c->trk, c->travel_um) / 1000u +
            VC4_SEARCH_BACK_EXTRA_MM;
        if (back_mm > VC4_SEARCH_BACK_MAX_MM) back_mm = VC4_SEARCH_BACK_MAX_MM;
        c->v4_search_back_mm = (INT)back_mm;
    }
    c->recovery_ms = in->now_ms;
    stop_motion(c);
    run_log_event(in->now_ms, RUN_EV_LINE_LOST, 0u);
    tm_printf((UB *)"[VC4] LINE LOST: backing up %d mm, then searching %s first\n",
              c->v4_search_back_mm,
              c->v4_search_first > 0 ? "left" : "right");
    v4_enter(c, V4_SEARCH, in->now_ms);
}

/* Barcode turn at a cross-junction: stop, advance 60 mm, turn. */
static void v4_cross(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    INT done;
    UB nav = c->navigation;

    set_state(c, ROBOT_NAV_COMMAND);
    /* The step's clearance wait safe-stops after 3 s WITHOUT clearance,
     * not 3 s after the junction. */
    c->navigation_ms = in->now_ms;
    done = v4_motion_done(c, in);
    if (done < 0) {
        tm_printf((UB *)"[VC4] junction move failed: stopped\n");
        safe_stop(c);
        v4_enter(c, V4_FOLLOW, in->now_ms);
        return;
    }
    if (done == 0) return;
    switch (c->v4_sub) {
    case 0u:
        if (nav == B3_NAV_STOP) {
            stop_motion(c);
            if (in->now_ms - c->v4_phase_ms < VC_NAV_STOP_MS) return;
        }
        if (v4_move(c, in, VC4_JCT_ADVANCE_CM, VC4_MOVE_MM_S, true)) {
            c->v4_sub = 1u;
        }
        return;
    case 1u: {
        bool started = true;
        if (nav == B3_NAV_FINISH) {
            run_log_event(in->now_ms, RUN_EV_NAV_DONE, (uint16_t)nav);
            v4_end_run(c, "FINISH barcode");
            v4_enter(c, V4_FOLLOW, in->now_ms);
            return;
        }
        if (nav == B3_NAV_LEFT) {
            started = v4_turn(c, in, 90, true);
        } else if (nav == B3_NAV_RIGHT) {
            started = v4_turn(c, in, 90, false);
        } else if (nav == B3_NAV_UTURN) {
            started = v4_turn(c, in, 180, true);
        } else if (nav == B3_NAV_SPIN) {
            started = v4_turn(c, in, 360, false);
        } else {
            c->v4_sub = 2u;          /* STRAIGHT / STOP: nothing to turn */
            return;
        }
        if (started) c->v4_sub = 2u;
        return;
    }
    default:
        run_log_event(in->now_ms, RUN_EV_NAV_DONE, (uint16_t)nav);
        tm_printf((UB *)"[VC4] barcode action %d done at the junction\n",
                  (INT)nav);
        c->navigation = 0u;
        v4_reset_follow(c);
        v4_enter(c, V4_FOLLOW, in->now_ms);
        return;
    }
}

static void v4_bar_clear(VEHICLE_CONTROLLER *c)
{
    c->bar_phase = BAR_IDLE;
    c->bar_clear_um = c->travel_um;
    c->bar_clear_known = true;
}

/* Reverse c->bar_back_um, then read at the barcode speed. */
static bool v4_barcode(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    INT done;

    if (c->bar_phase == BAR_BRAKE) {
        set_state(c, ROBOT_NAV_COMMAND);
        stop_motion(c);
        if (in->now_ms - c->bar_ms > VC_RESCAN_TIMEOUT_MS) {
            v4_bar_clear(c);
            v4_end_run(c, "barcode brake timeout");
            return true;
        }
        if (!in->motion.physically_stopped) return true;
        if (v4_move(c, in, (float)c->bar_back_um / 10000.0f,
                    VC4_BAR_REVERSE_MM_S, false)) {
            tm_printf((UB *)"[VC4] barcode: backing %d mm to read it again (try %d)\n",
                      (INT)(c->bar_back_um / 1000u), (INT)c->bar_tries + 1);
            run_log_event(in->now_ms, RUN_EV_RESCAN,
                          (uint16_t)(c->bar_back_um / 1000u));
            c->bar_phase = BAR_REVERSE;
        }
        return true;
    }
    if (c->bar_phase == BAR_REVERSE) {
        set_state(c, ROBOT_NAV_COMMAND);
        done = v4_motion_done(c, in);
        if (done == 0) return true;
        if (done < 0) {
            v4_bar_clear(c);
            v4_end_run(c, "barcode reverse failed");
            return true;
        }
        c->bar_phase = BAR_SETTLE;
        c->bar_ms = in->now_ms;
        return true;
    }
    if (c->bar_phase == BAR_SETTLE) {
        set_state(c, ROBOT_NAV_COMMAND);
        stop_motion(c);
        if (!in->motion.physically_stopped ||
            in->now_ms - c->bar_ms < VC4_BAR_WAIT_MS) return true;
        c->bar_phase = BAR_READ;
        c->bar_speed_cps = vc4_mm_s_to_cps(VC4_BAR_READ_MM_S);
        c->bar_mark_um = c->travel_um;
        c->bar_ms = in->now_ms;
        c->bar_rejects = in->barcode_rejects;
        v4_reset_follow(c);
        vc4_junction_reset(&c->trk);
        return false;   /* following drives the read pass */
    }
    return false;
}

/* Read pass finished without a decode: retry from its start or give up. */
static void v4_bar_pass_failed(VEHICLE_CONTROLLER *c, const VC_INPUT *in,
                               const char *why)
{
    uint32_t pass = c->travel_um - c->bar_mark_um;

    tm_printf((UB *)"[VC4] barcode read pass failed (%s) after %d mm\n",
              why, (INT)(pass / 1000u));
    if (c->bar_tries < VC4_BAR_RETRIES) {
        c->bar_tries++;
        c->bar_back_um = pass + VC4_BAR_RETRY_EXTRA_UM;
        if (c->bar_back_um > VC4_BAR_REWIND_MAX_UM) {
            c->bar_back_um = VC4_BAR_REWIND_MAX_UM;
        }
        c->bar_phase = BAR_BRAKE;
        c->bar_ms = in->now_ms;
    } else {
        tm_printf((UB *)"[VC4] barcode not read after %d tries: stopping\n",
                  (INT)VC4_BAR_RETRIES + 1);
        v4_bar_clear(c);
        v4_end_run(c, "barcode retry limit reached");
    }
}

static void v4_track_strip(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    if (!c->strip_known) {
        c->strip_known = true;
        c->strip_level = in->right;
    } else if (in->right != c->strip_level) {
        c->strip_level = in->right;
        if (!c->strip_active) {
            c->strip_active = true;
            c->strip_start_um = c->travel_um;
            c->strip_edges = 0u;
        }
        if (c->strip_edges < 255u) c->strip_edges++;
        c->strip_last_um = c->travel_um;
    } else if (c->strip_active &&
               c->travel_um - c->strip_last_um >= VC_STRIP_END_UM) {
        c->strip_active = false;
    }
}

static void v4_ramp(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    bool pitched = in->hump || in->pitch_deg >= VC4_RAMP_PITCH_DEG ||
        in->pitch_deg <= -VC4_RAMP_PITCH_DEG;

    if (pitched) {
        if (!c->on_ramp) tm_printf((UB *)"[VC4] on a ramp: short obstacle stop\n");
        c->on_ramp = true;
        c->ramp_ms = in->now_ms;
        c->ramp_um = c->travel_um;
    } else if (c->on_ramp && in->now_ms - c->ramp_ms >= VC4_RAMP_HOLD_MS &&
               c->travel_um - c->ramp_um >= VC4_RAMP_CLEAR_UM) {
        c->on_ramp = false;
    }
    s5_mission_set_ramp(c->on_ramp);
}

/* True when the car has been picked up (lift-stop). */
static bool v4_lifted(VEHICLE_CONTROLLER *c, const VC_INPUT *in)
{
    if (in->raw26 >= VC4_LIFT_IR_RAW && in->raw27 >= VC4_LIFT_IR_RAW) {
        if (c->lift_ir_ms == 0u) c->lift_ir_ms = in->now_ms | 1u;
        if (in->now_ms - c->lift_ir_ms >= VC4_LIFT_IR_MS) return true;
    } else {
        c->lift_ir_ms = 0u;
    }
    return in->lifted_imu;
}

/* Follow / search / junction / barcode.  speed = dashboard cruise, cps. */
static void v4_drive(VEHICLE_CONTROLLER *c, const VC_INPUT *in, UB nav,
                     INT speed)
{
    uint32_t dt = c->v4_time_valid ? in->now_ms - c->v4_last_ms : VC_PERIOD_MS;
    UH nl = in->centre;   /* robot's left sensor  (GP27) */
    UH nr = in->left;     /* robot's right sensor (GP26) */
    int32_t steer, speed_cps, steering;
    INT cruise_mm, base_mm;
    motion_command_t command = {0};
    vc4_jct_t jct = VC4_JCT_NONE;
    bool inhibit, barcode_straight;

    VC_INPUT filtered = *in;

    bar_filter(c, &filtered);   /* a line sensor over barcode bars */
    in = &filtered;
    nl = in->centre;
    nr = in->left;
    c->v4_last_ms = in->now_ms;
    c->v4_time_valid = true;
    (void)crossing_tape(c, in);           /* odometer */
    v4_track_strip(c, in);
    v4_ramp(c, in);
    if (c->post_avoid_slow &&
        (int32_t)(c->travel_um - c->post_avoid_um) >= 0) {
        c->post_avoid_slow = false;
        tm_printf((UB *)"[VC4] 150 mm after the bypass: cruise speed, junctions on\n");
    }

    /* A decoded barcode arms the next cross-junction (a newer one
     * replaces it), whatever the car is doing. */
    /* Decodes while stopping/reversing over a strip are partial or
     * backwards reads: ignored (the slow pass reads it). */
    if (c->bar_phase != BAR_IDLE && c->bar_phase != BAR_READ) nav = B3_NAV_NONE;
    if (c->mode == MODE_AUTO && nav >= B3_NAV_LEFT && nav < B3_NAV_COUNT) {
        c->pending_navigation = nav;
        tm_printf((UB *)"[VC4] barcode action %d armed for the next cross-junction\n",
                  (INT)nav);
        if (c->bar_phase != BAR_IDLE) {
            if (c->bar_phase != BAR_READ) stop_motion(c);
            v4_bar_clear(c);
            c->v4_motion_id = 0u;
        }
    }

    if (c->v4_phase == V4_SEARCH) {
        v4_search(c, in);
        return;
    }
    if (c->v4_phase == V4_CROSS) {
        v4_cross(c, in);
        return;
    }
    if (c->bar_phase != BAR_IDLE && c->bar_phase != BAR_READ) {
        if (v4_barcode(c, in)) return;
    }

    /* ---- FOLLOW ---- */
    /* Bars passing under GP28 now (B3 edge activity, or the strip seen
     * in the last VC_STRIP_END_UM of travel, which covers wide gaps). */
    barcode_straight = in->barcode_active ||
        (c->strip_active && c->strip_edges >= VC4_BAR_STRIP_EDGES);
    /* Keep following the line across a barcode.  Holding the steering at
     * zero (and wiping the curve memory) for the whole read made the car
     * drive straight off a curve that starts just after the bars, since a
     * read lasts until a decode, up to VC4_BAR_READ_UM.  While bars pass,
     * the steering is only limited (bar_filter already ignores a line
     * sensor flickering on them) and a loss does not start a sweep. */
    steer = vc4_steer(&c->trk, nl, nr, dt, c->travel_um);
    if (barcode_straight) {
        if (steer > VC4_BAR_STEER_MAX) steer = VC4_BAR_STEER_MAX;
        if (steer < -VC4_BAR_STEER_MAX) steer = -VC4_BAR_STEER_MAX;
    }
    if (!vc4_on_line(&c->trk) && !barcode_straight) {
        if (c->bar_phase == BAR_READ) v4_bar_clear(c);
        v4_start_search(c, in);
        return;
    }
    if (c->bar_phase == BAR_READ) {
        if (in->barcode_rejects != c->bar_rejects && !in->barcode_active) {
            c->bar_rejects = in->barcode_rejects;
            v4_bar_pass_failed(c, in, "rejected");
            if (!c->running) return;
            if (c->bar_phase == BAR_BRAKE) { stop_motion(c); return; }
        } else if (c->travel_um - c->bar_mark_um >= VC4_BAR_READ_UM ||
                   in->now_ms - c->bar_ms >= VC4_BAR_READ_MS) {
            v4_bar_pass_failed(c, in, "no decode");
            if (!c->running) return;
            if (c->bar_phase == BAR_BRAKE) { stop_motion(c); return; }
        }
    }

    inhibit = c->post_avoid_slow || barcode_straight ||
        c->bar_phase != BAR_IDLE;
    if (!inhibit) {
        jct = vc4_junction(&c->trk, nl, nr, c->travel_um);
    } else {
        vc4_junction_reset(&c->trk);
    }
    if (jct != VC4_JCT_NONE) {
        run_log_event(in->now_ms, RUN_EV_JUNCTION,
                      (uint16_t)c->pending_navigation);
        tm_printf((UB *)"[VC4] junction class %d (1 cross, 2 left, 3 right), armed %d\n",
                  (INT)jct, (INT)c->pending_navigation);
    }
    if (jct == VC4_JCT_CROSS && c->mode == MODE_AUTO &&
        c->pending_navigation != B3_NAV_NONE) {
        if (c->bar_phase == BAR_READ) v4_bar_clear(c);
        c->navigation = c->pending_navigation;
        c->pending_navigation = B3_NAV_NONE;
        c->navigation_ms = in->now_ms;
        c->navigation_id = 0u;
        run_log_event(in->now_ms, RUN_EV_NAV_START, (uint16_t)c->navigation);
        stop_motion(c);
        v4_enter(c, V4_CROSS, in->now_ms);
        set_state(c, ROBOT_NAV_COMMAND);
        return;
    }

    /* Barcode start: bars under GP28 while following.  Read it on this
     * pass at the speed the car already has (held steady while the bars
     * pass, below: the decoder times bar widths).  Only a rejected read
     * backs up and reads again.  A crossing tape gives one black run
     * (2 edges), not 3+. */
    if (c->mode == MODE_AUTO && c->bar_phase == BAR_IDLE &&
        c->strip_active && c->strip_edges >= VC4_BAR_STRIP_EDGES &&
        c->trk.jct_state == 0u && !c->post_avoid_slow &&
        (!c->bar_clear_known ||
         ((int32_t)(c->strip_start_um - c->bar_clear_um) > 0 &&
          c->travel_um - c->bar_clear_um >= VC4_BAR_REARM_UM))) {
        c->bar_tries = 0u;
        c->bar_phase = BAR_READ;
        c->bar_speed_cps = c->drive_sent && c->last_speed > 0 ?
            c->last_speed : vc4_mm_s_to_cps(VC4_BAR_READ_MM_S);
        c->bar_ms = in->now_ms;
        c->bar_rejects = in->barcode_rejects;
        /* A failed pass backs up (travel - mark) + RETRY_EXTRA: start the
         * mark so that is the strip start + LEAD + BACK_EXTRA. */
        c->bar_mark_um = c->strip_start_um - VC4_BAR_LEAD_UM -
            VC4_BAR_BACK_EXTRA_UM + VC4_BAR_RETRY_EXTRA_UM;
        tm_printf((UB *)"[VC4] barcode start seen: reading it at a steady speed\n");
    }

    set_state(c, in->hump ? ROBOT_HUMP : ROBOT_LINE_FOLLOW);
    if (c->drive_sent && in->now_ms - c->last_drive_ms < VC_PERIOD_MS) return;

    cruise_mm = (INT)((int32_t)speed * (int32_t)VC4_UM_PER_COUNT / 1000);
    base_mm = cruise_mm;
    if (c->post_avoid_slow || in->approach_slow || c->slow_mode) {
        INT slow = cruise_mm * VC4_SLOW_MM_S / VC4_CRUISE_MM_S;
        if (base_mm > slow) base_mm = slow;
    }
    vc4_wheels(steer, base_mm,
               cruise_mm * VC4_SLOW_MM_S / VC4_CRUISE_MM_S,
               &speed_cps, &steering);
    command.type = MOTION_CMD_DRIVE;
    command.direction = MOTION_DIR_FORWARD;
    command.steering_permille = (INT)steering;
    command.target_speed_cps = (INT)speed_cps;
    /* Fixed read speed while bars pass, and on a retry from the rewind
     * on; after the bars of a first pass the curve slow-down applies. */
    if (c->bar_phase == BAR_READ && (barcode_straight || c->bar_tries > 0u)) {
        command.target_speed_cps = c->bar_speed_cps > 0 ?
            c->bar_speed_cps : vc4_mm_s_to_cps(VC4_BAR_READ_MM_S);
    }
    /* Gentle start from standstill; slowing applies at once. */
    {
        INT previous = c->drive_sent ? c->last_speed : 0;
        if (command.target_speed_cps > previous + VC_SPEED_RISE_PER_TICK) {
            command.target_speed_cps = previous + VC_SPEED_RISE_PER_TICK;
        }
    }
    /* Constant speed across a barcode (as on barcode-robust): bar widths
     * are timed, so a curve slowdown part-way through the bars stretches
     * the later ones and the read is rejected.  Keep the speed the car had
     * at the first bar for the complete pass, including quiet gaps. */
    if (c->bar_phase != BAR_READ && (in->barcode_active || barcode_straight) &&
        c->drive_sent && c->last_speed > 0) {
        command.target_speed_cps = c->last_speed;
    }
    if (command.target_speed_cps < 1) command.target_speed_cps = 1;
    c->last_speed = command.target_speed_cps;
    c->last_steer = command.steering_permille;
    c->follow_steer = command.steering_permille;
    c->curve_steer = (INT)(steer < 0 ? -steer : steer);
    command.safety_generation = in->safety_generation;
    if (motion_send_command(&command) == E_OK) {
#if VC_STEERING_DIAGNOSTICS
        report_steering(c, in, &command);
#endif
        c->stopped = false;
        c->drive_sent = true;
        c->last_drive_ms = in->now_ms;
    } else if (!c->drive_sent || in->now_ms - c->last_drive_ms >= 60u) {
        safe_stop(c);
    }
}
#endif /* VC_FOLLOW_V4 */

void vehicle_controller_init(VEHICLE_CONTROLLER *c)
{
    *c = (VEHICLE_CONTROLLER){0};
    c->line_pos = B3_LINE_POS_CENTRE;
    c->mode = MODE_AUTO;
    c->speed_percent = VC_FOLLOW_V4 ? 29u : 20u; /* v4: 29 % = 120 mm/s */
#if VC_FOLLOW_V4
    vc4_reset(&c->trk);
#endif
    c->line_straddle = VC_LINE_STRADDLE != 0;
    c->approach_um = VC_JUNCTION_APPROACH_MM * 1000u;
    tm_printf((UB *)"[VC] line sensors: %s layout (VC_LINE_STRADDLE)\n",
              c->line_straddle ? "SPREAD (line between them)" :
              "TOGETHER (both on the line)");
    c->state = ROBOT_INIT;
    stop_motion(c);
    set_state(c, ROBOT_IDLE);
}

void vehicle_controller_command(VEHICLE_CONTROLLER *c, const REMOTE_CMD *cmd)
{
    switch (cmd->type) {
    case RCMD_START:
        if (c->mode != MODE_TEST) {
            if (!c->running) {
                c->line_recovery_active = false;
                c->follow_steer = 0;
                c->error_filter_valid = false;
                c->slow_mode = false; /* a new run starts at normal speed */
#if VC_FOLLOW_V4
                vc4_reset(&c->trk);
                c->v4_phase = V4_FOLLOW;
                c->v4_sub = 0u;
                c->v4_motion_id = 0u;
                c->bar_phase = BAR_IDLE;
                c->bar_clear_known = false;
                c->post_avoid_slow = false;
                c->navigation = 0u;
                c->pending_navigation = B3_NAV_NONE;
                c->lift_ir_ms = 0u;
#endif
            }
            c->estop = false; /* Only an explicit START can release ESTOP. */
            c->running = true;
            c->paused = false;
            /* B2 latches stall/encoder faults until reboot and every step
             * safe-stops while one is set, so after one stall no dashboard
             * command could move the car again.  START is the operator's
             * acknowledgement: clear them; B2 re-raises them if still real.
             * B2 also keeps publishing the stalled command's FAULT status
             * until it runs a new command (which nobody would send), and
             * B5 latched on it: acknowledge it there too, and give B5 a
             * moment to re-arm before its mission FAULT counts again. */
            fault_clear(FAULT_MOTOR_STALL | FAULT_ENCODER);
            motion_acknowledge_fault();
            c->ack_grace_pending = true;
            c->stall_retries = 0u;
            c->hold_active = c->tolerate_active = false;
        }
        break;
    case RCMD_RESUME:
        if (c->paused && !c->estop && c->mode != MODE_TEST) {
            c->running = true;
            c->paused = false;
        }
        break;
    case RCMD_ESTOP:
        c->estop = true;
        safe_stop(c);
        break;
    case RCMD_STOP:
        safe_stop(c);
        break;
    case RCMD_PAUSE:
        if (c->running) c->paused = true;
        c->running = false;
        c->navigation = 0u;
        c->navigation_id = 0u;
        c->pending_navigation = 0u;
        c->junction_enter_samples = 0u;
        c->junction_clear_samples = 0u;
        c->junction_latched = false;
        c->rescan_phase = 0u;
#if VC_FOLLOW_V4
        c->v4_phase = 0u;
        c->v4_motion_id = 0u;
        c->bar_phase = 0u;
#endif
        stop_motion(c);
        if (!c->estop) set_state(c, ROBOT_IDLE);
        break;
    case RCMD_MODE:
        if (cmd->arg < MODE_COUNT) {
            safe_stop(c);
            c->mode = (RUN_MODE)cmd->arg;
            if (!c->estop) set_state(c, ROBOT_IDLE);
        }
        break;
    case RCMD_SPEED:
        if (cmd->arg <= 100u) {
            c->speed_percent = cmd->arg;
            if (cmd->arg == 0u) {
                c->navigation = 0u;
                c->navigation_id = 0u;
                c->pending_navigation = B3_NAV_NONE;
                c->junction_enter_samples = 0u;
                c->junction_clear_samples = 0u;
                c->junction_latched = false;
                stop_motion(c);
            }
        }
        break;
    case RCMD_RESET_PEAK:
        CHECK(subsystem4_reset_peak(), "VC reset peak");
        break;
    default: break;
    }
}

void vehicle_controller_step(VEHICLE_CONTROLLER *c, const VC_INPUT *in, UB nav)
{
    INT speed;
    motion_command_t command = {0};
    bool line_detected;
    bool new_reject = c->rejects_known &&
        in->barcode_rejects != c->seen_rejects;

    c->seen_rejects = in->barcode_rejects;
    c->rejects_known = true;
    if (nav != B3_NAV_NONE && c->rescan_phase == 0u) {
        c->rescan_tries = 0u;      /* a barcode was read */
        c->rescan_pending = false;
    }
    if (c->rescan_tries != 0u && c->rescan_phase == 0u &&
        in->now_ms - c->rescan_ms >= VC_RESCAN_FORGET_MS) {
        c->rescan_tries = 0u;
    }
    if (!c->running || c->estop || c->mode == MODE_TEST) {
        stop_motion(c);
        return;
    }
#if VC_FOLLOW_V4
    if (v4_lifted(c, in)) {
        tm_printf((UB *)"[VC4] LIFTED (line sensors raw %d/%d, imu %d): stopped, START to run again\n",
                  (INT)in->raw26, (INT)in->raw27, (INT)in->lifted_imu);
        safe_stop(c);
        return;
    }
#endif
    if (c->ack_grace_pending) {
        c->ack_grace_pending = false;
        c->tolerate_active = true;
        c->tolerate_until_ms = in->now_ms + VC_FAULT_ACK_GRACE_MS;
    }
    if (c->hold_active && (int32_t)(c->hold_until_ms - in->now_ms) <= 0) {
        c->hold_active = false;
    }
    if (c->tolerate_active &&
        (int32_t)(c->tolerate_until_ms - in->now_ms) <= 0) {
        c->tolerate_active = false;
    }
    if (c->stall_retries != 0u &&
        in->now_ms - c->last_stall_ms >= VC_STALL_RETRY_RESET_MS) {
        c->stall_retries = 0u;
    }
    /* Motor stall (wheel blocked or too slow): stop, wait, and try again a
     * couple of times before giving up.  Encoder/IMU faults never retry. */
    if (in->motion_valid &&
        ((in->faults & FAULT_MOTOR_STALL) != 0u ||
         (motion_fault_active(&in->motion) &&
          in->motion.fault == MOTION_FAULT_STALL)) &&
        (in->faults & (FAULT_ENCODER | FAULT_IMU)) == 0u) {
        if (c->stall_retries < VC_STALL_AUTO_RETRIES) {
            c->stall_retries++;
            c->last_stall_ms = in->now_ms;
            motion_acknowledge_fault();
            fault_clear(FAULT_MOTOR_STALL);
            c->hold_until_ms = in->now_ms + VC_STALL_RETRY_MS;
            c->tolerate_until_ms = c->hold_until_ms + VC_FAULT_ACK_GRACE_MS;
            c->hold_active = c->tolerate_active = true;
            c->drive_sent = false;
            c->last_speed = c->last_steer = 0;
            c->error_filter_valid = false;
            stop_motion(c);
            tm_printf((UB *)"[VC] motor stall: automatic retry %d of %d in %d ms\n",
                      (INT)c->stall_retries, (INT)VC_STALL_AUTO_RETRIES,
                      (INT)VC_STALL_RETRY_MS);
            return;
        }
        tm_printf((UB *)"[VC] motor stall: %d retries failed; stopped. Clear the wheels and press START\n",
                  (INT)VC_STALL_AUTO_RETRIES);
        safe_stop(c);
        return;
    }
    if (c->hold_active) {
        stop_motion(c); /* stall retry pause */
        return;
    }
    if (in->obstacle_fault && c->tolerate_active) {
        stop_motion(c); /* B5 re-arming after an acknowledged fault */
        return;
    }
    if (!in->motion_valid || !in->imu_valid || in->hazard || in->obstacle_fault ||
        (in->faults & (FAULT_MOTOR_STALL | FAULT_ENCODER | FAULT_IMU)) ||
        motion_fault_active(&in->motion)) {
        tm_printf((UB *)"[VC-STOP-REASON] motion_valid=%d imu_valid=%d hazard=%d obstacle_fault=%d faults=%d motion_status=%d\n",
                  (INT)in->motion_valid, (INT)in->imu_valid, (INT)in->hazard,
                  (INT)in->obstacle_fault, (INT)in->faults, (INT)in->motion.status);
        safe_stop(c);
        return;
    }
    if (c->speed_percent == 0u) {
        stop_motion(c);
        set_state(c, ROBOT_IDLE);
        return;
    }
    if (!in->line_valid) {
        tm_printf((UB *)"[VC-STOP-REASON] line data invalid/stale\n");
        safe_stop(c);
        return;
    }
    if (in->obstacle_active) {
        /* B5 owns bounded bypass/recovery legs; never overwrite them. */
        c->obstacle_handoff = true;
        c->drive_sent = false;
        c->stopped = false;
        c->rescan_phase = 0u;
        c->navigation = 0u;
        c->navigation_id = 0u;
#if !VC_FOLLOW_V4
        c->pending_navigation = B3_NAV_NONE;
#endif
        c->junction_latched = false;
        c->junction_enter_samples = c->junction_clear_samples = 0u;
#if VC_FOLLOW_V4
        /* v4: an armed barcode action survives the bypass. */
        c->v4_motion_id = 0u;
        c->v4_time_valid = false;
        (void)crossing_tape(c, in);   /* odometer keeps counting */
#endif
        return;
    }
    if (c->obstacle_handoff) {
        /* The bypass changed position and heading. Discard steering/speed
         * history from before the obstacle and give a lost line a fresh
         * recovery window, even if we entered avoidance during recovery. */
        c->obstacle_handoff = false;
        c->drive_sent = false;
        c->last_steer = c->last_error = c->curve_steer = c->last_speed = 0;
        c->follow_steer = 0;
        c->error_filter_valid = false;
        c->line_pos = B3_LINE_POS_CENTRE;
        c->last_line_search_direction = 0;
        c->recovery_ms = in->now_ms;
        c->line_recovery_active = false;
        c->straddle_black_ms = in->now_ms;
        c->straddle_lost = false;
        c->straddle_outside = 0;
#if VC_BEND_HOLD
        bend_reset(c);
#endif
#if VC_FOLLOW_V4
        /* After the bypass: slow, no junctions for 150 mm. */
        vc4_reset(&c->trk);
        c->v4_phase = V4_FOLLOW;
        c->v4_motion_id = 0u;
        c->bar_phase = BAR_IDLE;
        c->post_avoid_slow = true;
        c->post_avoid_um = c->travel_um + VC4_POST_AVOID_UM;
#endif
    }
    if (!in->clearance || (in->faults & FAULT_NO_ECHO)) {
        if (c->state != ROBOT_OBSTACLE_SCAN) {
            tm_printf((UB *)"[VC-CLEARANCE-WAIT] now_ms=%d allowed=%d no_echo=%d generation=%d\n",
                      (INT)in->now_ms, (INT)in->clearance,
                      (INT)((in->faults & FAULT_NO_ECHO) != 0u), (INT)in->safety_generation);
        }
        if (c->navigation != 0u) {
            /* Our navigation STOP revokes clearance. Wait for a fresh B5
             * report before submitting the turn, without losing the request. */
            if (c->navigation_id != 0u ||
                in->now_ms - c->navigation_ms >= VC_RECOVERY_MS) safe_stop(c);
            return;
        }
        stop_motion(c);
        c->navigation = 0u;
        c->navigation_id = 0u;
        set_state(c, ROBOT_OBSTACLE_SCAN);
        return;
    }
    /* Speed barcodes act as soon as they are read, not at a crossing. */
    if (c->mode == MODE_AUTO && (nav == B3_NAV_SLOW || nav == B3_NAV_NORMAL)) {
        c->slow_mode = nav == B3_NAV_SLOW;
        tm_printf((UB *)"[VC] barcode: %s speed\n",
                  c->slow_mode ? "half" : "normal");
        nav = B3_NAV_NONE;
    }
    (void)motion_speed_from_percent((INT)c->speed_percent, &speed);
    if (c->slow_mode) speed /= 2;
#if VC_FOLLOW_V4
    v4_drive(c, in, nav, speed);
    return;
#endif
    if (in->approach_slow && speed > ROBOT_BYPASS_SPEED_CPS) speed = ROBOT_BYPASS_SPEED_CPS;
    VC_INPUT filtered;
    if (c->line_straddle) {
        filtered = *in;
        bar_filter(c, &filtered);
        in = &filtered;
    }
    if (c->line_straddle) {
        /* Standing still (START, after a turn, a stall retry or a
         * clearance wait) is not "no line seen for a while". */
        if (!c->drive_sent && !c->line_recovery_active) {
            c->straddle_black_ms = in->now_ms;
        }
        line_detected = straddle_following(c, in);
        if (c->bars_active) c->straddle_outside = 0; /* bars, not exits */
    } else {
        line_detected = ((in->line_flags & LINE_F_DETECTED) != 0u) &&
            ((in->line_flags & LINE_F_LOST) == 0u);
    }
    if (line_detected) c->line_recovery_active = false;
    if (c->rescan_phase != 0u || c->navigation != 0u ||
        c->line_recovery_active || !c->drive_sent || c->bars_active) {
        c->jct_block_um = c->travel_um;   /* not following steadily */
    }
    if (c->rescan_phase == 0u) {
        update_junction_state(c, in);
    } else {
        (void)crossing_tape(c, in);  /* odometer only while re-reading */
    }
    /* GP28 strip episode: from its first change to VC_STRIP_END_UM of
     * travel without one (the barcode re-read reverses over all of it). */
    if (!c->strip_known) {
        c->strip_known = true;
        c->strip_level = in->right;
    } else if (in->right != c->strip_level) {
        c->strip_level = in->right;
        if (!c->strip_active) {
            c->strip_active = true;
            c->strip_start_um = c->travel_um;
            c->strip_edges = 0u;
        }
        if (c->strip_edges < 255u) c->strip_edges++;
        c->strip_last_um = c->travel_um;
    } else if (c->strip_active &&
               c->travel_um - c->strip_last_um >= VC_STRIP_END_UM) {
        c->strip_active = false;
    }
    if (c->mode == MODE_AUTO && c->navigation == 0u &&
        c->pending_navigation == B3_NAV_NONE &&
        nav >= B3_NAV_LEFT && nav < B3_NAV_COUNT) {
        c->pending_navigation = nav;
    }
    if (new_reject && c->mode == MODE_AUTO && c->navigation == 0u &&
        c->pending_navigation == B3_NAV_NONE && c->rescan_phase == 0u &&
        c->rescan_tries < VC_BARCODE_RESCANS) {
        c->rescan_pending = true;   /* reverse once past the strip */
    }
    if (c->rescan_pending && (nav != B3_NAV_NONE ||
                              c->pending_navigation != B3_NAV_NONE ||
                              c->mode != MODE_AUTO || c->navigation != 0u)) {
        c->rescan_pending = false;  /* read after all, or busy */
    }
    if (c->rescan_pending && !c->strip_active) {
        c->rescan_pending = false;
        c->rescan_tries++;
        c->rescan_back_um = c->travel_um - c->strip_start_um +
            VC_RESCAN_MARGIN_UM;
        if (c->rescan_back_um > VC_RESCAN_MAX_UM) {
            c->rescan_back_um = VC_RESCAN_MAX_UM;
        }
        c->rescan_phase = 1u;
        c->rescan_ms = in->now_ms;
        tm_printf((UB *)"[VC] barcode rejected: backing up %d mm to read it again\n",
                  (INT)(c->rescan_back_um / 1000u));
        run_log_event(in->now_ms, RUN_EV_RESCAN,
                      (uint16_t)(c->rescan_back_um / 1000u));
    }
    if (c->rescan_phase != 0u && barcode_rescan(c, in, speed)) return;
    if (c->mode == MODE_AUTO && c->navigation == 0u &&
        c->pending_navigation != B3_NAV_NONE && c->junction_latched &&
        !c->approach_active) {
        /* Keep following until the wheels are on the junction. */
        c->approach_active = true;
        c->approach_start_um = c->travel_um;
    }
    if (c->approach_active && (c->pending_navigation == B3_NAV_NONE ||
                               c->navigation != 0u ||
                               c->mode != MODE_AUTO)) {
        c->approach_active = false;
    }
    if (c->approach_active &&
        c->travel_um - c->approach_start_um >= c->approach_um) {
        c->approach_active = false;
        c->navigation = c->pending_navigation;
        c->pending_navigation = B3_NAV_NONE;
        c->navigation_ms = in->now_ms;
        run_log_event(in->now_ms, RUN_EV_NAV_START, (uint16_t)c->navigation);
        stop_motion(c);
        set_state(c, ROBOT_NAV_COMMAND);
        return;
    }
    if (c->navigation == 0u && line_detected) {
        (void)line_position(c, in); /* remember which side the line is on */
        c->line_seen_ms = in->now_ms;
    }
    if (c->navigation == 0u && !line_detected) {
        c->error_filter_valid = false; /* a reacquired line starts fresh */
#if VC_BEND_HOLD
        bend_reset(c);
#endif
        uint32_t recovery_elapsed;
        INT direction;
        INT search_steer = VC_LINE_SEARCH_STEER;

        if (!c->line_recovery_active) {
            c->recovery_ms = in->now_ms;
            c->line_recovery_active = true;
            run_log_event(in->now_ms, RUN_EV_LINE_LOST, 0u);
            c->spin_dir = c->last_line_search_direction != 0 ?
                c->last_line_search_direction : 1;
            c->spin_steps = 0u;
            c->spin_sweep = 0u;
            c->spin_target = VC_SPIN_FIRST_STEPS;
            c->spin_id = 0u;
            c->spin_done = false;
        }
        recovery_elapsed = in->now_ms - c->recovery_ms;
        set_state(c, ROBOT_LINE_RECOVERY);
        if (c->line_straddle && recovery_elapsed < VC_LINE_SEARCH_MS) {
            straddle_spin_search(c, in);
            return;
        }
        if (recovery_elapsed >= VC_LINE_SEARCH_MS) {
            /* Search exhausted: stop but stay armed.  Line following
             * resumes by itself once B3 sees the line again (e.g. the car
             * is put back on it); no new START is needed.  Dashboard STOP,
             * faults and stale data still disarm as before. */
            stop_motion(c);
            return;
        }

        direction = c->last_line_search_direction;
        if (direction == 0) direction = 1;
        if (recovery_elapsed < VC_LINE_REACQUIRE_MS) {
            /* Pololu behaviour: turn hard back toward the last side. */
            search_steer = B3_LINE_STEER_MAX;
        } else {
            /* Not found there: widening sweeps, starting the other way. */
            direction = line_search_direction(
                recovery_elapsed - VC_LINE_REACQUIRE_MS, -direction);
        }
        if (c->drive_sent && in->now_ms - c->last_drive_ms < VC_PERIOD_MS) {
            return;
        }
        command.type = MOTION_CMD_DRIVE;
        command.direction = MOTION_DIR_FORWARD;
        command.steering_permille = direction * search_steer;
        command.target_speed_cps = speed / 2;
        if (command.target_speed_cps == 0) command.target_speed_cps = 1;
        command.target_speed_cps = command.target_speed_cps * 1000 /
            (1000 + search_steer);
        c->last_steer = command.steering_permille;
        c->last_error = direction * B3_LINE_POS_CENTRE;
        apply_wheel_minimum(&command, true);
        command.safety_generation = in->safety_generation;
        if (motion_send_command(&command) == E_OK) {
#if VC_STEERING_DIAGNOSTICS
            report_steering(c, in, &command);
#endif
            c->stopped = false;
            c->drive_sent = true;
            c->last_drive_ms = in->now_ms;
        } else if (!c->drive_sent ||
                   in->now_ms - c->last_drive_ms >= 60u) {
            safe_stop(c);
        }
        return;
    }
    if (c->navigation != 0u) {
        set_state(c, ROBOT_NAV_COMMAND);
        if (c->navigation_id != 0u) {
            motion_status_snapshot_t result;
            if (motion_get_result(c->navigation_id, &result) == E_OK) {
                if (result.status != MOTION_STATUS_COMPLETED || !result.physically_stopped) {
                    safe_stop(c);
                    return;
                }
                run_log_event(in->now_ms, RUN_EV_NAV_DONE,
                              (uint16_t)c->navigation);
                if (c->line_straddle && c->navigation != B3_NAV_STRAIGHT) {
                    /* Turn ended short on purpose (VC_TURN_SHORT_DEG):
                     * keep turning the same way until a sensor finds the
                     * line, then follow it. */
                    c->straddle_lost = true;
                    c->straddle_outside = 0;
                    c->line_recovery_active = false;
                    c->last_line_search_direction =
                        (c->navigation == B3_NAV_RIGHT ||
                         c->navigation == B3_NAV_SPIN) ? 1 : -1;
                }
                c->navigation = 0u;
                c->navigation_id = 0u;
            } else if (in->now_ms - c->navigation_ms > 65000u) {
                safe_stop(c);
            }
            return;
        }
        if (c->navigation == B3_NAV_FINISH) {
            /* End of the run: stopped, B5 disarmed, START begins again. */
            stop_motion(c);
            c->running = false;
            c->paused = false;
            c->navigation = 0u;
            c->navigation_id = 0u;
            set_state(c, ROBOT_IDLE);
            tm_printf((UB *)"[VC] barcode FINISH: run complete\n");
            return;
        }
        if (c->navigation == B3_NAV_STOP) {
            /* Wait at the crossing, then cross it like STRAIGHT. */
            stop_motion(c);
            if (in->now_ms - c->navigation_ms < VC_NAV_STOP_MS) return;
            c->navigation = B3_NAV_STRAIGHT;
            c->navigation_ms = in->now_ms;
        }
        if (in->now_ms - c->navigation_ms > VC_RECOVERY_MS) {
            safe_stop(c);
            return;
        }
        if (!in->motion.physically_stopped) return;
        ER error;
        if (c->navigation == B3_NAV_STRAIGHT) {
            error = motion_move_cm(10.0f, speed, MOTION_DIR_FORWARD, &c->navigation_id);
        } else if (c->navigation == B3_NAV_SPIN) {
            error = motion_turn_degrees(
                c->line_straddle ? 360 - VC_TURN_SHORT_DEG : 360, speed,
                MOTION_TURN_RIGHT, &c->navigation_id);
        } else {
            INT angle = c->navigation == B3_NAV_UTURN ? 180 : 90;
            if (c->line_straddle) angle -= VC_TURN_SHORT_DEG;
            error = motion_turn_degrees(angle,
                speed, c->navigation == B3_NAV_RIGHT ? MOTION_TURN_RIGHT : MOTION_TURN_LEFT,
                &c->navigation_id);
        }
        if (error == E_OK) {
            c->stopped = false;
            c->navigation_ms = in->now_ms;
        }
        return;
    }
    set_state(c, in->hump ? ROBOT_HUMP : ROBOT_LINE_FOLLOW);
    if (c->spin_id != 0u) {
        /* Line found during an in-place search step: stop the turn first
         * (a DRIVE in the same tick would race B2's stop). */
        stop_motion(c);
        c->spin_id = 0u;
        return;
    }
    if (c->drive_sent && in->now_ms - c->last_drive_ms < VC_PERIOD_MS) return;
    command.type = MOTION_CMD_DRIVE;
    command.direction = MOTION_DIR_FORWARD;
    /* PD on the line position: balanced sensors give position 500 and
     * zero steering, so a junction is still crossed straight, while a curve
     * is followed even when both sensors see the line. */
    command.steering_permille = line_follow_steering(c, in);
#if VC_BEND_HOLD
    if (c->line_straddle && c->straddle_outside == 0 && !in->barcode_active &&
        c->rescan_phase == 0u) {
        INT held = straddle_bend(c, in, command.steering_permille);
        if (held != command.steering_permille) {
            command.steering_permille = held;
            c->straddle_black_ms = in->now_ms; /* no 0.4 s seek on top */
        }
    } else {
        bend_reset(c);
    }
#endif
    if (c->line_straddle && c->straddle_outside != 0) {
        /* Line left a sensor: coast, then a growing arc toward it. */
        uint32_t elapsed = in->now_ms - c->straddle_outside_ms;
        INT arc = 0;

        if (elapsed >= c->straddle_coast_ms) {
            arc = c->straddle_arc_start + VC_STRADDLE_RECOVER_RAMP *
                (INT)((elapsed - c->straddle_coast_ms) / VC_PERIOD_MS);
            if (arc > VC_STRADDLE_OUTSIDE_STEER) arc = VC_STRADDLE_OUTSIDE_STEER;
        }
        command.steering_permille = c->straddle_outside * arc;
    } else if (c->line_straddle && c->straddle_touch != 0 &&
               !B3_LINE_LEFT_ACTIVE(in->left) &&
               !B3_LINE_CENTRE_ACTIVE(in->centre) &&
               in->now_ms - c->straddle_black_ms >= VC_STRADDLE_SEEK_MS) {
        /* Both on floor for a while: drift toward the last touched side. */
        INT seek = VC_STRADDLE_SEEK_START + VC_STRADDLE_SEEK_RAMP *
            (INT)((in->now_ms - c->straddle_black_ms - VC_STRADDLE_SEEK_MS) /
                  VC_PERIOD_MS);
        if (seek > VC_STRADDLE_SEEK_MAX) seek = VC_STRADDLE_SEEK_MAX;
        command.steering_permille = c->straddle_touch * seek;
    }
    if (c->line_straddle && in->barcode_active) {
        if (command.steering_permille > VC_BARCODE_MAX_STEER) {
            command.steering_permille = VC_BARCODE_MAX_STEER;
        } else if (command.steering_permille < -VC_BARCODE_MAX_STEER) {
            command.steering_permille = -VC_BARCODE_MAX_STEER;
        }
    }
    /* Smooth steering changes so the car does not jerk between corrections. */
    if (c->drive_sent) {
        INT previous = c->last_steer;
        INT requested = command.steering_permille;
        INT previous_mag = previous < 0 ? -previous : previous;
        INT requested_mag = requested < 0 ? -requested : requested;
        bool same_side = (requested >= 0) == (previous >= 0);
        /* Turning harder ramps at the SLEW rate.  Easing off, or crossing
         * to the other side, ramps at the faster RELEASE rate: quick enough
         * that old steering does not carry the car across the line, but no
         * longer a one-tick snap from full turn to straight, which jerked
         * the car on every correction. */
        INT rate = (same_side && requested_mag > previous_mag) ?
            VC_STEER_SLEW_PER_TICK : VC_STEER_RELEASE_PER_TICK;
        if (requested > previous + rate) {
            command.steering_permille = previous + rate;
        } else if (requested < previous - rate) {
            command.steering_permille = previous - rate;
        }
    }
    c->last_steer = command.steering_permille;
    c->follow_steer = command.steering_permille;
    {
        INT magnitude = command.steering_permille < 0 ?
            -command.steering_permille : command.steering_permille;
        INT slow_permille;

        /* Look-ahead: remember the hardest recent correction and only
         * relax it gradually, so speed stays low through the curve. */
        c->curve_steer -= VC_CURVE_DECAY_PER_TICK;
        if (c->curve_steer < magnitude) c->curve_steer = magnitude;
        if (c->curve_steer < 0) c->curve_steer = 0;
        slow_permille = 1000 - (1000 - VC_CURVE_MIN_PERMILLE) *
            c->curve_steer / B3_LINE_STEER_MAX;
        if (slow_permille < VC_CURVE_MIN_PERMILLE) {
            slow_permille = VC_CURVE_MIN_PERMILLE;
        }
        speed = speed * slow_permille / 1000;
    }
    if ((in->line_flags & LINE_F_JUNCTION) != 0u || c->junction_latched) {
        command.target_speed_cps = speed / 2;
    } else {
        if (c->mode == MODE_AUTO &&
            c->pending_navigation != B3_NAV_NONE) {
            speed = speed * 3 / 4;
        }
        command.target_speed_cps = speed;
    }
    if (command.target_speed_cps < 1) command.target_speed_cps = 1;
    /* Constant speed across a barcode.  Bar widths are timed, so a speed
     * change part-way through the strip (the junction or curve slowdown, or
     * braking at the first bar) stretches the later elements and the read
     * is rejected.  Keep the speed the car had when the first bar arrived
     * until the bars stop. */
    if (in->barcode_active && c->drive_sent && c->last_speed > 0) {
        command.target_speed_cps = c->last_speed;
    }
    /* Gentle acceleration, also from standstill; slowing applies at once. */
    {
        INT previous = c->drive_sent ? c->last_speed : 0;
        if (command.target_speed_cps > previous + VC_SPEED_RISE_PER_TICK) {
            command.target_speed_cps = previous + VC_SPEED_RISE_PER_TICK;
        }
    }
    c->last_speed = command.target_speed_cps;
    /* Preserve B2's outer-wheel <=5000 cps constraint in either direction.
     * Only scale when the outer wheel would actually exceed it, so low
     * speeds keep enough inner-wheel speed to avoid B2 stall detection. */
    {
        INT steering_magnitude = command.steering_permille < 0 ?
            -command.steering_permille : command.steering_permille;
        if ((int64_t)command.target_speed_cps * (1000 + steering_magnitude) >
            5000000) {
            command.target_speed_cps = 5000000 / (1000 + steering_magnitude);
        }
    }
    if (in->hump) command.target_speed_cps /= 2;
    {
        INT steer_mag = command.steering_permille < 0 ?
            -command.steering_permille : command.steering_permille;
        /* Pivot only when one sensor has held the line long enough that
         * the car clearly cannot turn as tight as the bend. */
        bool held = c->line_straddle && c->straddle_side != 0 &&
            in->now_ms - c->straddle_side_ms >= VC_STRADDLE_HELD_PIVOT_MS;
        bool pivot = held && !in->barcode_active &&
            steer_mag >= VC_STRADDLE_PIVOT_STEER;
        if (c->line_straddle) {
            straddle_wheels(&command, pivot);
        } else {
            apply_wheel_minimum(&command, pivot);
        }
    }
    command.safety_generation = in->safety_generation;
    if (motion_send_command(&command) == E_OK) {
#if VC_STEERING_DIAGNOSTICS
        report_steering(c, in, &command);
#endif
        c->stopped = false;
        c->drive_sent = true;
        c->last_drive_ms = in->now_ms;
    } else if (!c->drive_sent || in->now_ms - c->last_drive_ms >= 60u) {
        safe_stop(c);
    }
}

#if VC_STEERING_DIAGNOSTICS
/* Log only accepted commands; measured speeds describe the preceding motion,
 * not the response to the command just submitted. Physical left/right labels
 * differ from B3's historical ADC variable names.
 */
static void report_steering(VEHICLE_CONTROLLER *c, const VC_INPUT *in,
                            const motion_command_t *command)
{
    if (in->now_ms - c->last_steering_log_ms < VC_STEERING_LOG_MS) {
        return;
    }
    c->last_steering_log_ms = in->now_ms;
    tm_printf((UB *)"[VC-STEER] t=%u state=%d R26=%d L27=%d flags=%u "
                    "steer=%d base=%d measL=%d measR=%d age=%u\n",
              (UW)in->now_ms, (INT)c->state,
              (INT)in->left, (INT)in->centre,
              (UINT)in->line_flags, command->steering_permille,
              command->target_speed_cps, in->motion.left_speed_cps,
              in->motion.right_speed_cps,
              (UW)(in->now_ms - in->motion.sample_time_ms));
}
#endif

static void controller_task(INT stacd, void *exinf)
{
    VEHICLE_CONTROLLER c;
    imu_notice_t notice = {0};
    imu_event_t imu = {0};
    bool hazard = false;
    bool autostart_pending = ROBOT_AUTOSTART != 0;
    uint32_t last_boot_log_ms = 0u;
#if VC_FOLLOW_V4
    uint32_t lift_imu_since = 0u;
    float rest_az = 0.0f;
#endif
    (void)stacd;
    (void)exinf;
    vehicle_controller_init(&c);
    for (;;) {
        SYSTIM now;
        VC_INPUT in = {0};
        B3_STATE line;
        uint32_t line_ms = 0u;
        UB nav = 0u;
        bool stop_barrier = false;
        bool was_running = c.running;
        if (tk_get_tim(&now) != E_OK) {
            safe_stop(&c);
            tk_dly_tsk(VC_PERIOD_MS);
            continue;
        }
        in.now_ms = now.lo;
        /* Bounded drains keep network/sensor bursts below the DRIVE budget. */
        for (UINT i = 0u; i < IMU_VC_QUEUE_DEPTH; ++i) {
            if (imu_vc_receive(&notice, TMO_POL) != E_OK) break;
            imu = notice.snapshot;
            if (notice.events & (IMU_NOTICE_IMPACT | IMU_NOTICE_FAULT |
                                 IMU_NOTICE_UNAVAILABLE)) hazard = true;
        }
        for (UINT i = 0u; i < REMOTE_CMD_QUEUE_LEN; ++i) {
            REMOTE_CMD remote;
            if (tk_rcv_mbf(mbf_remote_cmd, &remote, TMO_POL) != (INT)sizeof(remote)) break;
            bool halt_command = remote.type == RCMD_STOP || remote.type == RCMD_ESTOP ||
                remote.type == RCMD_PAUSE || remote.type == RCMD_MODE;
            if (halt_command || remote.type == RCMD_START) autostart_pending = false;
            if (!stop_barrier || halt_command || remote.type == RCMD_RESET_PEAK) {
                vehicle_controller_command(&c, &remote);
            }
            if (halt_command) stop_barrier = true;
        }
        for (UINT i = 0u; i < B3_NAV_QUEUE_LENGTH; ++i) {
            UB next;
            if (tk_rcv_mbf(mbf_nav_cmd, &next, TMO_POL) != (INT)sizeof(next)) break;
            if (nav == 0u && was_running && c.running && !stop_barrier) nav = next;
        }
        in.line_valid = b3_get_timed_state(&line, &line_ms) == E_OK;
        in.motion_valid = motion_get_status(&in.motion) == E_OK;
        /* Read time after producer snapshots, including mutex waits. A
         * freshly published sample must never look UINT32_MAX ms old. */
        if (tk_get_tim(&now) != E_OK) {
            safe_stop(&c);
            tk_dly_tsk(VC_PERIOD_MS);
            continue;
        }
        in.now_ms = now.lo;
        in.line_valid = in.line_valid && now.lo - line_ms <= VC_LINE_MAX_AGE_MS;
        if (in.line_valid) {
            in.left = (UH)line.ir_left;
            in.centre = (UH)line.ir_centre;
            in.line_flags = (UB)line.line_flags;
            in.right = (UB)line.ir_right;
        }
        {
            uint16_t r26 = 0u, r27 = 0u;
            b3_line_raw(&r26, &r27);
            in.raw26 = r26;
            in.raw27 = r27;
        }
#if VC_FOLLOW_V4
        {
            imu_live_t live;
            imu_live_get(&live);
            if (live.b_valid && now.lo - live.timestamp_ms <= 100u) {
                float ax = live.accel_x, ay = live.accel_y, az = live.accel_z;
                float a2 = ax * ax + ay * ay + az * az;
                float g = 9.81f;
                float lo = (g - VC4_LIFT_DG_MS2) * (g - VC4_LIFT_DG_MS2);
                float hi = (g + VC4_LIFT_DG_MS2) * (g + VC4_LIFT_DG_MS2);
                /* The tilt rule needs the IMU's z axis vertical: learn it
                 * while standing (not running); off if z is not vertical. */
                bool tilt_rule = rest_az > 7.0f || rest_az < -7.0f;
                bool odd = (tilt_rule && az < VC4_LIFT_TILT_MS2 &&
                            az > -VC4_LIFT_TILT_MS2) || a2 < lo || a2 > hi;
                if (!c.running) rest_az = az;
                in.pitch_deg = live.pitch_deg;
                if (odd) {
                    if (lift_imu_since == 0u) lift_imu_since = now.lo | 1u;
                } else {
                    lift_imu_since = 0u;
                }
                in.lifted_imu = lift_imu_since != 0u && !imu.b_hump_detected &&
                    now.lo - lift_imu_since >= VC4_LIFT_IMU_MS;
            } else {
                lift_imu_since = 0u;
            }
        }
#endif
        in.barcode_active = b3_barcode_active();
        in.barcode_rejects = (UH)b3_barcode_rejects();
        in.imu_valid = imu.b_valid && now.lo - imu.timestamp_ms <= VC_IMU_MAX_AGE_MS;
        in.hump = imu.b_hump_detected;
        in.hazard = hazard || imu.b_sudden_impact ||
            imu.roll_deg > 25.0f || imu.roll_deg < -25.0f ||
            imu.pitch_deg > 40.0f || imu.pitch_deg < -40.0f;
        in.motion_valid = in.motion_valid &&
            now.lo - in.motion.sample_time_ms <= VC_MOTION_MAX_AGE_MS;
        in.clearance = b2_safety_motion_allowed();
        in.approach_slow = s5_mission_approach_slow();
        in.safety_generation = b2_safety_stop_generation();
        in.faults = fault_get();
        {
            S5MissionState obstacle = s5_mission_state();
            in.obstacle_active = obstacle == S5_MISSION_BUSY;
            in.obstacle_fault = obstacle == S5_MISSION_FAULT;
        }
        /* Higher-priority VC can run before B2/B3 publish their first
         * sample. Arm boot motion only after producers are actually ready. */
        bool boot_sensors_ready = in.line_valid && in.imu_valid &&
            in.motion_valid && !in.hazard && !in.obstacle_fault &&
            !(in.faults & (FAULT_MOTOR_STALL | FAULT_ENCODER | FAULT_IMU));
        /* Start the normal bounded line search if fresh sensors see white;
         * requiring black here prevents the robot from ever finding tape.
         * B5 needs an enabled mission to establish line reference and
         * clearance. Keep boot arming pending until it actually permits
         * DRIVE; a transient fault during warmup must not consume START. */
        if (autostart_pending && boot_sensors_ready && in.clearance &&
            !(in.faults & FAULT_NO_ECHO)) {
            REMOTE_CMD start = {0};
            start.type = RCMD_START;
            vehicle_controller_command(&c, &start);
            autostart_pending = false;
            tm_printf((UB *)"[VC-BOOT] sensors and front clearance ready; START\n");
        } else if (autostart_pending && in.now_ms - last_boot_log_ms >= 1000u) {
            last_boot_log_ms = in.now_ms;
            tm_printf((UB *)"[VC-BOOT-WAIT] line=%d detected=%d imu=%d motion=%d hazard=%d obstacle_fault=%d clearance=%d faults=%d\n",
                      (INT)in.line_valid, (INT)((in.line_flags & LINE_F_DETECTED) != 0u),
                      (INT)in.imu_valid, (INT)in.motion_valid, (INT)in.hazard,
                      (INT)in.obstacle_fault, (INT)in.clearance, (INT)in.faults);
        }
        vehicle_controller_step(&c, &in, nav);
        if (c.running != was_running) {
            run_log_event(in.now_ms, c.running ? RUN_EV_START : RUN_EV_STOP,
                          (uint16_t)c.state);
        }
        if (c.running) {
            RUN_TICK tick = {0};
            tick.t_ms = in.now_ms;
            tick.travel_mm = c.travel_um / 1000u;
            tick.n26 = in.left;
            tick.n27 = in.centre;
            tick.steer = (int16_t)c.last_steer;
            tick.speed = (int16_t)c.last_speed;
            tick.curve = (int16_t)c.curve_steer;
            tick.state = (uint8_t)c.state;
            tick.pending = c.pending_navigation;
            tick.nav = c.navigation;
            tick.bits = (uint16_t)(
                (in.right ? RUN_BIT_GP28 : 0u) |
                ((in.line_flags & LINE_F_DETECTED) ? RUN_BIT_DETECTED : 0u) |
                ((in.line_flags & LINE_F_LOST) ? RUN_BIT_LOST : 0u) |
                ((in.line_flags & LINE_F_JUNCTION) ? RUN_BIT_JFLAG : 0u) |
                (c.tape_now ? RUN_BIT_TAPE : 0u) |
                (c.junction_latched ? RUN_BIT_JLATCH : 0u) |
                (in.barcode_active ? RUN_BIT_BARCODE : 0u) |
                (c.floor_known ? RUN_BIT_FLOOR : 0u) |
                (c.line_straddle ? RUN_BIT_STRADDLE : 0u));
            run_log_tick(&tick);
        }
        s5_mission_enable(((autostart_pending && boot_sensors_ready) || c.running) && !c.estop && !c.paused &&
                          c.mode != MODE_TEST && c.speed_percent != 0u);
        /* Hazards stop this tick even when coalesced with recovery. A later
         * explicit START is required; no automatic mission restart. */
        hazard = false;
        tk_dly_tsk(VC_PERIOD_MS);
    }
}

ER vehicle_controller_start(void)
{
    ID task = app_start_task((FP)controller_task, 6, TP_PRC1, "vehicle_controller");
    if (task < E_OK) return task;
    return (ER)run_log_start();
}
