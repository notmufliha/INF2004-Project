#include "vehicle_controller.h"
#include "robot_config.h"
#include "subsystem2/b2_motor.h"
#include "subsystem3/b3_line_barcode.h"
#include "subsystem4/imu_vc.h"
#include "subsystem4/subsystem4.h"
#include "subsystem5/s5_mission.h"
#include "run_log.h"
#include "subsystem3/b3_buzzer.h"

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

void vehicle_controller_init(VEHICLE_CONTROLLER *c)
{
    *c = (VEHICLE_CONTROLLER){0};
    c->line_pos = B3_LINE_POS_CENTRE;
    c->mode = MODE_AUTO;
    c->speed_percent = 20u;
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
        c->pending_navigation = B3_NAV_NONE;
        c->junction_latched = false;
        c->junction_enter_samples = c->junction_clear_samples = 0u;
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
    if (in->approach_slow && speed > ROBOT_BYPASS_SPEED_CPS) speed = ROBOT_BYPASS_SPEED_CPS;
    if (c->line_straddle) {
        /* Standing still (START, after a turn, a stall retry or a
         * clearance wait) is not "no line seen for a while". */
        if (!c->drive_sent && !c->line_recovery_active) {
            c->straddle_black_ms = in->now_ms;
        }
        line_detected = straddle_following(c, in);
    } else {
        line_detected = ((in->line_flags & LINE_F_DETECTED) != 0u) &&
            ((in->line_flags & LINE_F_LOST) == 0u);
    }
    if (line_detected) c->line_recovery_active = false;
    if (c->rescan_phase != 0u || c->navigation != 0u ||
        c->line_recovery_active || !c->drive_sent) {
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
        }
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
        uint32_t recovery_elapsed;
        INT direction;
        INT search_steer = VC_LINE_SEARCH_STEER;

        if (!c->line_recovery_active) {
            c->recovery_ms = in->now_ms;
            c->line_recovery_active = true;
            run_log_event(in->now_ms, RUN_EV_LINE_LOST, 0u);
        }
        recovery_elapsed = in->now_ms - c->recovery_ms;
        set_state(c, ROBOT_LINE_RECOVERY);
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
            error = motion_turn_degrees(360, speed, MOTION_TURN_RIGHT,
                &c->navigation_id);
        } else {
            error = motion_turn_degrees(c->navigation == B3_NAV_UTURN ? 180 : 90,
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
    if (c->drive_sent && in->now_ms - c->last_drive_ms < VC_PERIOD_MS) return;
    command.type = MOTION_CMD_DRIVE;
    command.direction = MOTION_DIR_FORWARD;
    /* PD on the line position: balanced sensors give position 500 and
     * zero steering, so a junction is still crossed straight, while a curve
     * is followed even when both sensors see the line. */
    command.steering_permille = line_follow_steering(c, in);
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
        apply_wheel_minimum(&command, held && !in->barcode_active &&
                            steer_mag >= VC_STRADDLE_PIVOT_STEER);
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
            if (!stop_barrier || halt_command) vehicle_controller_command(&c, &remote);
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
