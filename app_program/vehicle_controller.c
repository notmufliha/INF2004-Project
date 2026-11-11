#include "vehicle_controller.h"
#include "robot_config.h"
#include "subsystem2/b2_motor.h"
#include "subsystem3/b3_line_barcode.h"
#include "subsystem4/imu_vc.h"
#include "subsystem4/subsystem4.h"
#include "subsystem5/s5_mission.h"

#if ROBOT_MODE && (ROBOT_MOTION_TEST || ROBOT_WHEELS_UP_MOTOR_TEST || S1_STANDALONE_DEMO)
#error "The Vehicle Controller requires exclusive command ownership; disable demos/tests"
#endif

static void set_state(VEHICLE_CONTROLLER *c, ROBOT_STATE state)
{
    if (c->state != state) {
        c->state = state;
        comm_set_robot_state(state);
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
    c->filtered_line_valid = false;
}

static void safe_stop(VEHICLE_CONTROLLER *c)
{
    b2_request_stop(); /* Revoke B5 movement as well as VC movement. */
    c->running = false;
    c->paused = false;
    c->navigation = 0u;
    c->navigation_id = 0u;
    c->pending_navigation = B3_NAV_NONE;
    c->junction_enter_samples = 0u;
    c->junction_clear_samples = 0u;
    c->junction_latched = false;
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

    /* IIR on the sensor difference: ignore isolated ADC jumps while
     * retaining strong corrections on sustained edges. Re-prime after STOP
     * and recovery instead of letting old steering leak into a new run. */
    if (!c->filtered_line_valid || !c->drive_sent) {
        c->filtered_line_error = error;
        c->filtered_line_valid = true;
    } else {
        c->filtered_line_error +=
            (error - c->filtered_line_error) / VC_LINE_ERROR_FILTER_DIV;
    }
    magnitude = c->filtered_line_error < 0 ?
        -c->filtered_line_error : c->filtered_line_error;

    (void)line_position(c, in); /* keep the last-seen side for recovery */
    if (magnitude > B3_LINE_STEER_DEADBAND) {
        magnitude -= B3_LINE_STEER_DEADBAND;
        if (magnitude > B3_LINE_STEER_ERROR_FULL_SCALE -
                        B3_LINE_STEER_DEADBAND) {
            magnitude = B3_LINE_STEER_ERROR_FULL_SCALE -
                        B3_LINE_STEER_DEADBAND;
        }
        steering = magnitude * B3_LINE_STEER_MAX /
            (B3_LINE_STEER_ERROR_FULL_SCALE - B3_LINE_STEER_DEADBAND);
        if (c->filtered_line_error < 0) steering = -steering;
    }
    c->last_error = error;
    return steering;
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
 * B2 drives left = v(1+s), right = v(1-s) with s in permille. */
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
    if (magnitude <= 200) {
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
    if (allow_pivot && inner < (int64_t)VC_MIN_WHEEL_CPS * 500) {
        inner = 0; /* deliberate lost-line recovery may pivot */
    } else {
        /* During normal line following, do not abruptly cut the inner
         * wheel to zero: the sharp moving->pivot transition is jerky. */
        inner = VC_MIN_WHEEL_CPS;
    }
    if (outer <= inner) {
        /* Normal tracking should never pivot merely because speed is low.
         * Only the explicit recovery manoeuvre may pivot in place. */
        if (allow_pivot) inner = 0;
        else outer = inner;
    }
    command->target_speed_cps = (INT)((outer + inner) / 2);
    magnitude = (INT)((outer - inner) * 1000 / (outer + inner));
    command->steering_permille = steering < 0 ? -magnitude : magnitude;
}

static void update_junction_state(VEHICLE_CONTROLLER *c, UB line_flags)
{
    bool junction = (line_flags & LINE_F_JUNCTION) != 0u &&
        (line_flags & LINE_F_DETECTED) != 0u &&
        (line_flags & LINE_F_LOST) == 0u;

    if (junction) {
        c->junction_clear_samples = 0u;
        if (c->junction_enter_samples < VC_JUNCTION_CONFIRM_SAMPLES) {
            c->junction_enter_samples++;
        }
        if (c->junction_enter_samples >= VC_JUNCTION_CONFIRM_SAMPLES) {
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
    c->state = ROBOT_INIT;
    stop_motion(c);
    set_state(c, ROBOT_IDLE);
}

void vehicle_controller_command(VEHICLE_CONTROLLER *c, const REMOTE_CMD *cmd)
{
    switch (cmd->type) {
    case RCMD_START:
        if (c->mode != MODE_TEST) {
            c->estop = false; /* Only an explicit START can release ESTOP. */
            c->running = true;
            c->paused = false;
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
    if (!c->running || c->estop || c->mode == MODE_TEST) {
        stop_motion(c);
        return;
    }
    if (!in->motion_valid || !in->imu_valid || in->hazard || in->obstacle_fault ||
        (in->faults & (FAULT_MOTOR_STALL | FAULT_ENCODER | FAULT_IMU)) ||
        in->motion.status == MOTION_STATUS_FAULT ||
        in->motion.status == MOTION_STATUS_TIMEOUT) {
        safe_stop(c);
        return;
    }
    if (c->speed_percent == 0u) {
        stop_motion(c);
        set_state(c, ROBOT_IDLE);
        return;
    }
    if (!in->line_valid) {
        safe_stop(c);
        return;
    }
    if (in->obstacle_active) {
        /* B5 owns bounded bypass/recovery legs; never overwrite them. */
        c->drive_sent = false;
        c->stopped = false;
        c->navigation = 0u;
        c->navigation_id = 0u;
        c->pending_navigation = B3_NAV_NONE;
        c->junction_latched = false;
        c->junction_enter_samples = c->junction_clear_samples = 0u;
        return;
    }
    if (!in->clearance || (in->faults & FAULT_NO_ECHO)) {
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
    (void)motion_speed_from_percent((INT)c->speed_percent, &speed);
    line_detected = ((in->line_flags & LINE_F_DETECTED) != 0u) &&
        ((in->line_flags & LINE_F_LOST) == 0u);
    update_junction_state(c, in->line_flags);
    if (c->mode == MODE_AUTO && c->navigation == 0u &&
        c->pending_navigation == B3_NAV_NONE &&
        nav >= B3_NAV_LEFT && nav <= B3_NAV_UTURN) {
        c->pending_navigation = nav;
    }
    if (c->mode == MODE_AUTO && c->navigation == 0u &&
        c->pending_navigation != B3_NAV_NONE && c->junction_latched) {
        c->navigation = c->pending_navigation;
        c->pending_navigation = B3_NAV_NONE;
        c->navigation_ms = in->now_ms;
        stop_motion(c);
        set_state(c, ROBOT_NAV_COMMAND);
        return;
    }
    if (c->navigation == 0u && line_detected) {
        (void)line_position(c, in); /* remember which side the line is on */
    }
    if (c->navigation == 0u && !line_detected) {
        uint32_t recovery_elapsed;
        INT direction;
        INT search_steer = VC_LINE_SEARCH_STEER;

        if (c->state != ROBOT_LINE_RECOVERY) c->recovery_ms = in->now_ms;
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
        c->filtered_line_valid = false;
        c->last_error = direction * B3_LINE_POS_CENTRE;
        apply_wheel_minimum(&command, true);
        command.safety_generation = in->safety_generation;
        if (motion_send_command(&command) == E_OK) {
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
                c->navigation = 0u;
                c->navigation_id = 0u;
            } else if (in->now_ms - c->navigation_ms > 65000u) {
                safe_stop(c);
            }
            return;
        }
        if (in->now_ms - c->navigation_ms > VC_RECOVERY_MS) {
            safe_stop(c);
            return;
        }
        if (!in->motion.physically_stopped) return;
        ER error;
        if (c->navigation == B3_NAV_STRAIGHT) {
            error = motion_move_cm(10.0f, speed, MOTION_DIR_FORWARD, &c->navigation_id);
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
    /* Smooth steering changes so the car does not jerk between corrections. */
    if (c->drive_sent) {
        if (command.steering_permille > c->last_steer + VC_STEER_SLEW_PER_TICK) {
            command.steering_permille = c->last_steer + VC_STEER_SLEW_PER_TICK;
        } else if (command.steering_permille <
                   c->last_steer - VC_STEER_SLEW_PER_TICK) {
            command.steering_permille = c->last_steer - VC_STEER_SLEW_PER_TICK;
        }
    }
    c->last_steer = command.steering_permille;
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
    /* Steady and slow across a barcode (slowing applies at once). */
    if (in->barcode_active && command.target_speed_cps > VC_BARCODE_CPS) {
        command.target_speed_cps = VC_BARCODE_CPS;
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
    apply_wheel_minimum(&command, false);
    /* Slew from the steering actually requested of B2, not the intermediate
     * pre-wheel-floor value. */
    c->last_steer = command.steering_permille;
    command.safety_generation = in->safety_generation;
    if (motion_send_command(&command) == E_OK) {
        c->stopped = false;
        c->drive_sent = true;
        c->last_drive_ms = in->now_ms;
    } else if (!c->drive_sent || in->now_ms - c->last_drive_ms >= 60u) {
        safe_stop(c);
    }
}

static void controller_task(INT stacd, void *exinf)
{
    VEHICLE_CONTROLLER c;
    imu_notice_t notice = {0};
    imu_event_t imu = {0};
    bool hazard = false;
    bool autostart_pending = ROBOT_AUTOSTART != 0;
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
        }
        in.barcode_active = b3_barcode_active();
        in.imu_valid = imu.b_valid && now.lo - imu.timestamp_ms <= VC_IMU_MAX_AGE_MS;
        in.hump = imu.b_hump_detected;
        in.hazard = hazard || imu.b_sudden_impact ||
            imu.roll_deg > 25.0f || imu.roll_deg < -25.0f ||
            imu.pitch_deg > 40.0f || imu.pitch_deg < -40.0f;
        in.motion_valid = in.motion_valid &&
            now.lo - in.motion.sample_time_ms <= VC_MOTION_MAX_AGE_MS;
        in.clearance = b2_safety_motion_allowed();
        in.safety_generation = b2_safety_stop_generation();
        in.faults = fault_get();
        {
            S5MissionState obstacle = s5_mission_state();
            in.obstacle_active = obstacle == S5_MISSION_BUSY;
            in.obstacle_fault = obstacle == S5_MISSION_FAULT;
        }
        /* Higher-priority VC can run before B2/B3 publish their first
         * sample. Arm boot motion only after producers are actually ready. */
        if (autostart_pending && in.line_valid && in.imu_valid &&
            in.motion_valid && !in.hazard && !in.obstacle_fault &&
            (in.line_flags & LINE_F_DETECTED) &&
            !(in.faults & (FAULT_MOTOR_STALL | FAULT_ENCODER | FAULT_IMU))) {
            REMOTE_CMD start = {0};
            start.type = RCMD_START;
            vehicle_controller_command(&c, &start);
            autostart_pending = false;
        }
        vehicle_controller_step(&c, &in, nav);
        s5_mission_enable(c.running && !c.estop && !c.paused &&
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
    return task < E_OK ? task : E_OK;
}
