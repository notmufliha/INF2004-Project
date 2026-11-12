#include "subsystem5.h"
#if S5_MOTION_BRIDGE_ENABLED
#include "s5_navigation.h"
#include "s5_maneuver.h"
#include "s5_bridge.h"
#include "../subsystem2/b2_motor.h"
#include "../subsystem2/motion_command.h"
#include "../subsystem3/b3_line_barcode.h"
#include "../subsystem1/subsystem1.h"

typedef enum { NAV_FOLLOW, NAV_STOP, NAV_AVOID, NAV_RECOVER,
               NAV_REVERSE, NAV_FAULT, NAV_PROBE } NavigationMode;
static NavigationMode mode;
static S5Maneuver maneuver;
static bool initialized;
static bool stop_queued;
static unsigned clear_hits;
static unsigned invalid_front_hits;
static uint32_t last_clear_ms;
static uint32_t last_turn_guard_log_ms;
static unsigned initial_line_hits;
static uint32_t initial_line_sample;
static bool line_reference;
static bool (*rear_sensor)(float);
static S5Error navigation_error;
static uint32_t probe_id, probe_started_ms;
static bool probe_complete;
static int probe_degrees;

static bool now_ms(uint32_t *now)
{
    SYSTIM time;
    if (tk_get_tim(&time) != E_OK) return false;
    *now = time.lo;
    return true;
}
static bool stop_motor(void)
{
    motion_command_t stop = {0};
    b2_request_stop();
    if (!stop_queued) {
        stop.type = MOTION_CMD_STOP;
        stop_queued = motion_send_command(&stop) == E_OK;
    }
    return stop_queued;
}
static void fault_stop(S5Error error)
{
    navigation_error = error;
    tm_printf((UB*)"[S5-NAV-FAULT] error=%d; movement stopped\n", (int)error);
    mode = NAV_FAULT;
    s5_maneuver_abort(&maneuver, error);
    (void)stop_motor();
    comm_set_robot_state(ROBOT_SAFE_STOP);
}
static bool ready(void)
{
    return initialized && mode != NAV_FAULT;
}
static bool request(S5Command command, const S5Profile *profile)
{
    uint32_t now;
    if (command == S5_CMD_STOP || command == S5_CMD_HALT) {
        clear_hits = 0u;
        invalid_front_hits = 0u;
        mode = command == S5_CMD_HALT ? NAV_FAULT : NAV_STOP;
        s5_maneuver_init(&maneuver);
        comm_set_robot_state(command == S5_CMD_HALT ? ROBOT_SAFE_STOP : ROBOT_OBSTACLE_SCAN);
        return stop_motor();
    }
    if (!ready() || !now_ms(&now)) return false;
    if (command == S5_CMD_FIND_LINE) {
        if (mode != NAV_AVOID || maneuver.state != S5_MAN_TURN_IN)
            return false;
        mode = NAV_RECOVER;
        comm_set_robot_state(ROBOT_LINE_RECOVERY);
        return true;
    }
    if (command == S5_CMD_RESUME) {
        bool recovered = mode == NAV_RECOVER && maneuver.state == S5_MAN_COMPLETE;
        bool obstacle_removed = mode == NAV_STOP && maneuver.state == S5_MAN_IDLE &&
                                s5_navigation_stopped();
        if (!recovered && !obstacle_removed)
            return false;
        mode = NAV_FOLLOW;
        clear_hits = 0u;
        s5_maneuver_init(&maneuver);
        comm_set_robot_state(ROBOT_LINE_FOLLOW);
        return stop_motor();
    }
    if (mode != NAV_STOP && !(mode == NAV_REVERSE && maneuver.state == S5_MAN_COMPLETE))
        return false;
    if (command == S5_CMD_REVERSE &&
        (!rear_sensor || !rear_sensor(ROBOT_BYPASS_REVERSE_CM))) return false;
    s5_maneuver_init(&maneuver);
    if (!s5_maneuver_begin(&maneuver, command, profile,
                           ROBOT_BYPASS_MAX_DEPTH_CM, now)) return false;
    tm_printf((UB*)"[S5-ROUTE] side=%d offset_cm_x100=%d pass_cm_x100=%d search_limit_cm_x100=%d course_bounds=%d\n",
              (int)maneuver.side, (int)(maneuver.offset_cm * 100.0f),
              (int)(maneuver.pass_cm * 100.0f), (int)(maneuver.search_limit_cm * 100.0f),
              profile ? (int)profile->uses_course_bounds : 0);
    mode = command == S5_CMD_REVERSE ? NAV_REVERSE : NAV_AVOID;
    comm_set_robot_state(ROBOT_OBSTACLE_AVOID);
    return true;
}
static bool motion_complete(void)
{
    return (mode == NAV_AVOID && maneuver.state == S5_MAN_TURN_IN &&
            maneuver.pending_id == 0u) ||
           (mode == NAV_REVERSE && maneuver.state == S5_MAN_COMPLETE);
}
static bool line_found(void)
{
    return mode == NAV_RECOVER && maneuver.state == S5_MAN_COMPLETE;
}
static void emergency_stop(void)
{
    fault_stop(navigation_error == S5_ERR_NONE ? S5_ERR_MOTION_TIMEOUT : navigation_error);
}
bool s5_navigation_set_rear_sensor(bool (*sensor)(float))
{
    if (initialized || !sensor || rear_sensor) return false;
    rear_sensor = sensor;
    return true;
}
bool s5_navigation_init(void)
{
    const S5BridgeDriver driver = {
        ready, request, motion_complete, line_found, emergency_stop
    };
    if (initialized) return false;
    s5_maneuver_init(&maneuver);
    mode = NAV_FOLLOW;
    navigation_error = S5_ERR_NONE;
    stop_queued = false;
    clear_hits = initial_line_hits = 0u;
    invalid_front_hits = 0u;
    last_clear_ms = 0u;
    last_turn_guard_log_ms = 0u;
    initial_line_sample = 0u;
    line_reference = false;
    if (!s5_bridge_install(&driver)) return false;
    initialized = true;
    return true;
}
void s5_navigation_pause(void)
{
    (void)stop_motor();
    probe_id = 0u;
    probe_complete = false;
    if (mode == NAV_FAULT) return;
    mode = NAV_FOLLOW;
    clear_hits = 0u;
    invalid_front_hits = 0u;
    s5_maneuver_init(&maneuver);
}
S5Error s5_navigation_error(void) { return navigation_error; }
bool s5_navigation_has_line_reference(void) { return line_reference; }
bool s5_navigation_probe_complete(void) { return probe_complete; }
bool s5_navigation_probe_begin(int degrees)
{
    uint32_t now, id = 0u;
    if (!ready() || mode != NAV_STOP || !s5_navigation_stopped() ||
        !now_ms(&now) || degrees == 0 || degrees < -120 || degrees > 120) return false;
    b2_request_forward();
    if (!b2_safety_motion_allowed() || motion_turn_degrees(degrees < 0 ? -degrees : degrees,
        ROBOT_BYPASS_SPEED_CPS, degrees < 0 ? MOTION_TURN_LEFT : MOTION_TURN_RIGHT, &id) != E_OK || !id)
        return false;
    probe_id = id;
    probe_degrees = degrees;
    probe_started_ms = now;
    probe_complete = false;
    stop_queued = false;
    mode = NAV_PROBE;
    tm_printf((UB*)"[S5-LOOK] turn_deg=%d id=%d\n", degrees, (int)id);
    return true;
}
bool s5_navigation_stopped(void)
{
    uint32_t now;
    motion_status_snapshot_t status;
    return motion_get_status(&status) == E_OK && now_ms(&now) &&
           now - status.sample_time_ms <= 200u && status.physically_stopped &&
           !b2_safety_stop_requested() && b2_motor_is_stopped();
}
bool s5_navigation_probe_bypass(const S5Profile *profile)
{
    uint32_t now;
    if (!profile || !profile->valid || mode != NAV_STOP || !probe_complete ||
        !s5_navigation_stopped() || !now_ms(&now)) return false;
    int expected = profile->bypass_side == S5_SIDE_LEFT ? -ROBOT_BYPASS_TURN_DEG : ROBOT_BYPASS_TURN_DEG;
    if (profile->bypass_side == S5_SIDE_NONE || probe_degrees != expected) return false;
    s5_maneuver_init(&maneuver);
    if (!s5_maneuver_begin(&maneuver,
        profile->bypass_side == S5_SIDE_LEFT ? S5_CMD_GO_LEFT : S5_CMD_GO_RIGHT,
        profile, ROBOT_BYPASS_MAX_DEPTH_CM, now)) return false;
    /* The viewpoint turn already completed. Start the diagonal offset,
     * rather than restoring heading and submitting another outward turn. */
    maneuver.state = S5_MAN_OFFSET;
    probe_complete = false;
    mode = NAV_AVOID;
    tm_printf((UB*)"[S5-LOOK-ADVANCE] side=%d offset_cm_x100=%d pass_cm_x100=%d\n",
        (int)maneuver.side, (int)(maneuver.offset_cm * 100.0f), (int)(maneuver.pass_cm * 100.0f));
    return true;
}

void s5_navigation_tick(bool front_valid, float front_cm, bool follow)
{
    S5ManeuverInput in = {0};
    B3_STATE line = {0};
    motion_status_snapshot_t status;
    uint32_t sample = 0u;
    S5Action action;
    ER line_error;
    ER motion_error;
    if (!ready()) { (void)stop_motor(); return; }
    motion_error = motion_get_status(&status);
    line_error = b3_get_timed_state(&line, &sample);
    /* Read time after snapshots: mutex waits can allow a newer sample.
     * Using an earlier clock wrapped its age and latched a false fault. */
    if (!now_ms(&in.now_ms) || motion_error != E_OK ||
        in.now_ms - status.sample_time_ms > 200u ||
        status.status == MOTION_STATUS_FAULT || status.status == MOTION_STATUS_TIMEOUT ||
        (fault_get() & (FAULT_MOTOR_STALL | FAULT_ENCODER))) {
        fault_stop(S5_ERR_MOTION_TIMEOUT);
        return;
    }
    in.line_valid = line_error == E_OK &&
        in.now_ms - sample <= ROBOT_BYPASS_LINE_MAX_AGE_MS;
    in.line_sample_ms = sample;
    in.line_detected = (line.line_flags & B3_LINE_F_DETECTED) != 0u;
    in.line_junction = (line.line_flags & B3_LINE_F_JUNCTION) != 0u;
    in.front_valid = front_valid;
    in.front_cm = front_cm;
    in.stopped = status.physically_stopped;
    if (!in.line_valid) { fault_stop(S5_ERR_LINE_TIMEOUT); return; }
    if (mode == NAV_PROBE) {
        motion_status_snapshot_t terminal;
        if (!front_valid || !(front_cm > ROBOT_BYPASS_MIN_TURN_CM) ||
            front_cm > S5_MAX_RANGE_CM || in.now_ms - probe_started_ms >= ROBOT_BYPASS_LEG_TIMEOUT_MS) {
            fault_stop(!front_valid ? S5_ERR_FRONT_SENSOR : S5_ERR_INSUFFICIENT_CLEARANCE);
            return;
        }
        if (motion_get_result(probe_id, &terminal) == E_OK) {
            if (terminal.status != MOTION_STATUS_COMPLETED || !terminal.physically_stopped || !in.stopped) {
                fault_stop(S5_ERR_MOTION_TIMEOUT);
                return;
            }
            probe_complete = true;
            mode = NAV_STOP;
            (void)stop_motor();
        } else b2_request_forward();
        return;
    }
    if (follow && !line_reference) {
        if (!in.line_detected) initial_line_hits = 0u;
        else if (initial_line_hits == 0u || sample != initial_line_sample) {
            initial_line_sample = sample;
            if (++initial_line_hits >= ROBOT_BYPASS_LINE_CONFIRM)
                line_reference = true;
        }
    }
    if (follow && (mode == NAV_FOLLOW || mode == NAV_STOP)) {
        bool valid_front = front_valid && front_cm > 0.0f && front_cm <= S5_MAX_RANGE_CM;
        if (!valid_front) {
            if (invalid_front_hits < S5_FRONT_INVALID_LIMIT) ++invalid_front_hits;
            if (mode == NAV_FOLLOW && !stop_queued && clear_hits >= ROBOT_CLEAR_CONFIRM &&
                invalid_front_hits < S5_FRONT_INVALID_LIMIT &&
                in.now_ms - last_clear_ms <= ROBOT_FRONT_GLITCH_GRACE_MS) {
                /* Keep the existing, unexpired clearance. Do not fabricate
                 * a reading or refresh permission from an invalid echo. */
                return;
            }
        } else invalid_front_hits = 0u;
        /* Hysteresis: require release distance to START, but keep following
         * down to STOP distance. Stopping at RELEASE left the robot parked
         * between STOP and RELEASE forever, outside the scan trigger. */
        bool clearance = mode == NAV_FOLLOW && !stop_queued &&
                         clear_hits >= ROBOT_CLEAR_CONFIRM ?
                         front_cm > ROBOT_STOP_CM : front_cm >= ROBOT_RELEASE_CM;
        if (!valid_front || !clearance) {
            clear_hits = 0u;
            (void)stop_motor();
            mode = NAV_STOP;
            return;
        }
        last_clear_ms = in.now_ms;
        if (clear_hits < ROBOT_CLEAR_CONFIRM) ++clear_hits;
        if (clear_hits < ROBOT_CLEAR_CONFIRM || (!ROBOT_CLEAR_PATH_FORWARD && !line_reference)) { (void)stop_motor(); return; }
        b2_request_forward();
        if (!b2_safety_motion_allowed()) return; /* Wait for STOP ACK. */
        /* Normal DRIVE belongs to the vehicle controller. */
        stop_queued = false;
        mode = NAV_FOLLOW;
        return;
    }
    if (mode == NAV_STOP || mode == NAV_FOLLOW) {
        (void)stop_motor();
        return;
    }
    /* Keep the bypass/recovery handoff explicit: no return-to-line leg
     * starts before the high-level controller requests FIND_LINE. */
    if (mode == NAV_AVOID && maneuver.state == S5_MAN_TURN_IN && !maneuver.pending_id)
        return;
    if (maneuver.state == S5_MAN_COMPLETE) return;
    if (maneuver.pending_id) {
        motion_status_snapshot_t result;
        if (motion_get_result(maneuver.pending_id, &result) == E_OK) {
            in.motion_terminal = true;
            in.motion_success = result.status == MOTION_STATUS_COMPLETED;
            in.result_id = result.command_id;
            in.stopped = result.physically_stopped && status.physically_stopped;
        }
    }
    in.rear_clear = rear_sensor && rear_sensor(ROBOT_BYPASS_REVERSE_CM);
    if (maneuver.pending_id &&
        (maneuver.state == S5_MAN_TURN_OUT || maneuver.state == S5_MAN_FORWARD ||
         maneuver.state == S5_MAN_TURN_IN || maneuver.state == S5_MAN_ALIGN ||
         maneuver.state == S5_MAN_ALIGN_CHECK) &&
        in.now_ms - last_turn_guard_log_ms >= 200u) {
        last_turn_guard_log_ms = in.now_ms;
        tm_printf((UB*)"[S5-TURN-GUARD] side=%d front_cm_x100=%d valid=%d\n",
                  (int)maneuver.side, (int)(front_cm * 100.0f), (int)front_valid);
    }
    action = s5_maneuver_step(&maneuver, &in);
    if (action.type == S5_ACT_STOP) {
        (void)stop_motor();
        if (maneuver.waiting_clear && in.now_ms - last_turn_guard_log_ms >= 1000u) {
            last_turn_guard_log_ms = in.now_ms;
            tm_printf((UB*)"[S5-FORWARD-WAIT] state=%d front_cm_x100=%d release_cm=%d clear_hits=%d\n",
                (int)maneuver.state, (int)(front_cm * 100.0f),
                (int)ROBOT_RELEASE_CM, (int)maneuver.clear_hits);
        }
        if (maneuver.state == S5_MAN_FAILED) fault_stop(maneuver.error);
        return;
    }
    /* B2's clearance heartbeat remains active only with fresh actual
     * front echoes and the maneuver's per-state clearance checks. */
    b2_request_forward();
    if (action.type != S5_ACT_NONE) {
        uint32_t id = 0u;
        ER error;
        if (!b2_safety_motion_allowed()) { fault_stop(S5_ERR_STOP_TIMEOUT); return; }
        if (action.type == S5_ACT_TURN)
            error = motion_turn_degrees(action.turn_deg < 0 ? -action.turn_deg : action.turn_deg,
                ROBOT_BYPASS_SPEED_CPS,
                action.turn_deg < 0 ? MOTION_TURN_LEFT : MOTION_TURN_RIGHT, &id);
        else
            error = motion_move_cm(action.distance_cm < 0 ? -action.distance_cm : action.distance_cm,
                ROBOT_BYPASS_SPEED_CPS,
                action.distance_cm < 0 ? MOTION_DIR_REVERSE : MOTION_DIR_FORWARD, &id);
        if (error != E_OK || !s5_maneuver_submitted(&maneuver, id)) {
            fault_stop(S5_ERR_MOTION_TIMEOUT);
            return;
        }
        stop_queued = false;
        tm_printf((UB*)"[S5-MOVE] state=%d id=%d cm_x100=%d turn_deg=%d\n",
                  (int)maneuver.state, (int)id, (int)(action.distance_cm * 100.0f), action.turn_deg);
    }
}
#endif
