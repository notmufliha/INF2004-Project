#include "vehicle_controller.h"
#include "robot_config.h"
#include "subsystem2/b2_motor.h"
#include "subsystem3/b3_line_barcode.h"
#include "subsystem4/imu_vc.h"
#include "subsystem4/subsystem4.h"
#include <stddef.h>

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
}

static void safe_stop(VEHICLE_CONTROLLER *c)
{
    c->running = false;
    c->paused = false;
    c->navigation = 0u;
    c->navigation_id = 0u;
    stop_motion(c);
    set_state(c, ROBOT_SAFE_STOP);
}

void vehicle_controller_init(VEHICLE_CONTROLLER *c)
{
    *c = (VEHICLE_CONTROLLER){0};
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
    if (!c->running || c->estop || c->mode == MODE_TEST) {
        stop_motion(c);
        return;
    }
    if (!in->motion_valid || !in->imu_valid || in->hazard ||
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
    if (c->navigation == 0u &&
        (!(in->line_flags & LINE_F_DETECTED) || (in->line_flags & LINE_F_LOST))) {
        if (c->state != ROBOT_LINE_RECOVERY) c->recovery_ms = in->now_ms;
        stop_motion(c);
        set_state(c, ROBOT_LINE_RECOVERY);
        if (in->now_ms - c->recovery_ms >= VC_RECOVERY_MS) safe_stop(c);
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
    if (c->mode == MODE_AUTO && c->navigation == 0u &&
        nav >= B3_NAV_LEFT && nav <= B3_NAV_UTURN) {
        c->navigation = nav;
        c->navigation_ms = in->now_ms;
        stop_motion(c);
        set_state(c, ROBOT_NAV_COMMAND);
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
    /* Provisional two-sensor steering: left-only detection steers left.
     * Centre or junction detection drives straight; calibrate on the track. */
    command.steering_permille = (in->left <= 2048u && in->centre > 2048u) ? -350 : 0;
    /* Preserve B2's outer-wheel <=5000 cps constraint at SPEED 100. */
    command.target_speed_cps = speed * 1000 / (1000 - command.steering_permille);
    if (in->hump) command.target_speed_cps /= 2;
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
            if (!stop_barrier || halt_command) vehicle_controller_command(&c, &remote);
            if (halt_command) stop_barrier = true;
        }
        for (UINT i = 0u; i < B3_NAV_QUEUE_LENGTH; ++i) {
            UB next;
            if (tk_rcv_mbf(mbf_nav_cmd, &next, TMO_POL) != (INT)sizeof(next)) break;
            if (nav == 0u && was_running && c.running && !stop_barrier) nav = next;
        }
        in.line_valid = b3_get_timed_state(&line, &line_ms) == E_OK &&
            now.lo - line_ms <= VC_LINE_MAX_AGE_MS;
        if (in.line_valid) {
            in.left = (UH)line.ir_left;
            in.centre = (UH)line.ir_centre;
            in.line_flags = (UB)line.line_flags;
        }
        in.imu_valid = imu.b_valid && now.lo - imu.timestamp_ms <= VC_IMU_MAX_AGE_MS;
        in.hump = imu.b_hump_detected;
        in.hazard = hazard || imu.b_sudden_impact ||
            imu.roll_deg > 25.0f || imu.roll_deg < -25.0f ||
            imu.pitch_deg > 40.0f || imu.pitch_deg < -40.0f;
        in.motion_valid = motion_get_status(&in.motion) == E_OK &&
            now.lo - in.motion.sample_time_ms <= VC_MOTION_MAX_AGE_MS;
        in.clearance = b2_safety_motion_allowed();
        in.safety_generation = b2_safety_stop_generation();
        in.faults = fault_get();
        vehicle_controller_step(&c, &in, nav);
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
