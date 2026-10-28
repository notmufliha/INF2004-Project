#include "b2_integration.h"
#include "encoder_control.h"
#include "../subsystem1/subsystem1.h"
#include "../fault_flags.h"
#include "../error_codes.h"

/* The old JSON B1 branch has no shared fault service. Supply the same
 * event-flag API here until the binary B1 branch is merged. That branch
 * defines COMM_SPEED_MAX and owns these symbols and initialization itself.
 */
#ifndef COMM_SPEED_MAX
ID flg_faults;

void fault_raise(UINT bits)
{
    if (flg_faults > 0)
    {
        (void)tk_set_flg(flg_faults, bits & FAULT_ALL);
    }
}

void fault_clear(UINT bits)
{
    if (flg_faults > 0)
    {
        (void)tk_clr_flg(flg_faults, ~(bits & FAULT_ALL));
    }
}

UB fault_get(void)
{
    T_RFLG flag;
    if (flg_faults <= 0 || tk_ref_flg(flg_faults, &flag) != E_OK)
    {
        return 0u;
    }
    return (UB)(flag.flgptn & FAULT_ALL);
}
#endif

ER b2_integration_init(void)
{
#ifndef COMM_SPEED_MAX
    T_CFLG flag = {0};
    flag.flgatr = TA_TFIFO | TA_WMUL;
    flg_faults = tk_cre_flg(&flag);
#endif
    if (flg_faults <= 0)
    {
        return flg_faults < 0 ? flg_faults : E_OBJ;
    }
    /* Reset only B2's bits during subsystem initialization. */
    fault_clear(FAULT_MOTOR_STALL | FAULT_ENCODER);
    return E_OK;
}

void b2_integration_fault(motion_fault_t fault)
{
    if (fault == MOTION_FAULT_STALL)
    {
        fault_raise(FAULT_MOTOR_STALL);
    }
    else if (fault == MOTION_FAULT_ENCODER)
    {
        fault_raise(FAULT_ENCODER);
    }
}

static UB b2_error_code(motion_fault_t fault)
{
    switch (fault)
    {
        case MOTION_FAULT_STALL: return ERR_S2_MOTOR_STALL;
        case MOTION_FAULT_ENCODER: return ERR_S2_ENCODER;
        case MOTION_FAULT_TIMING: return ERR_S2_LOOP_OVERRUN;
        case MOTION_FAULT_DURATION: return ERR_S2_MOTION_TIMEOUT;
        case MOTION_FAULT_STOPPING: return ERR_S2_STOP_TIMEOUT;
        case MOTION_FAULT_COMMAND: return ERR_S2_COMMAND;
        case MOTION_FAULT_WATCHDOG: return ERR_S2_DRIVE_WATCHDOG;
        default: return 0u;
    }
}

#ifndef COMM_SPEED_MAX
static H b2_legacy_speed(INT cps, float counts_per_rev)
{
    float speed = cps * 3.14159265f *
        ENC_WHEEL_DIAMETER_CM * 10.0f / counts_per_rev;
    if (speed > 32767.0f)
    {
        speed = 32767.0f;
    }
    if (speed < -32768.0f)
    {
        speed = -32768.0f;
    }
    return (H)speed;
}
#endif

static void b2_send_motion(const motion_status_snapshot_t *snapshot)
{
#ifdef COMM_SPEED_MAX
    /* Keep full signed totals here; B1 packs the low 24 bits on the wire. */
    comm_set_motion(snapshot->left_speed_cps, snapshot->right_speed_cps,
        snapshot->left_encoder_counts, snapshot->right_encoder_counts);
#else
    float distance = snapshot->total_distance_mm;
    if (distance > 2147483520.0f)
    {
        distance = 2147483520.0f;
    }
    comm_set_motion(b2_legacy_speed(snapshot->left_speed_cps,
        ENC_LEFT_COUNTS_PER_REV), b2_legacy_speed(snapshot->right_speed_cps,
        ENC_RIGHT_COUNTS_PER_REV), snapshot->left_encoder_counts,
        snapshot->right_encoder_counts, (W)distance);
#endif
}

/* Called from the low-priority telemetry task, never the 10 ms loop/ISR. */
void b2_integration_publish(const motion_status_snapshot_t *snapshot)
{
    static uint32_t last_error_id;
    static motion_fault_t last_error_fault;
    UB code = b2_error_code(snapshot->fault);
    ER error;
    b2_send_motion(snapshot);
    if (code == 0u || (snapshot->command_id == last_error_id &&
        snapshot->fault == last_error_fault))
    {
        return;
    }
#ifdef COMM_SPEED_MAX
    error = comm_report_error(code, (UB)snapshot->fault);
#else
    error = comm_report_error(2u, (H)code, "B2 motion fault");
#endif
    if (error == E_OK)
    {
        last_error_id = snapshot->command_id;
        last_error_fault = snapshot->fault;
    }
}
