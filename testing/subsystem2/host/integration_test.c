#include <assert.h>
#include <stdio.h>
#include "mock_platform.h"
#define SUBSYSTEM1_H
#if B2_TEST_BINARY
#define COMM_SPEED_MAX 8191
void comm_set_motion(W left, W right, W enc_left, W enc_right);
ER comm_report_error(UB code, UB detail);
#else
void comm_set_motion(H left, H right, W enc_left, W enc_right, W distance);
ER comm_report_error(UB subsystem, H code, const char *text);
#endif
#include "../../../app_program/subsystem2/b2_integration.c"

static UINT flags;
static W sent_left;
static W sent_right;
static W sent_enc_left;
static W sent_enc_right;
static UB sent_error;
static UINT reports;
static ER report_result;

ID tk_cre_flg(const T_CFLG *flag)
{ flags = flag->iflgptn; return 1; }
ER tk_set_flg(ID id, UINT bits) { (void)id; flags |= bits; return E_OK; }
ER tk_clr_flg(ID id, UINT mask) { (void)id; flags &= mask; return E_OK; }
ER tk_ref_flg(ID id, T_RFLG *flag)
{ (void)id; flag->flgptn = flags; return E_OK; }
#if B2_TEST_BINARY
ID flg_faults = 1;
void fault_raise(UINT bits) { flags |= bits; }
void fault_clear(UINT bits) { flags &= ~bits; }
UB fault_get(void) { return (UB)flags; }
void comm_set_motion(W left, W right, W enc_left, W enc_right)
#else
void comm_set_motion(H left, H right, W enc_left, W enc_right, W distance)
#endif
{
#if !B2_TEST_BINARY
    (void)distance;
#endif
    sent_left = left; sent_right = right;
    sent_enc_left = enc_left; sent_enc_right = enc_right;
}
#if B2_TEST_BINARY
ER comm_report_error(UB code, UB detail)
{
    (void)detail; sent_error = code; reports++; return report_result;
}
#else
ER comm_report_error(UB subsystem, H code, const char *text)
{
    assert(subsystem == 2); (void)text;
    sent_error = (UB)code; reports++; return report_result;
}
#endif

int main(void)
{
    motion_status_snapshot_t snapshot = {0};
    assert(b2_integration_init() == E_OK);
    flags = FAULT_IMU | FAULT_NO_ECHO | FAULT_LINE_LOST;
    b2_integration_fault(MOTION_FAULT_STALL);
    assert((flags & FAULT_MOTOR_STALL) != 0);
    b2_integration_fault(MOTION_FAULT_ENCODER);
    assert((flags & FAULT_ENCODER) != 0);
    fault_clear(FAULT_MOTOR_STALL | FAULT_ENCODER);
    assert(flags == (FAULT_IMU | FAULT_NO_ECHO | FAULT_LINE_LOST));
    b2_integration_fault(MOTION_FAULT_TIMING);
    assert(flags == (FAULT_IMU | FAULT_NO_ECHO | FAULT_LINE_LOST));
    snapshot.left_speed_cps = 5000; snapshot.right_speed_cps = -1000;
    snapshot.left_encoder_counts = 0x12345678;
    snapshot.right_encoder_counts = -123456789;
    b2_integration_publish(&snapshot);
#if B2_TEST_BINARY
    assert(sent_left == 5000 && sent_right == -1000);
#else
    assert(sent_left == 408 && sent_right == -87);
#endif
    assert(sent_enc_left == 0x12345678 && sent_enc_right == -123456789);
    snapshot.fault = MOTION_FAULT_ENCODER; snapshot.command_id = 42;
    b2_integration_publish(&snapshot);
    assert(sent_error == ERR_S2_ENCODER && reports == 1);
    b2_integration_publish(&snapshot);
    assert(reports == 1);
    snapshot.fault = MOTION_FAULT_TIMING;
    report_result = E_TMOUT;
    b2_integration_publish(&snapshot);
    assert(sent_error == ERR_S2_LOOP_OVERRUN && reports == 2);
    report_result = E_OK;
    b2_integration_publish(&snapshot);
    assert(reports == 3);
    b2_integration_publish(&snapshot);
    assert(reports == 3);
    puts(B2_TEST_BINARY ? "PASS: B1 binary counts/s and B2 faults" :
        "PASS: B1 legacy compatibility and B2 faults");
    return 0;
}
