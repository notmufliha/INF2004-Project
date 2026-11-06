#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdio.h>
#define B3_LINE_H
#define SUBSYSTEM1_H
typedef int ER;
typedef int ID;
typedef int INT;
typedef int SZ;
typedef uint32_t UW;
typedef unsigned char UB;
typedef void (*FP)(INT, void *);
typedef struct { int mtxatr; } T_CMTX;
typedef struct { uint32_t lo; } SYSTIM;
#define CONST const
#define E_OK 0
#define E_OBJ (-1)
#define E_PAR (-2)
#define E_TMOUT (-3)
#define TD_READ 1
#define TA_INHERIT 1
#define TP_PRC1 1
#define B3_LINE_THRESHOLD 2048u
#define B3_LINE_UPDATE_MS 20u
#define B3_LINE_F_DETECTED 1u
#define B3_LINE_F_LOST 2u
#define B3_LINE_F_JUNCTION 4u
typedef struct { uint16_t ir_left, ir_centre; uint8_t line_flags; } B3_STATE;
static jmp_buf iteration;
static uint32_t values[2] = {1000u, 3000u};
static int fail_channel = -1;
static bool busy, short_read, bad_clock;
static uint32_t time_ms = 100u;
static ER tk_opn_dev(const UB *name, int mode)
{ assert(name && mode == TD_READ); return 3; }
static ER tk_cls_dev(ID id, int mode) { (void)id; (void)mode; return E_OK; }
static ID tk_cre_mtx(T_CMTX *m) { assert(m->mtxatr == TA_INHERIT); return 4; }
static ER tk_loc_mtx(ID id, int timeout)
{ assert(id == 4 && timeout == 5); return busy ? E_TMOUT : E_OK; }
static ER tk_unl_mtx(ID id) { assert(id == 4); return E_OK; }
static ER tk_srea_dev(ID id, int channel, UW *out, int size, SZ *actual)
{
    assert(id == 3 && channel >= 0 && channel <= 1 && size == 1);
    if (channel == fail_channel) return E_OBJ;
    *out = values[channel];
    *actual = short_read ? 0 : 1;
    return E_OK;
}
static ER tk_get_tim(SYSTIM *time) { time->lo = time_ms; return bad_clock ? E_OBJ : E_OK; }
static void comm_set_line(uint16_t l, uint16_t c, unsigned r, unsigned flags)
{ assert(l <= 4095u && c <= 4095u && r == 0u && flags != 0u); }
static void tk_dly_tsk(unsigned ms)
{ assert(ms == B3_LINE_UPDATE_MS); longjmp(iteration, 1); }
static ID app_start_task(FP entry, INT pri, UW core, const char *name)
{ assert(entry && pri == 16 && core == TP_PRC1 && name); return 5; }
#include "../../app_program/subsystem3/b3_line.c"
static void sample(void)
{
    if (setjmp(iteration) == 0) line_task(0, NULL);
    ++time_ms;
}
int main(void)
{
    B3_STATE out;
    uint32_t timestamp;
    assert(b3_line_start() == E_OBJ);
    assert(b3_get_timed_state(&out, &timestamp) == E_OBJ);
    assert(b3_get_timed_state(NULL, &timestamp) == E_PAR);
    assert(b3_line_init() == E_OK);
    assert(b3_line_init() == E_OBJ);
    assert(b3_get_timed_state(&out, &timestamp) == E_OBJ);
    assert(b3_line_start() == E_OK);
    assert(b3_line_start() == E_OBJ);
    sample();
    assert(b3_get_timed_state(&out, &timestamp) == E_OK);
    assert(out.ir_left == 1000u && out.ir_centre == 3000u && timestamp == 100u);
    assert(out.line_flags == B3_LINE_F_DETECTED);
    fail_channel = 1;
    sample();
    assert(b3_get_timed_state(&out, &timestamp) == E_OBJ); /* No partial pair. */
    fail_channel = -1;
    values[0] = values[1] = 2048u;
    sample();
    assert(b3_get_timed_state(&out, &timestamp) == E_OK);
    assert(out.line_flags == (B3_LINE_F_DETECTED | B3_LINE_F_JUNCTION));
    values[0] = values[1] = 4095u;
    sample();
    assert(b3_get_timed_state(&out, &timestamp) == E_OK && out.line_flags == B3_LINE_F_LOST);
    values[0] = 4096u;
    sample();
    assert(b3_get_timed_state(&out, &timestamp) == E_OBJ);
    values[0] = 1000u;
    short_read = true;
    sample();
    assert(b3_get_timed_state(&out, &timestamp) == E_OBJ);
    short_read = false;
    bad_clock = true;
    sample();
    assert(b3_get_timed_state(&out, &timestamp) == E_OBJ);
    bad_clock = false;
    sample();
    assert(b3_get_timed_state(&out, &timestamp) == E_OK);
    busy = true;
    assert(b3_get_timed_state(&out, &timestamp) == E_TMOUT);
    puts("PASS B3 line input: GP26/27 channel mapping, polarity, timestamp, initialization, complete ADC pairs, invalidation, recovery, bounded locking");
    return 0;
}
