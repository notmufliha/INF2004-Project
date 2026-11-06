#include "b3_line.h"
#include "../subsystem1/subsystem1.h"
#include <stdbool.h>

static ID adc_device;
static ID state_mutex;
static B3_STATE state;
static uint32_t sample_ms;
static bool valid;
static bool started;

ER b3_line_init(void)
{
    T_CMTX mutex = {0};
    if (adc_device > 0 || state_mutex > 0) return E_OBJ;
    adc_device = tk_opn_dev((CONST UB *)"adca", TD_READ);
    if (adc_device < E_OK) return (ER)adc_device;
    mutex.mtxatr = TA_INHERIT;
    state_mutex = tk_cre_mtx(&mutex);
    if (state_mutex < E_OK) {
        (void)tk_cls_dev(adc_device, 0);
        adc_device = 0;
        return (ER)state_mutex;
    }
    valid = started = false;
    state = (B3_STATE){0, 0, B3_LINE_F_LOST};
    return E_OK;
}
ER b3_get_timed_state(B3_STATE *out, uint32_t *timestamp)
{
    ER error;
    bool available;
    if (!out || !timestamp) return E_PAR;
    if (state_mutex <= 0) return E_OBJ;
    error = tk_loc_mtx(state_mutex, 5);
    if (error != E_OK) return error;
    *out = state;
    *timestamp = sample_ms;
    available = valid;
    error = tk_unl_mtx(state_mutex);
    if (error != E_OK) return error;
    return available ? E_OK : E_OBJ;
}
static bool read_channel(int channel, uint16_t *value)
{
    UW word = 0u;
    SZ actual = 0;
    if (tk_srea_dev(adc_device, channel, &word, 1, &actual) != E_OK ||
        actual != 1 || word > 4095u) return false;
    *value = (uint16_t)word;
    return true;
}
static void line_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    for (;;) {
        B3_STATE next = {0};
        SYSTIM time;
        bool ok = read_channel(0, &next.ir_left) && read_channel(1, &next.ir_centre) &&
                  tk_get_tim(&time) == E_OK;
        if (ok) {
            bool left = next.ir_left <= B3_LINE_THRESHOLD;
            bool centre = next.ir_centre <= B3_LINE_THRESHOLD;
            next.line_flags = left || centre ? B3_LINE_F_DETECTED : B3_LINE_F_LOST;
            if (left && centre) next.line_flags |= B3_LINE_F_JUNCTION;
        }
        if (tk_loc_mtx(state_mutex, 5) == E_OK) {
            valid = ok;
            if (ok) { state = next; sample_ms = time.lo; }
            (void)tk_unl_mtx(state_mutex);
        }
        if (ok) comm_set_line(next.ir_left, next.ir_centre, 0u, next.line_flags);
        tk_dly_tsk(B3_LINE_UPDATE_MS);
    }
}
ER b3_line_start(void)
{
    ID task;
    if (state_mutex <= 0 || adc_device <= 0 || started) return E_OBJ;
    task = app_start_task((FP)line_task, 16, TP_PRC1, "b3_line");
    if (task < E_OK) return (ER)task;
    started = true;
    return E_OK;
}
