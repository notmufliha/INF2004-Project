/**
 * @file run_log.c
 * @brief Run recorder (see run_log.h).
 */
#include "run_log.h"
#include "app.h"

#if RUN_LOG_ENABLE

/* USB CDC line state (DTR) from the T-Monitor USB glue. */
IMPORT int tm_tusb_connected(void);
IMPORT volatile UW tm_usb_dropped;

#define RUN_LOG_TASK_PRI 22   /* below every subsystem band (TEAM.md 4) */
#define RUN_LOG_POLL_MS 200u
#define RUN_LOG_LINE_GAP_MS 2 /* pace the dump: the USB ring is 4 KB */

static RUN_TICK g_ring[RUN_LOG_RECORDS];
static volatile uint32_t g_head;      /* next slot */
static volatile uint32_t g_count;     /* valid records, <= RUN_LOG_RECORDS */
static volatile uint32_t g_written;   /* total ever written */
static volatile uint16_t g_raw26, g_raw27;
static volatile bool g_dumping;

static void put(const RUN_TICK *record)
{
    UINT status;

    if (g_dumping) return; /* the dump reads the ring in place */
    DI(status);
    g_ring[g_head] = *record;
    g_head = (g_head + 1u) % RUN_LOG_RECORDS;
    if (g_count < RUN_LOG_RECORDS) g_count++;
    g_written++;
    EI(status);
}

void run_log_raw(uint16_t raw26, uint16_t raw27)
{
    g_raw26 = raw26;
    g_raw27 = raw27;
}

void run_log_tick(RUN_TICK *tick)
{
    tick->raw26 = g_raw26;
    tick->raw27 = g_raw27;
    tick->event = RUN_EV_NONE;
    tick->arg = 0u;
    put(tick);
}

void run_log_event(uint32_t now_ms, uint8_t event, uint16_t arg)
{
    RUN_TICK record = {0};

    if (now_ms == 0u) { /* caller has no kernel time: take it here */
        SYSTIM now;
        if (tk_get_tim(&now) == E_OK) now_ms = now.lo;
    }
    record.t_ms = now_ms;
    record.event = event;
    record.arg = arg;
    record.raw26 = g_raw26;
    record.raw27 = g_raw27;
    put(&record);
}

static const char *const g_event_names[RUN_EV_COUNT] = {
    "", "START", "STOP", "BARCODE_READ", "BARCODE_REJECT", "GP28_RUN",
    "JUNCTION", "NAV_START", "NAV_DONE", "NAV_WAITING", "LINE_LOST",
    "RESCAN", "LINE_AGE", "LOOP_TIME", "MEAS_LEFT", "MEAS_RIGHT", "MOTION_AGE"
};

static uint16_t diagnostic_ms(uint32_t value)
{
    return value > UINT16_MAX ? UINT16_MAX : (uint16_t)value;
}

/* Reuse event records to keep the RAM allocation and CSV columns stable.
 * Wheel measurements describe feedback preceding this tick's command. */
void run_log_control(uint32_t now_ms, uint32_t line_age, uint32_t loop_ms,
                     int16_t left_cps, int16_t right_cps, uint32_t motion_age)
{
    run_log_event(now_ms, RUN_EV_LINE_AGE, diagnostic_ms(line_age));
    run_log_event(now_ms, RUN_EV_LOOP_TIME, diagnostic_ms(loop_ms));
    run_log_event(now_ms, RUN_EV_MEAS_LEFT, (uint16_t)left_cps);
    run_log_event(now_ms, RUN_EV_MEAS_RIGHT, (uint16_t)right_cps);
    run_log_event(now_ms, RUN_EV_MOTION_AGE, diagnostic_ms(motion_age));
}

static void dump(void)
{
    uint32_t count, start, dropped_before;

    g_dumping = true;
    count = g_count;
    start = (g_head + RUN_LOG_RECORDS - count) % RUN_LOG_RECORDS;
    dropped_before = tm_usb_dropped;
    tm_printf((UB *)"#RUNLOG v1 records=%d written=%d\n",
              (INT)count, (INT)g_written);
    tk_dly_tsk(RUN_LOG_LINE_GAP_MS);
    tm_printf((UB *)"t_ms,travel_mm,state,raw26,raw27,n26_right,n27_left,gp28,detected,lost,jflag,tape,jlatch,barcode,floor,straddle,curve,steer,speed,pending,nav,event,arg\n");
    for (uint32_t i = 0u; i < count; ++i) {
        const RUN_TICK *r = &g_ring[(start + i) % RUN_LOG_RECORDS];

        if (r->event != RUN_EV_NONE) {
            /* Event: time, raw sensors, name, argument. */
            INT argument = (INT)r->arg;
            if ((r->event == RUN_EV_MEAS_LEFT || r->event == RUN_EV_MEAS_RIGHT) &&
                r->arg > INT16_MAX) argument -= 65536;
            tm_printf((UB *)"%u,,,%d,%d,,,,,,,,,,,,,,,,,%s,%d\n",
                      (UW)r->t_ms, (INT)r->raw26, (INT)r->raw27,
                      r->event < RUN_EV_COUNT ? g_event_names[r->event] : "?",
                      argument);
        } else {
            tm_printf((UB *)"%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,,\n",
                      (UW)r->t_ms, (UW)r->travel_mm, (INT)r->state,
                      (INT)r->raw26, (INT)r->raw27, (INT)r->n26, (INT)r->n27,
                      (INT)((r->bits & RUN_BIT_GP28) != 0u),
                      (INT)((r->bits & RUN_BIT_DETECTED) != 0u),
                      (INT)((r->bits & RUN_BIT_LOST) != 0u),
                      (INT)((r->bits & RUN_BIT_JFLAG) != 0u),
                      (INT)((r->bits & RUN_BIT_TAPE) != 0u),
                      (INT)((r->bits & RUN_BIT_JLATCH) != 0u),
                      (INT)((r->bits & RUN_BIT_BARCODE) != 0u),
                      (INT)((r->bits & RUN_BIT_FLOOR) != 0u),
                      (INT)((r->bits & RUN_BIT_STRADDLE) != 0u),
                      (INT)r->curve, (INT)r->steer, (INT)r->speed,
                      (INT)r->pending, (INT)r->nav);
        }
        tk_dly_tsk(RUN_LOG_LINE_GAP_MS);
    }
    tk_dly_tsk(20);
    tm_printf((UB *)"#END records=%d usb_dropped=%d\n", (INT)count,
              (INT)(tm_usb_dropped - dropped_before));
    g_dumping = false;
}

/* Dump whenever a terminal opens the USB port (DTR goes on). */
static void run_log_task(INT stacd, void *exinf)
{
    int was_connected = 1; /* a port already open at boot: no dump */

    (void)stacd;
    (void)exinf;
    for (;;) {
        int connected = tm_tusb_connected();

        if (connected && !was_connected) {
            tk_dly_tsk(500); /* let the host settle */
            dump();
        }
        was_connected = connected;
        tk_dly_tsk(RUN_LOG_POLL_MS);
    }
}

int run_log_start(void)
{
    ID task = app_start_task((FP)run_log_task, RUN_LOG_TASK_PRI, 0,
                             "run_log");
    return task < E_OK ? (int)task : E_OK;
}

#endif
