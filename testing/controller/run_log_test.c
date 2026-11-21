/* Host coverage of recorder diagnostics and the existing 23-column CSV. */
#include <assert.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define APP_H
#define IMPORT extern
#define E_OK 0
#define DI(s) ((s)=0u)
#define EI(s) ((void)(s))
typedef int INT;
typedef int ID;
typedef unsigned UINT;
typedef uint32_t UW;
typedef unsigned char UB;
typedef void (*FP)(void);
typedef struct { uint32_t lo; } SYSTIM;
volatile UW tm_usb_dropped;
static char output[8192];
static size_t used;
int tm_tusb_connected(void) { return 0; }
static int tk_get_tim(SYSTIM *t) { t->lo=1000; return E_OK; }
static int tk_dly_tsk(unsigned ms) { (void)ms; return E_OK; }
static int app_start_task(FP f, int p, int c, const char *n)
{ (void)f; (void)p; (void)c; (void)n; return 1; }
static int tm_printf(UB *fmt, ...)
{
    va_list args;
    va_start(args,fmt);
    int n=vsnprintf(output+used,sizeof(output)-used,(const char *)fmt,args);
    va_end(args);
    assert(n>=0 && (size_t)n<sizeof(output)-used);
    used+=(size_t)n;
    return n;
}
#include "../../app_program/run_log.c"

int main(void)
{
    run_log_raw(355,842);
    run_log_control(100,70000,21,-8192,8191,5);
    assert(g_count==5 && g_ring[0].arg==UINT16_MAX);
    RUN_TICK tick={0};
    tick.t_ms=100; tick.n26=1000; tick.n27=1000;
    tick.steer=-400; tick.speed=700;
    run_log_tick(&tick);
    assert(g_count==6 && g_ring[5].raw26==355 && g_ring[5].raw27==842);
    dump();
    assert(strstr(output,"LINE_AGE,65535\n"));
    assert(strstr(output,"LOOP_TIME,21\n"));
    assert(strstr(output,"MEAS_LEFT,-8192\n"));
    assert(strstr(output,"MEAS_RIGHT,8191\n"));
    assert(strstr(output,"MOTION_AGE,5\n"));
    unsigned rows=0;
    for(char *line=strtok(output,"\n");line;line=strtok(NULL,"\n")) {
        if(line[0]=='#') continue;
        unsigned commas=0;
        for(char *p=line;*p;p++) if(*p==',') ++commas;
        assert(commas==22); /* capture_run_log.ps1 accepts exactly 23 columns */
        ++rows;
    }
    assert(rows==7);
    puts("PASS run recorder timing, signed wheel feedback and CSV compatibility");
    return 0;
}
