"""Exercise the actual safety gate and forward task with mocked motor I/O."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
safety = (root / 'app_program/subsystem2/b2_motor.c').read_text()
safety = '\n'.join(line for line in safety.splitlines()
                   if not line.startswith('#include'))
source = (root / 'app_program/subsystem2/subsystem2.c').read_text()
task = source[source.index('static void b2_forward_stop_task('):
              source.index('\n#endif', source.index('static void b2_forward_stop_task('))]
config = (root / 'app_program/robot_config.h').as_posix()
code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <setjmp.h>
#include "CONFIG"
typedef unsigned UINT;
typedef int ER;
typedef int INT;
typedef unsigned char UB;
typedef struct { uint32_t hi, lo; } SYSTIM;
#define E_OK 0
#define TK_SUPPORT_SMP 0
#define DI(x) ((x) = 0)
#define EI(x) ((void)(x))
#define tm_printf(...) ((void)0)
static uint32_t clock_ms;
static int left, right;
static jmp_buf end_tick;
static ER tk_get_tim(SYSTIM *t) { t->lo = clock_ms; return E_OK; }
static void motor_stop(void) { left = right = 0; }
static void motor_left_set(int v) { left = v; }
static void motor_right_set(int v) { right = v; }
static void tk_dly_tsk(unsigned ms) { (void)ms; longjmp(end_tick, 1); }
SAFETY
TASK
static void tick(void) {
    if (setjmp(end_tick) == 0) b2_forward_stop_task(0, 0);
}
int main(void) {
    b2_safety_init();
    tick(); assert(left == 0 && right == 0);
    b2_request_forward();
    tick(); assert(left == 50 && right == 50);
    uint32_t stale = b2_safety_stop_generation();
    b2_request_stop(); assert(left == 0 && right == 0);
    b2_request_forward(); assert(!b2_safety_motion_allowed());
    tick(); assert(left == 0 && right == 0);
    b2_request_forward(); tick(); assert(left == 50 && right == 50);
    b2_safety_apply_pwm(50, 50, stale); assert(left == 0 && right == 0);
    b2_request_forward(); tick(); assert(left == 50 && right == 50);
    clock_ms += ROBOT_MOTOR_WATCHDOG_MS + 1;
    tick(); assert(left == 0 && right == 0);
    b2_request_forward(); tick(); assert(left == 50 && right == 50);
    puts("PASS: boot STOP, forward PWM, immediate STOP, STOP acknowledgement, stale commands, watchdog, recovery");
}
'''.replace('CONFIG', config).replace('SAFETY', safety).replace('TASK', task)
with tempfile.TemporaryDirectory() as tmp:
    src = Path(tmp) / 'test.c'
    exe = Path(tmp) / 'test'
    src.write_text(code)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=undefined', str(src), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
