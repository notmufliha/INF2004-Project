"""Run the real B5 driver against a simulated GPIO/PWM register bank."""
from pathlib import Path
import os
import subprocess
import tempfile
import re

root = Path(__file__).resolve().parents[2]
clock_source = (root / 'kernel/sysdepend/pico_rp2040/cpu_clock.c').read_text()
clock_init = re.search(r'LOCAL void init_timer_tick\(void\)\s*\{[^}]*\}', clock_source).group(0).replace('LOCAL', 'static')
assert clock_source.index('init_timer_tick();') < clock_source.index('/* CLK SYS settings */')
driver = (root / 'app_program/subsystem5/drv_ultrasonic.c').read_text()
driver = '\n'.join(line for line in driver.splitlines()
                   if not line.startswith('#include'))
code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "HEADER"
typedef unsigned UINT;
#define S5_TRIG_PIN 16u
#define ROBOT_NO_TARGET_FORWARD 1
#define S5_ECHO_PIN 17u
#define GPIO_MODE_OUT 1
#define GPIO_OE_CLR 0x20u
#define GPIO_OD 128u
#define GPIO_IE 64u
#define GPIO_CTRL_FUNCSEL_PWM 4u
#define GPIO(n) (0x100u + (n))
#define GPIO_CTRL(n) (0x200u + (n))
#define PWM_CH0_BASE 0x300u
#define PWM_CHx_CSR 0u
#define PWM_CHx_DIV 4u
#define PWM_CHx_CTR 8u
#define PWM_CHx_TOP 16u
#define PWM_CH_CSR_EN 1u
#define TIMER_TIMERAWL 0x400u
#define DI(x) ((x) = 0)
#define EI(x) ((void)(x))
#define tm_printf(...) ((void)0)
static uint32_t regs[1024], clock_us, triggered_at, width;
static bool triggered, stuck;
static bool trigger_high;
static unsigned yield_calls;
static uint32_t watchdog_tick = 0x200u;
static void tk_dly_tsk(unsigned ms) {
    assert(!trigger_high && ms == 1u);
    ++yield_calls;
    clock_us += ms * 1000u;
}
static uint32_t last_fire;
static bool fired;
static uint32_t in_w(uint32_t address) {
    if (address == TIMER_TIMERAWL) {
        unsigned cycles = watchdog_tick & 0x1ffu;
        return clock_us++ * 12u / (cycles ? cycles : 512u);
    }
    if (address == 0x4005802cu) return watchdog_tick;
    if (address == 0x4000c008u) return 1u << 14u;
    if (address == 0x308u && triggered) {
        uint32_t age = clock_us - triggered_at;
        return age <= 100u ? 0u : (age - 100u < width ? age - 100u : width);
    }
    return regs[address];
}
static void out_w(uint32_t a, uint32_t v) {
    if (a == 0x4005802cu) watchdog_tick = v;
    else regs[a] = v;
}
static void set_w(uint32_t a, uint32_t v) { regs[a] |= v; }
static void clr_w(uint32_t a, uint32_t v) {
    if (a != 0x4000c000u) regs[a] &= ~v;
}
static void gpio_set_pin(unsigned p, unsigned m) { (void)p; (void)m; }
static void gpio_set_val(unsigned p, unsigned v) {
    (void)p;
    trigger_high = v != 0u;
    if (v) {
        if (fired) assert((uint32_t)(clock_us - last_fire) >= 65000u);
        fired = true; last_fire = clock_us;
        triggered = true; triggered_at = clock_us;
    }
}
static unsigned gpio_get_val(unsigned p) {
    (void)p;
    uint32_t age = clock_us - triggered_at;
    return stuck || (triggered && age >= 100u && age < 100u + width);
}
#define XOSC_MHz 12u
CLOCK_INIT
DRIVER
static S5EchoReading read_width(uint32_t us) {
    triggered = false; stuck = false; width = us;
    return s5_ultrasonic_read_detailed();
}
int main(void) {
    /* Replay the reset divisor: 65k timer ticks incorrectly take ~2.77 s. */
    assert(65000u * 512u / 12u > 2700000u);
    init_timer_tick();
    assert((watchdog_tick & 0x3ffu) == 0x20cu);
    s5_ultrasonic_init();
    assert(regs[0x300] == 17u && regs[0x304] == 2000u);
    assert(regs[0x310] == 65535u);
    assert(regs[GPIO_CTRL(17)] == GPIO_CTRL_FUNCSEL_PWM);
    assert((regs[GPIO(17)] & (GPIO_IE | GPIO_OD)) == (GPIO_IE | GPIO_OD));
    S5EchoReading r = read_width(3000u);
    assert(r.status == S5_ECHO_OK && r.pulse_us == 3000u);
    assert(r.cm > 51.0f && r.cm < 52.0f);
    assert(yield_calls > 0u); /* RTOS switches do not distort PWM width. */
    r = read_width(150u); /* A real close echo is measurable. */
    assert(r.status == S5_ECHO_OK && r.cm < 3.0f);
    assert(read_width(50u).status == S5_ECHO_TOO_SHORT);
    assert(read_width(0u).status == S5_ECHO_NO_RISE);
    assert(read_width(25000u).status == S5_ECHO_OUT_OF_RANGE);
    assert(read_width(55000u).status == S5_ECHO_OUT_OF_RANGE);
    assert(read_width(70000u).status == S5_ECHO_NO_FALL);
    triggered = false; width = 0u;
    assert(s5_ultrasonic_read_front().status == S5_ECHO_OPEN_SPACE);
    triggered = false; width = 25000u;
    assert(s5_ultrasonic_read_front().status == S5_ECHO_OPEN_SPACE);
    triggered = false; width = 1500u;
    assert(s5_ultrasonic_read_front().status == S5_ECHO_OK);
    triggered = false; width = 38000u;
    assert(s5_ultrasonic_read_front().status == S5_ECHO_OPEN_SPACE);
    triggered = false; width = 55000u;
    assert(s5_ultrasonic_read_front().status == S5_ECHO_OPEN_SPACE);
    triggered = false; width = 70000u;
    assert(s5_ultrasonic_read_front().status == S5_ECHO_NO_FALL);
    for (unsigned i = 0; i < 12u; ++i) {
        uint32_t started_us = clock_us;
        triggered = false; width = i % 3u == 0u ? 0u : i % 3u == 1u ? 38000u : 55000u;
        assert(s5_ultrasonic_read_front().status == S5_ECHO_OPEN_SPACE);
        assert(clock_us - started_us < 130000u); /* Hardware time, below 350-ms lease. */
    }
    assert(read_width(1500u).status == S5_ECHO_OK); /* Recovers on next fresh pulse. */
    triggered = false; stuck = true;
    assert(s5_ultrasonic_read_detailed().status == S5_ECHO_STUCK_HIGH);
    puts("PASS B5 driver: initialization, clear/close echo, glitch, no rise, out of range, no fall, stuck high");
}
'''.replace('HEADER', (root / 'app_program/subsystem5/drv_ultrasonic.h').as_posix()).replace('CLOCK_INIT', clock_init).replace('DRIVER', driver)
with tempfile.TemporaryDirectory() as tmp:
    src = Path(tmp) / 'driver_test.c'
    exe = Path(tmp) / 'driver_test'
    src.write_text(code)
    # MinGW does not ship libubsan. Keep sanitizer coverage on Unix builds.
    sanitizer = [] if os.name == 'nt' else ['-fsanitize=undefined']
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    *sanitizer, str(src), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
