"""Exercise the production GY-511 adapter with deterministic bus failures."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / "app_program/subsystem4/gy511_i2c.c").read_text()
source = "\n".join(line for line in source.splitlines() if not line.startswith("#include"))
stub = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
typedef int ID, ER, INT, SZ;
typedef unsigned UINT, UW;
typedef unsigned char UB;
#define E_OK 0
#define E_TMOUT (-1)
#define E_IO (-2)
#define E_PAR (-3)
#define TD_UPDATE 1
#define TDN_I2C_EXEC (-10)
typedef struct { unsigned sadr; SZ snd_size; void *snd_data; SZ rcv_size; void *rcv_data; } T_I2C_EXEC;
typedef struct {
    void *p_context;
    bool (*p_read)(void *, uint8_t, uint8_t, uint8_t *, uint32_t);
    bool (*p_write)(void *, uint8_t, uint8_t, uint8_t);
} gy511_bus_t;
typedef struct { uint32_t lo; } SYSTIM;
typedef struct { uint32_t timestamp_ms; } imu_data_t;
typedef struct {
    void *p_context;
    bool (*p_clock)(void *, uint32_t *);
    void (*p_pair)(void *, imu_data_t *, uint32_t, uint32_t, bool);
} gy511_capture_t;
typedef struct { gy511_capture_t capture; } gy511_t;
static ER tk_get_tim(SYSTIM *now) { now->lo = 123U; return E_OK; }
static unsigned paired;
static void imu_odom_pair(imu_data_t *data, uint32_t begin,
                          uint32_t end, bool valid) {
    assert(data != NULL && begin == 120U && end == 123U && valid);
    ++paired;
}
static bool gy511_set_capture(gy511_t *sensor,
                              const gy511_capture_t *capture) {
    sensor->capture = *capture; return true;
}
static unsigned calls;
static ER errors[2];
static bool short_transfer;
static ID tk_opn_dev(UB *name, int mode) { (void)name; (void)mode; return 7; }
static ER tk_swri_dev(ID id, int address, void *data, SZ size, SZ *actual) {
    (void)address; (void)data;
    assert(id == 7 && calls < 2);
    *actual = short_transfer ? size - 1 : size;
    return errors[calls++];
}
static INT tm_printf(UB *format, ...) { (void)format; return 0; }
'''
tests = r'''
static void reset(ER first, ER second) {
    calls = 0; errors[0] = first; errors[1] = second; short_transfer = false;
}
int main(void) {
    gy511_bus_t bus;
    uint8_t data[6] = {0};
    assert(gy511_i2c_open(&bus));
    gy511_t sensor;
    imu_data_t sample = {0};
    uint32_t time_ms = 0U;
    assert(gy511_i2c_pair(&sensor));
    assert(sensor.capture.p_clock(NULL, &time_ms) && time_ms == 123U);
    sensor.capture.p_pair(NULL, &sample, 120U, time_ms, true);
    assert(paired == 1U);
    reset(E_OK, E_OK);
    assert(bus.p_read(NULL, 0x19, 0xA8, data, 6) && calls == 1);
    reset(E_TMOUT, E_OK);
    assert(bus.p_read(NULL, 0x19, 0xA8, data, 6) && calls == 2);
    reset(E_IO, E_OK);
    assert(bus.p_write(NULL, 0x19, 0x20, 0x57) && calls == 2);
    reset(E_TMOUT, E_TMOUT);
    assert(!bus.p_read(NULL, 0x19, 0xA8, data, 6) && calls == 2);
    reset(E_IO, E_IO);
    assert(!bus.p_write(NULL, 0x19, 0x20, 0x57) && calls == 2);
    reset(E_PAR, E_OK);
    assert(!bus.p_read(NULL, 0x19, 0xA8, data, 6) && calls == 1);
    reset(E_OK, E_OK); short_transfer = true;
    assert(!bus.p_read(NULL, 0x19, 0xA8, data, 6) && calls == 1);
    reset(E_OK, E_OK);
    assert(!bus.p_read(NULL, 0x19, 0xA8, data, 8) && calls == 0);
    puts("PASS GY-511 bus: transient retry, bounded persistent failure, short transfer and argument rejection");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="imu_i2c_retry_") as folder:
    path = Path(folder)
    test = path / "test.c"
    binary = path / ("test.exe" if os.name == "nt" else "test")
    test.write_text(stub + source + tests)
    cc = os.environ.get("CC") or shutil.which("gcc") or shutil.which("cc")
    subprocess.run([cc, "-std=c11", "-Wall", "-Wextra", "-Werror", str(test), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
