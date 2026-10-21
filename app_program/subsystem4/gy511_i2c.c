#include "subsystem4.h"
#include "gy511_i2c.h"
#include "../../device/include/dev_i2c.h"

static ID device_id;

static bool read_registers(void *context, uint8_t address, uint8_t reg,
                           uint8_t *data, uint32_t count)
{
    (void)context;
    T_I2C_EXEC transfer = {0};
    SZ actual = 0;
    transfer.sadr = address;
    transfer.snd_size = 1;
    transfer.snd_data = &reg;
    transfer.rcv_size = count;
    transfer.rcv_data = data;
    ER er = tk_swri_dev(device_id, TDN_I2C_EXEC, &transfer, sizeof(transfer), &actual);
    return er == E_OK && actual == sizeof(transfer);
}

static bool write_register(void *context, uint8_t address, uint8_t reg, uint8_t value)
{
    (void)context;
    UB bytes[2] = {reg, value};
    SZ actual = 0;
    ER er = tk_swri_dev(device_id, address, bytes, sizeof(bytes), &actual);
    return er == E_OK && actual == sizeof(bytes);
}

bool gy511_i2c_open(gy511_bus_t *bus)
{
    if (bus == NULL) { return false; }
    if (device_id <= 0) { device_id = tk_opn_dev((UB*)"iica", TD_UPDATE); }
    if (device_id <= 0) { return false; }
    *bus = (gy511_bus_t){NULL, read_registers, write_register};
    return true;
}
