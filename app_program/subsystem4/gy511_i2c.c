/* NOTE: Each UB pointer cast below adapts a string/byte buffer to the
 * kernel console API. Character bytes are preserved and formats are not
 * changed.
 */
/** @file gy511_i2c.c
 * @brief Buddy 4 gy511 i2c module.
 */
#include "../app.h"
#include "gy511_i2c.h"
#include "../../device/include/dev_i2c.h"

static ID gh_i2c_device;
static UW bus_failures;

static void report_bus_failure(ER error, uint8_t address, uint8_t reg)
{
    ++bus_failures;
    if (bus_failures == 1U || bus_failures % 100U == 0U)
        tm_printf((UB *)"[B4-I2C] failed after retry: error=%d address=%d reg=%d failures=%u\n",
                  (INT)error, (INT)address, (INT)reg, bus_failures);
}

static bool read_registers (void * p_context, uint8_t address, uint8_t reg,
                            uint8_t * p_data, uint32_t count);

static bool write_register (void * p_context, uint8_t address, uint8_t reg,
                            uint8_t value);

/** @brief Open the exclusive sampling-task bus and bind its callbacks. */
bool
gy511_i2c_open (gy511_bus_t * p_bus)
{
    if (NULL == p_bus)
    {
        return false;
    }

    if (gh_i2c_device <= 0)
    {
        gh_i2c_device = tk_opn_dev((UB *)"iica", TD_UPDATE);
    }

    if (gh_i2c_device <= 0)
    {
        return false;
    }

    *p_bus = (gy511_bus_t){NULL, read_registers, write_register};
    return true;
}

/** @brief Read a bounded register burst through the kernel I2C driver. */
static bool
read_registers (void * p_context, uint8_t address, uint8_t reg,
                uint8_t * p_data, uint32_t count)
{
    /* This bus adapter needs no caller-owned callback context. */
    (void)p_context;

    if ((NULL == p_data) || (0U == count) || (count > 6U))
    {
        return false;
    }

    T_I2C_EXEC transfer = {0};
    SZ         actual   = 0;

    transfer.sadr     = address;
    transfer.snd_size = 1;
    transfer.snd_data = &reg;
    /* The register protocol reads at most six bytes, which fits kernel SZ. */
    transfer.rcv_size = (SZ)count;
    transfer.rcv_data = p_data;

    ER error_code = E_OK;
    for (UINT attempt = 0U; attempt < 2U; ++attempt) {
        actual = 0;
        error_code = tk_swri_dev(gh_i2c_device, TDN_I2C_EXEC, &transfer,
                                sizeof(transfer), &actual);
        if (error_code == E_OK && actual == (SZ)sizeof(transfer)) return true;
        if (error_code != E_TMOUT && error_code != E_IO) break;
    }
    report_bus_failure(error_code, address, reg);
    return false;
}

/** @brief Write a register and its value through the kernel I2C driver. */
static bool
write_register (void * p_context, uint8_t address, uint8_t reg, uint8_t value)
{
    /* This bus adapter needs no caller-owned callback context. */
    (void)p_context;

    UB bytes[2] = {reg, value};
    SZ actual   = 0;
    ER error_code = E_OK;
    for (UINT attempt = 0U; attempt < 2U; ++attempt) {
        actual = 0;
        error_code = tk_swri_dev(gh_i2c_device, address, bytes, sizeof(bytes), &actual);
        if (error_code == E_OK && actual == (SZ)sizeof(bytes)) return true;
        if (error_code != E_TMOUT && error_code != E_IO) break;
    }
    report_bus_failure(error_code, address, reg);
    return false;
}

/*** end of file ***/

