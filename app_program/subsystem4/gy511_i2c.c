/* NOTE: Each UB pointer cast below adapts a string/byte buffer to the
 * kernel console API. Character bytes are preserved and formats are not
 * changed.
 */
/** @file gy511_i2c.c
 * @brief Buddy 4 gy511 i2c module.
 */
#include "../app.h"
#include "gy511_i2c.h"
#include "imu_odom.h"
#include "../../device/include/dev_i2c.h"

static ID   gh_i2c_device;
static bool capture_clock (void * p_context, uint32_t * p_time_ms);
static void pair_acceleration (void * p_context, imu_data_t * p_data,
                               uint32_t begin_ms, uint32_t end_ms,
                               bool b_time_valid);

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

/** @brief Bind real clock and encoder hooks without coupling decoder to kernel.
 */
bool
gy511_i2c_pair (gy511_t * p_sensor)
{
    const gy511_capture_t capture = {NULL, capture_clock, pair_acceleration};

    return gy511_set_capture(p_sensor, &capture);
}

/** @brief Read the software acquisition clock only in the sampling task. */
static bool
capture_clock (void * p_context, uint32_t * p_time_ms)
{
    SYSTIM now = {0};

    (void)p_context;

    if (E_OK != tk_get_tim(&now))
    {
        return false;
    }

    *p_time_ms = now.lo;
    return true;
}

/** @brief Capture paired counts while acceleration bytes are still fresh. */
static void
pair_acceleration (void * p_context, imu_data_t * p_data, uint32_t begin_ms,
                   uint32_t end_ms, bool b_time_valid)
{
    (void)p_context;
    imu_odom_pair(p_data, begin_ms, end_ms, b_time_valid);
}

/** @brief Read a bounded register burst through the kernel I2C driver. */
static bool
read_registers (void * p_context, uint8_t address, uint8_t reg,
                uint8_t * p_data, uint32_t count)
{
    /* This bus adapter needs no caller-owned callback context. */
    (void)p_context;

    if ((NULL == p_data) || (0U == count) || (count > 7U))
    {
        return false;
    }

    T_I2C_EXEC transfer = {0};
    SZ         actual   = 0;

    transfer.sadr     = address;
    transfer.snd_size = 1;
    transfer.snd_data = &reg;
    /* Status plus XYZ is at most seven bytes, which fits kernel SZ. */
    transfer.rcv_size = (SZ)count;
    transfer.rcv_data = p_data;

    ER error_code = tk_swri_dev(gh_i2c_device, TDN_I2C_EXEC, &transfer,
                                sizeof(transfer), &actual);

    return (E_OK == error_code) &&
           ((SZ)sizeof(transfer) == actual); /* Fixed B4 message sizes fit the
                                                kernel signed size type. */
}

/** @brief Write a register and its value through the kernel I2C driver. */
static bool
write_register (void * p_context, uint8_t address, uint8_t reg, uint8_t value)
{
    /* This bus adapter needs no caller-owned callback context. */
    (void)p_context;

    UB bytes[2] = {reg, value};
    SZ actual   = 0;
    ER error_code =
        tk_swri_dev(gh_i2c_device, address, bytes, sizeof(bytes), &actual);

    return (E_OK == error_code) &&
           ((SZ)sizeof(bytes) == actual); /* Fixed B4 message sizes fit the
                                             kernel signed size type. */
}

/*** end of file ***/

