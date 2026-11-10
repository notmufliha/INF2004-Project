/**
 * @file s5_bridge.c
 * @brief Fail-closed B5/VC/B2/B3 bridge with a real B2 emergency STOP.
 *
 * This wrapper delegates to the installed command owner. The autonomous
 * profile installs s5_navigation.c, which executes s5_maneuver.c through B2
 * and reads timestamped B3 line data. It does not bypass B2 safety gates.
 */
#include "s5_bridge.h"
#include "../subsystem2/b2_motor.h"

static S5BridgeDriver g_driver;
static bool g_driver_installed = false;

bool s5_bridge_install(const S5BridgeDriver *driver)
{
    /* This must run during startup, before any B5/VC task is started.
     * No runtime re-registration: the callback table is immutable. */
    if (g_driver_installed || (driver == NULL) ||
        (driver->ready == NULL) || (driver->request == NULL) ||
        (driver->motion_complete == NULL) ||
        (driver->line_found == NULL) ||
        (driver->emergency_stop == NULL))
    {
        return false;
    }

    g_driver = *driver;
    g_driver_installed = true;
    return true;
}

bool s5_bridge_ready(void)
{
    return g_driver_installed && g_driver.ready();
}

bool s5_bridge_request(S5Command command, const S5Profile *profile)
{
    /* B2 owns PWM. STOP is always available even without a VC callback.
     * The VC callback is additionally needed to cancel any queued motion. */
    if (command == S5_CMD_STOP || command == S5_CMD_HALT)
    {
        b2_request_stop();
        if (g_driver_installed)
        {
            return g_driver.request(command, profile);
        }
        return command == S5_CMD_STOP;
    }

    /* Never accept a bypass or a RESUME on the basis of ultrasonic rays
     * alone. An actual VC must explicitly accept each motion request. */
    if (!s5_bridge_ready())
    {
        return false;
    }

    switch (command)
    {
    case S5_CMD_REVERSE:
    case S5_CMD_GO_LEFT:
    case S5_CMD_GO_RIGHT:
    case S5_CMD_FIND_LINE:
    case S5_CMD_RESUME:
        return g_driver.request(command, profile);
    default:
        return false;
    }
}

bool s5_bridge_stopped(void)
{
    /* Read B2's real safety acknowledgement; never fake STOP completion. */
    return b2_motor_is_stopped();
}

bool s5_bridge_motion_complete(void)
{
    return s5_bridge_ready() && g_driver.motion_complete();
}

bool s5_bridge_line_found(void)
{
    return s5_bridge_ready() && g_driver.line_found();
}

void s5_bridge_emergency_stop(void)
{
    /* Always stop B2, including when the VC is absent or unresponsive. */
    b2_request_stop();
    if (g_driver_installed)
    {
        g_driver.emergency_stop();
    }
}
