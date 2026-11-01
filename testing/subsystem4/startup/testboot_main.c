/** @file testboot_main.c
 * @brief Test actual B4 startup, readiness and kernel error propagation.
 */
#include "testboot_main.h"
#include "subsystem4.h"
#include "s4_runtime.h"
#include "task_imu.h"
#include "task_monitor.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t g_buffers;
static uint32_t g_tasks;
static uint32_t g_monitor_tasks;
static uint32_t g_alarm_calls;
static UINT     g_flags;
static ER       g_buffer_error;
static ER       g_flag_error;
static ER       g_task_error;
static ER       g_alarm_error;

/** @brief Check startup profiles, readiness persistence and injected failures. */
int
main (void)
{
#if ROBOT_MODE && S4_IMU_USE_MOCK
    assert(E_PAR == subsystem4_wait_ready(100));
#else
    assert(E_OBJ == subsystem4_wait_ready(100));
#endif
    g_buffer_error = E_OBJ;
    assert(E_OBJ == subsystem4_init());
    g_buffer_error = E_OK;
    g_flag_error = E_OBJ;
    assert(E_OBJ == subsystem4_init());
    g_flag_error = E_OK;
    g_buffers = 0U;
    assert(E_OK == subsystem4_init());
#if ROBOT_MODE
    assert((0 == gh_summary) && (0 == gh_monitor_alarm));
    assert(0U == g_alarm_calls);
    assert((1U + APP_TEST) == g_buffers);
#else
    assert(gh_summary > 0);
    assert(1U == g_alarm_calls);
    assert((2U + APP_TEST) == g_buffers);
#endif
    assert(E_OK == subsystem4_start());
#if ROBOT_MODE
    assert((2U == g_tasks) && (0U == g_monitor_tasks));
#else
    assert((3U == g_tasks) && (1U == g_monitor_tasks));
#endif
#if ROBOT_MODE && S4_IMU_USE_MOCK
    assert(E_PAR == subsystem4_wait_ready(100));
    assert(E_OK == tk_set_flg(gh_events, S4_IMU_READY_FLAG));
    assert(E_PAR == subsystem4_wait_ready(100));
#else
    assert(E_PAR == subsystem4_wait_ready(0));
    assert(E_PAR == subsystem4_wait_ready(-1));
    assert(E_TMOUT == subsystem4_wait_ready(100));
    assert(E_OK == tk_set_flg(gh_events, S4_IMU_READY_FLAG));
    assert(E_OK == subsystem4_wait_ready(100));
    assert(E_OK == subsystem4_wait_ready(100));
    assert(E_OK == tk_clr_flg(gh_events, ~S4_IMU_READY_FLAG));
    assert(E_TMOUT == subsystem4_wait_ready(100));
#endif
    g_task_error = E_OBJ;
    assert(E_OBJ == subsystem4_start());
    puts("PASS: B4 startup profile, readiness and injected kernel failures");
    return 0;
}

/* These stub entry names match the production kernel/module ABI. The host
 * test records task creation; it never executes infinite RTOS tasks. */

/** @brief Simulate message-buffer creation with an injectable failure. */
ID
tk_cre_mbf (const T_CMBF * p_config)
{
    assert((p_config->maxmsz > 0) && (p_config->bufsz >= p_config->maxmsz));
    g_buffers++;
    return (g_buffer_error < E_OK) ? g_buffer_error : 10;
}

/** @brief Simulate flag creation and reset its initial pattern. */
ID
tk_cre_flg (const T_CFLG * p_config)
{
    g_flags = p_config->iflgptn;
    return (g_flag_error < E_OK) ? g_flag_error : 11;
}

/** @brief Verify readiness waits preserve the flag and use a bounded timeout. */
ER
tk_wai_flg (ID h_flag, UINT mask, UINT mode, UINT * p_flags, TMO timeout)
{
    assert((h_flag > 0) && (timeout > 0));
    assert((S4_IMU_READY_FLAG == mask) && (TWF_ORW == mode));
    *p_flags = g_flags;
    return (0U != (g_flags & mask)) ? E_OK : E_TMOUT;
}

/** @brief Set a simulated shared event bit. */
ER
tk_set_flg (ID h_flag, UINT mask)
{
    assert(h_flag > 0);
    g_flags |= mask;
    return E_OK;
}

/** @brief Apply the kernel's preserve-mask clear semantics. */
ER
tk_clr_flg (ID h_flag, UINT mask)
{
    assert(h_flag > 0);
    g_flags &= mask;
    return E_OK;
}

/** @brief Record task order and reject unexpected hardware ownership. */
ID
app_start_task (void (*p_entry)(INT, void *), INT priority,
                UW core, const char * p_name)
{
    assert((NULL != p_entry) && (priority > 0) && (0U == core));
    g_tasks++;
    if (1U == g_tasks)
    {
        assert(0 == strcmp(p_name, "task_imu_analysis_task"));
    }
    else if (2U == g_tasks)
    {
        assert(0 == strcmp(p_name, "task_imu_sampling_task"));
    }
    else
    {
        /* Further calls include the demo monitor or injected retries. */
    }
    if (0 == strcmp(p_name, "task_monitor_task"))
    {
        g_monitor_tasks++;
    }
    return (g_task_error < E_OK) ? g_task_error : 12;
}

/** @brief Record demo alarm creation. */
ID
task_monitor_create_alarm (void)
{
    g_alarm_calls++;
    return (g_alarm_error < E_OK) ? g_alarm_error : 13;
}

/** @brief Provide an inert entry address for host startup testing. */
void
task_imu_analysis_task (INT stacd, void * p_exinf)
{
    /* Stub arguments are intentionally unused; these casts discard them. */
    (void)stacd;
    (void)p_exinf;
}

/** @brief Provide an inert entry address for host startup testing. */
void
task_imu_sampling_task (INT stacd, void * p_exinf)
{
    /* Stub arguments are intentionally unused; these casts discard them. */
    (void)stacd;
    (void)p_exinf;
}

/** @brief Provide an inert demo monitor entry for host startup testing. */
void
task_monitor_task (INT stacd, void * p_exinf)
{
    /* Stub arguments are intentionally unused; these casts discard them. */
    (void)stacd;
    (void)p_exinf;
}

/*** end of file ***/

