/*
 * Buddy 2 - Quadrature Encoder Feedback
 * RP2040 + micro T-Kernel 3.0
 *
 * GPIO bank IRQ is shared through the application dispatcher.
 */

#include <stdint.h>
#include "encoder_control.h"
#include "../gpio_irq_dispatch.h"
#if TK_SUPPORT_SMP

#include <tk/smp.h>
#endif
#define ENC_L_A  0U
#define ENC_L_B  1U
#define ENC_R_A  2U
#define ENC_R_B  3U
/* RP2040 IO_BANK0 registers. */
#define ENC_IO_BASE        0x40014000U
#define ENC_INTR0          (*(volatile UW *)(ENC_IO_BASE + 0x0F0U))
#define ENC_PROC0_INTE0    (*(volatile UW *)(ENC_IO_BASE + 0x100U))
#define ENC_PROC0_INTS0    (*(volatile UW *)(ENC_IO_BASE + 0x120U))
/* Each GPIO has four interrupt bits.
 * Bit 2: falling edge
 * Bit 3: rising edge
 */
#define ENC_EDGE_MASK(pin) (0xCU << ((pin) * 4U))
#define ENC_ALL_EDGES (ENC_EDGE_MASK(ENC_L_A) | \
                      ENC_EDGE_MASK(ENC_L_B) | \
                      ENC_EDGE_MASK(ENC_R_A) | \
                      ENC_EDGE_MASK(ENC_R_B))

static volatile INT g_left_count = 0;
static volatile INT g_right_count = 0;
static UINT g_left_previous = 0;
static UINT g_right_previous = 0;
#if TK_SUPPORT_SMP

static T_SPLOCK g_encoder_lock;
#endif

static void encoder_lock_enter(UINT *p_status);
static void encoder_lock_leave(UINT status);
static UINT encoder_read_state(UINT pin_a, UINT pin_b);
static INT encoder_decode(UINT previous, UINT current);
static void encoder_gpio_isr(UINT intno);

/** @brief Initialize GPIO inputs and register the encoder interrupt. */
ER encoder_init(void)
{
    ER err;
#if TK_SUPPORT_SMP
    (void)InitSpinLock(&g_encoder_lock);
#endif

    err = gpio_set_pin(ENC_L_A, GPIO_MODE_IN);

    if (E_OK != err)
    {
        return (err);
    }
    err = gpio_set_pin(ENC_L_B, GPIO_MODE_IN);

    if (E_OK != err)
    {
        return (err);
    }
    err = gpio_set_pin(ENC_R_A, GPIO_MODE_IN);

    if (E_OK != err)
    {
        return (err);
    }
    err = gpio_set_pin(ENC_R_B, GPIO_MODE_IN);

    if (E_OK != err)
    {
        return (err);
    }
    g_left_count = 0;
    g_right_count = 0;
    g_left_previous =
        encoder_read_state(ENC_L_A, ENC_L_B);
    g_right_previous =
        encoder_read_state(ENC_R_A, ENC_R_B);
    /* Disable our GPIO edge sources before registration. */
    ENC_PROC0_INTE0 &= ~ENC_ALL_EDGES;
    /* Clear any previously latched encoder edges. */
    ENC_INTR0 = ENC_ALL_EDGES;
    err = app_gpio_irq_register(0x0Fu, encoder_gpio_isr);

    if (E_OK != err)
    {
        tm_printf((UB *)"[B2] Encoder IRQ registration failed\n");
        return (err);
    }
    /* Enable rising/falling edges on encoder pins. */
    ENC_PROC0_INTE0 |= ENC_ALL_EDGES;

    tm_printf((UB *)"[B2] Encoder interrupts initialized\n");
    return (E_OK);
}

/** @brief Read both encoder counters under the interrupt lock. */
void encoder_get_counts(INT *p_left, INT *p_right)
{
    UINT interrupt_status;

    encoder_lock_enter(&interrupt_status);
    if (NULL != p_left)
    {
        *p_left = g_left_count;
    }
    if (NULL != p_right)
    {
        *p_right = g_right_count;
    }
    encoder_lock_leave(interrupt_status);
}

/** @brief Calculate signed wheel travel in centimetres. */
void encoder_get_distance_cm(
    INT left_start,
    INT right_start,
    float *p_left_cm,
    float *p_right_cm
)
{
    INT left_now;
    INT right_now;
    int64_t left_delta;
    int64_t right_delta;

    encoder_get_counts(&left_now, &right_now);

    left_delta = (int64_t)left_now - (int64_t)left_start;
    right_delta = (int64_t)right_now - (int64_t)right_start;

    if (NULL != p_left_cm)
    {
        *p_left_cm =
            ((float)left_delta * 3.14159265f *
             ENC_WHEEL_DIAMETER_CM) /
            ENC_LEFT_COUNTS_PER_REV;
    }

    if (NULL != p_right_cm)
    {
        *p_right_cm =
            ((float)right_delta * 3.14159265f *
             ENC_WHEEL_DIAMETER_CM) /
            ENC_RIGHT_COUNTS_PER_REV;
    }
}

/** @brief Report encoder counts and estimated wheel distance. */
void encoder_test_task(INT stacd, void *p_exinf)
{
    INT left_start;
    INT right_start;
    INT left_now;
    INT right_now;
    float left_cm;
    float right_cm;

    (void)stacd;
    (void)p_exinf;

    tk_dly_tsk(3000);

    /* Record the starting encoder counts. */
    encoder_get_counts(&left_start, &right_start);

    tm_printf((UB *)"[B2] Distance calibration test starting\n");

    for (;;)
    {
        encoder_get_counts(&left_now, &right_now);

        encoder_get_distance_cm(
            left_start,
            right_start,
            &left_cm,
            &right_cm
        );

        tm_printf(
            (UB *)"[B2] Encoder L=%d R=%d\n",
            left_now,
            right_now
        );

        /*
         * Print millimetres as integers because embedded
         * tm_printf may not support floating-point formatting.
         */
        tm_printf(
            (UB *)"[B2] Distance L=%d R=%d mm\n",
            (INT)(left_cm * 10.0f),
            (INT)(right_cm * 10.0f)
        );

        tk_dly_tsk(500);
    }
}

/** @brief Measure speed over 100 ms and report every 500 ms. */
void encoder_speed_task(INT stacd, void *p_exinf)
{
    INT left_now;
    INT right_now;
    INT left_previous_count;
    INT right_previous_count;
    INT left_speed = 0;
    INT right_speed = 0;
    INT sample_counter = 0;
    SYSTIM previous_time;
    SYSTIM current_time;
    int64_t elapsed_ms;
    int64_t left_delta;
    int64_t right_delta;

    (void)stacd;
    (void)p_exinf;
    encoder_get_counts(
        &left_previous_count,
        &right_previous_count
    );
    if (E_OK != tk_get_tim(&previous_time))
    {
        tm_printf((UB *)"[B2] Failed to read system time\n");
        return;
    }
    tm_printf((UB *)"[B2] Improved speed task started\n");
    for (;;)
    {
        /* Request a 10 ms sampling interval. */
        tk_dly_tsk(10);
        sample_counter++;
        /*
         * Calculate speed over approximately 100 ms
         * instead of using individual 10 ms samples.
         */
        if (sample_counter >= 10)
        {
            encoder_get_counts(&left_now, &right_now);
            if (E_OK != tk_get_tim(&current_time))
            {
                continue;
            }
            elapsed_ms =
                ((int64_t)current_time.hi * INT64_C(4294967296)
                    + (int64_t)current_time.lo)
                -
                ((int64_t)previous_time.hi * INT64_C(4294967296)
                    + (int64_t)previous_time.lo);
            if (elapsed_ms > 0)
            {
                left_delta =
                    (int64_t)left_now - left_previous_count;
                right_delta =
                    (int64_t)right_now - right_previous_count;
                left_speed =
                    (INT)((left_delta * 1000) / elapsed_ms);
                right_speed =
                    (INT)((right_delta * 1000) / elapsed_ms);
            }
            left_previous_count = left_now;
            right_previous_count = right_now;
            previous_time = current_time;
            sample_counter = 0;
        }
        /* Print roughly every 500 ms. */
        static INT print_counter = 0;
        print_counter++;
        if (print_counter >= 50)
        {
            tm_printf(
                (UB *)"[B2] Avg Speed L=%d R=%d counts/s\n",
                left_speed,
                right_speed
            );
            print_counter = 0;
        }
    }
}

static void encoder_lock_enter(UINT *p_status)
{
#if TK_SUPPORT_SMP
    (void)ISpinLock(&g_encoder_lock, p_status);
#else
    DI(*p_status);
#endif
}

static void encoder_lock_leave(UINT status)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&g_encoder_lock, status);
#else
    EI(status);
#endif
}

/** @brief Read both encoder channels as a two-bit state. */
static UINT encoder_read_state(UINT pin_a, UINT pin_b)
{
    UINT channel_a = gpio_get_val(pin_a);
    UINT channel_b = gpio_get_val(pin_b);
    return ((channel_a << 1) | channel_b);
}

/** @brief Decode valid quadrature transitions. */
static INT encoder_decode(UINT previous, UINT current)
{
    static const int8_t table[16] =
        {
        0, -1,  1,  0,
        1,  0,  0, -1,
        -1,  0,  0,  1,
        0,  1, -1,  0
    };
    return (table[(previous << 2) | current]);
}

/** @brief Acknowledge GPIO edges and update quadrature counters. */
static void encoder_gpio_isr(UINT intno)
{
    UW pending = ENC_PROC0_INTS0 & ENC_ALL_EDGES;
    UW gpio_snapshot;
    UINT current;
    UINT interrupt_status;

    encoder_lock_enter(&interrupt_status);
    if (0U != pending)
    {
        /* Acknowledge GPIO edge events. */
        ENC_INTR0 = pending;
        /* Capture all GPIO input levels together. */
        gpio_snapshot =
            *(volatile UW *)0xD0000004U;
        if (0U != (pending & (ENC_EDGE_MASK(ENC_L_A) |
            ENC_EDGE_MASK(ENC_L_B))))
        {
            current =
                (((gpio_snapshot >> ENC_L_A) & 1U) << 1) |
                ((gpio_snapshot >> ENC_L_B) & 1U);
            g_left_count += encoder_decode(g_left_previous, current);
            g_left_previous = current;
        }
        if (0U != (pending & (ENC_EDGE_MASK(ENC_R_A) |
            ENC_EDGE_MASK(ENC_R_B))))
        {
            current =
                (((gpio_snapshot >> ENC_R_A) & 1U) << 1) |
                ((gpio_snapshot >> ENC_R_B) & 1U);
            g_right_count += encoder_decode(g_right_previous, current);
            g_right_previous = current;
        }
    }
    encoder_lock_leave(interrupt_status);
    (void)intno;
}
