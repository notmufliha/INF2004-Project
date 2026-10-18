/*
 * Buddy 2 - Quadrature Encoder Feedback
 * RP2040 + micro T-Kernel 3.0
 *
 * Standalone encoder test:
 * GPIO bank IRQ owned temporarily by Buddy 2.
 */

#include "../app.h"
#include "encoder_control.h"

#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#define ENC_L_A  0U
#define ENC_L_B  1U
#define ENC_R_A  2U
#define ENC_R_B  3U

/* RP2040 GPIO bank interrupt: external IRQ 13. */
#define ENC_GPIO_IRQ       13U
#define ENC_IRQ_PRIORITY   2

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

static volatile INT left_count = 0;
static volatile INT right_count = 0;

static UINT left_previous = 0;
static UINT right_previous = 0;

#if TK_SUPPORT_SMP
static T_SPLOCK encoder_lock;
#endif

#if TK_SUPPORT_SMP
#define ENCODER_LOCK(sts)   ((void)ISpinLock(&encoder_lock, &(sts)))
#define ENCODER_UNLOCK(sts) ((void)ISpinUnlock(&encoder_lock, (sts)))
#else
#define ENCODER_LOCK(sts)   DI(sts)
#define ENCODER_UNLOCK(sts) EI(sts)
#endif

/* Read both encoder channels as a two-bit state. */
static UINT encoder_read_state(UINT pin_a, UINT pin_b)
{
    UINT a = gpio_get_val(pin_a);
    UINT b = gpio_get_val(pin_b);

    return (a << 1) | b;
}

/* Decode valid quadrature transitions. */
static INT encoder_decode(UINT previous, UINT current)
{
    static const signed char table[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    return table[(previous << 2) | current];
}

/* GPIO interrupt handler. */
static void encoder_gpio_isr(UINT intno)
{
    UW pending = ENC_PROC0_INTS0 & ENC_ALL_EDGES;
    UW gpio_snapshot;
    UINT current;
    UINT locksts;

    ENCODER_LOCK(locksts);

    if (pending != 0U) {

        /* Acknowledge GPIO edge events. */
        ENC_INTR0 = pending;

        /* Capture all GPIO input levels together. */
        gpio_snapshot =
            *(volatile UW *)0xD0000004U;

        if (pending & (ENC_EDGE_MASK(ENC_L_A) |
                       ENC_EDGE_MASK(ENC_L_B))) {

            current =
                (((gpio_snapshot >> ENC_L_A) & 1U) << 1) |
                 ((gpio_snapshot >> ENC_L_B) & 1U);

            left_count += encoder_decode(left_previous, current);

            left_previous = current;
        }

        if (pending & (ENC_EDGE_MASK(ENC_R_A) |
                       ENC_EDGE_MASK(ENC_R_B))) {

            current =
                (((gpio_snapshot >> ENC_R_A) & 1U) << 1) |
                 ((gpio_snapshot >> ENC_R_B) & 1U);

            right_count += encoder_decode(right_previous, current);

            right_previous = current;
        }
    }
    ENCODER_UNLOCK(locksts);

    ClearInt(intno);
}

/* Initialise GPIO inputs and interrupts. */
ER encoder_init(void)
{
    ER err;

#if TK_SUPPORT_SMP
    (void)InitSpinLock(&encoder_lock);
#endif

    const T_DINT dint = {
        .intatr = TA_HLNG,
        .inthdr = encoder_gpio_isr
    };

    err = gpio_set_pin(ENC_L_A, GPIO_MODE_IN);
    if (err != E_OK) return err;

    err = gpio_set_pin(ENC_L_B, GPIO_MODE_IN);
    if (err != E_OK) return err;

    err = gpio_set_pin(ENC_R_A, GPIO_MODE_IN);
    if (err != E_OK) return err;

    err = gpio_set_pin(ENC_R_B, GPIO_MODE_IN);
    if (err != E_OK) return err;

    left_count = 0;
    right_count = 0;

    left_previous =
        encoder_read_state(ENC_L_A, ENC_L_B);

    right_previous =
        encoder_read_state(ENC_R_A, ENC_R_B);

    /* Disable our GPIO edge sources before registration. */
    ENC_PROC0_INTE0 &= ~ENC_ALL_EDGES;

    /* Clear any previously latched encoder edges. */
    ENC_INTR0 = ENC_ALL_EDGES;

    err = tk_def_int(ENC_GPIO_IRQ, &dint);

    if (err != E_OK) {
        tm_printf((UB*)"[B2] Encoder IRQ registration failed\n");
        return err;
    }

    ClearInt(ENC_GPIO_IRQ);

    /* Enable GPIO bank IRQ on core 0. */
    EnableInt(ENC_GPIO_IRQ, ENC_IRQ_PRIORITY);

    /* Enable rising/falling edges on encoder pins. */
    ENC_PROC0_INTE0 |= ENC_ALL_EDGES;

    tm_printf((UB*)"[B2] Encoder interrupts initialized\n");

    return E_OK;
}

/* Atomically read the encoder counters. */
void encoder_get_counts(INT *left, INT *right)
{
    UINT locksts;

    ENCODER_LOCK(locksts);

    if (left != NULL) {
        *left = left_count;
    }

    if (right != NULL) {
        *right = right_count;
    }

    ENCODER_UNLOCK(locksts);
}

/* Temporary manual wheel-rotation test. */
void encoder_test_task(INT stacd, void *exinf)
{
    INT left;
    INT right;

    (void)stacd;
    (void)exinf;

    tk_dly_tsk(3000);

    tm_printf((UB*)"[B2] Encoder test starting\n");

    while (1) {
        encoder_get_counts(&left, &right);

        tm_printf((UB*)"[B2] Encoder L=%d R=%d\n",
                  left, right);

        tk_dly_tsk(500);
    }
}


/*
 * Buddy 2 - Wheel Speed Measurement Task
 *
 * Nominal sampling interval: 10 ms (100 Hz)
 * Serial output interval: 500 ms
 *
 * Speed unit: encoder counts per second
 */
void encoder_speed_task(INT stacd, void *exinf)
{
    INT left_now;
    INT right_now;

    INT left_previous_count;
    INT right_previous_count;

    INT left_delta;
    INT right_delta;

    INT left_speed;
    INT right_speed;

    INT print_counter = 0;

    (void)stacd;
    (void)exinf;

    /* Read initial encoder counts. */
    encoder_get_counts(
        &left_previous_count,
        &right_previous_count
    );

    tm_printf((UB*)"[B2] Wheel speed task started\n");

    while (1) {

        /* Nominal 10 ms sampling delay. */
        tk_dly_tsk(10);

        /* Read current encoder counts. */
        encoder_get_counts(&left_now, &right_now);

        /* Calculate encoder movement. */
        left_delta = left_now - left_previous_count;
        right_delta = right_now - right_previous_count;

        /* Convert counts/10 ms to counts/second. */
        left_speed = left_delta * 100;
        right_speed = right_delta * 100;

        /* Save counts for the next interval. */
        left_previous_count = left_now;
        right_previous_count = right_now;

        /* Print approximately every 500 ms. */
        print_counter++;

        if (print_counter >= 50) {

            tm_printf(
                (UB*)"[B2] Speed L=%d R=%d counts/s\n",
                left_speed,
                right_speed
            );

            print_counter = 0;
        }
    }
}

