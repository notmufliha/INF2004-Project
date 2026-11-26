/**
 * @file b3_buzzer.c
 * @brief Barcode feedback beeps on the Robo Pico piezo buzzer (GP22).
 */
#include "b3_buzzer.h"

#include <stdbool.h>

#include "../app.h"
#include <tk/syslib.h>
#include <sys/sysdef.h>

#if B3_BUZZER_ENABLE

/* 125 MHz system clock / 4 = 31.25 MHz PWM tick (DIV is 8.4 fixed point).
 * GP22 is PWM slice 3 A; nothing else uses slice 3 (TEAM.md pins). */
#define B3_BUZZER_DIV (4U)
#define B3_BUZZER_TICK_HZ (125000000U / B3_BUZZER_DIV)
#define B3_BUZZER_HIGH_HZ (2700U) /* near the piezo's loudest point */
#define B3_BUZZER_LOW_HZ (1000U)

typedef struct
{
    uint16_t hz; /* 0 = silence */
    uint16_t duration_ms;
} B3_TONE;

static const B3_TONE b3_read_tones[]   = {{B3_BUZZER_HIGH_HZ, 150U}};
static const B3_TONE b3_reject_tones[] = {
    {B3_BUZZER_LOW_HZ, 80U}, {0U, 80U}, {B3_BUZZER_LOW_HZ, 80U}};
static const B3_TONE    b3_junction_tones[] = {{B3_BUZZER_HIGH_HZ, 50U},
       {0U, 50U},
       {B3_BUZZER_HIGH_HZ, 50U},
       {0U, 50U},
       {B3_BUZZER_HIGH_HZ, 50U}};
static volatile uint8_t g_request; /* B3_BEEP + 1, 0 = none */

static const B3_TONE * g_tones;
static uint8_t         g_tone_count;
static uint8_t         g_tone_index;
static uint32_t        g_tone_start_us;
static bool            g_ready;

static void
b3_buzzer_tone (uint16_t hz)
{
    if (hz == 0U)
    {
        (void)pwm_set_cc(B3_BUZZER_PIN, 0U);
        return;
    }
    {
        uint32_t wrap = B3_BUZZER_TICK_HZ / hz - 1U;

        (void)pwm_set_wrap(B3_BUZZER_PIN, wrap);
        (void)pwm_set_cc(B3_BUZZER_PIN, (wrap + 1U) / 2U); /* square wave */
    }
}

void
b3_buzzer_init (void)
{
    uint32_t slice = (B3_BUZZER_PIN >> 1U) & 7U;
    ER       error = pwm_set_enabled(B3_BUZZER_PIN, FALSE);

    out_w(PWM_BASE + slice * 0x14U + PWM_CHx_DIV, B3_BUZZER_DIV << 4U);
    if (error == E_OK)
    {
        error = pwm_set_wrap(B3_BUZZER_PIN, 0xFFFFU);
    }
    if (error == E_OK)
    {
        error = pwm_set_cc(B3_BUZZER_PIN, 0U);
    }
    if (error == E_OK)
    {
        error = pwm_set_pin(B3_BUZZER_PIN);
    }
    if (error == E_OK)
    {
        error = pwm_set_enabled(B3_BUZZER_PIN, TRUE);
    }
    g_ready      = error == E_OK;
    g_tone_count = 0U;
    tm_printf(
        (UB *)"[B3] buzzer GP%d %s (beep = barcode read, 2 low = rejected)\n",
        (int)B3_BUZZER_PIN,
        g_ready ? "ready" : "init FAILED");
}

void
b3_buzzer_beep (B3_BEEP pattern, uint32_t now_us)
{
    if (!g_ready)
    {
        return;
    }
    if (pattern == B3_BEEP_READ)
    {
        g_tones = b3_read_tones;
        g_tone_count =
            (uint8_t)(sizeof(b3_read_tones) / sizeof(b3_read_tones[0]));
    }
    else if (pattern == B3_BEEP_REJECT)
    {
        g_tones = b3_reject_tones;
        g_tone_count =
            (uint8_t)(sizeof(b3_reject_tones) / sizeof(b3_reject_tones[0]));
    }
    else
    {
        g_tones = b3_junction_tones;
        g_tone_count =
            (uint8_t)(sizeof(b3_junction_tones) / sizeof(b3_junction_tones[0]));
    }
    g_tone_index    = 0U;
    g_tone_start_us = now_us;
    b3_buzzer_tone(g_tones[0].hz);
}

void
b3_buzzer_request (B3_BEEP pattern)
{
    g_request = (uint8_t)((uint8_t)pattern + 1U);
}

void
b3_buzzer_poll (uint32_t now_us)
{
    if (g_request != 0U)
    {
        uint8_t request = g_request;

        g_request = 0U;
        b3_buzzer_beep((B3_BEEP)(request - 1U), now_us);
    }
    if (!g_ready || g_tone_count == 0U)
    {
        return;
    }
    if (now_us - g_tone_start_us <
        (uint32_t)g_tones[g_tone_index].duration_ms * 1000U)
    {
        return;
    }
    ++g_tone_index;
    g_tone_start_us = now_us;
    if (g_tone_index >= g_tone_count)
    {
        g_tone_count = 0U;
        b3_buzzer_tone(0U);
        return;
    }
    b3_buzzer_tone(g_tones[g_tone_index].hz);
}

#else

void
b3_buzzer_init (void)
{
}

void
b3_buzzer_beep (B3_BEEP pattern, uint32_t now_us)
{
    (void)pattern;
    (void)now_us;
}

void
b3_buzzer_poll (uint32_t now_us)
{
    (void)now_us;
}

void
b3_buzzer_request (B3_BEEP pattern)
{
    (void)pattern;
}

#endif

/*** end of file ***/

