/**
 * @file b3_line_cal.c
 * @brief Per-sensor auto-calibration of the analogue IR line sensors
 *        (Pololu QTRSensors scheme, see b3_line_cal.h).
 */

#include "b3_line_cal.h"

#include <stddef.h>

static void
block_reset (B3_LINE_CAL * cal)
{
    cal->block_min    = 0x0FFFU;
    cal->block_max    = 0U;
    cal->block_count  = 0U;
    cal->window_index = 0U;
    for (unsigned i = 0U; i < B3_CAL_BLOCK_SAMPLES; ++i)
    {
        cal->window[i] = 0U;
    }
}

void
b3_floor_init (B3_FLOOR * f)
{
    if (f != NULL)
    {
        f->floor_x16 = 0U;
        f->high_ms   = 0U;
        f->started   = false;
    }
}

void
b3_floor_update (B3_FLOOR * f,
    const B3_FLOOR_CFG *    cfg,
    uint16_t                raw,
    uint16_t                dt_ms,
    uint16_t *              white,
    uint16_t *              black)
{
    int32_t  fl, target;
    uint32_t w, b;

    if (f == NULL || cfg == NULL || white == NULL || black == NULL)
    {
        return;
    }
    if (!f->started)
    {
        f->floor_x16 = (uint32_t)cfg->init * 16U;
        f->started   = true;
    }
    fl     = (int32_t)(f->floor_x16 / 16U);
    target = (int32_t)raw * 16;
    if ((int32_t)raw < fl)
    {
        f->floor_x16 = (uint32_t)((int32_t)f->floor_x16 +
            (target - (int32_t)f->floor_x16) / B3_FLOOR_DOWN_DIV);
        f->high_ms   = 0U;
    }
    else if ((int32_t)raw < fl + (int32_t)cfg->band)
    {
        f->floor_x16 = (uint32_t)((int32_t)f->floor_x16 +
            (target - (int32_t)f->floor_x16) / B3_FLOOR_UP_DIV);
        f->high_ms   = 0U;
    }
    else if (raw < cfg->tape)
    {
        if (f->high_ms < 60000U)
        {
            f->high_ms = (uint16_t)(f->high_ms + dt_ms);
        }
        if (f->high_ms >= B3_FLOOR_SHIFT_MS)
        {
            f->floor_x16 = (uint32_t)((int32_t)f->floor_x16 +
                (target - (int32_t)f->floor_x16) / B3_FLOOR_UP_DIV);
        }
    }
    else
    {
        f->high_ms = 0U;
    }
    if (f->floor_x16 < (uint32_t)cfg->lo * 16U)
    {
        f->floor_x16 = (uint32_t)cfg->lo * 16U;
    }
    if (f->floor_x16 > (uint32_t)cfg->hi * 16U)
    {
        f->floor_x16 = (uint32_t)cfg->hi * 16U;
    }
    w = f->floor_x16 / 16U + cfg->margin;
    b = w + cfg->span;
    if (b < cfg->black_min)
    {
        b = cfg->black_min;
    }
    *white = (uint16_t)w;
    *black = (uint16_t)b;
}

void
b3_line_cal_init (B3_LINE_CAL * cal)
{
    if (cal != NULL)
    {
        /* Nothing learned yet: min starts at full scale, max at 0. */
        cal->white = 0x0FFFU;
        cal->black = 0U;
        block_reset(cal);
    }
}

bool
b3_line_cal_ready (const B3_LINE_CAL * cal)
{
    return cal != NULL && cal->black > cal->white &&
        (uint32_t)cal->black - cal->white >= B3_CAL_MIN_SPAN;
}

uint16_t
b3_line_cal_scale (uint16_t raw, uint16_t white, uint16_t black)
{
    if ((black <= white) || (raw <= white))
    {
        return 0U;
    }
    if (raw >= black)
    {
        return (uint16_t)B3_CAL_FULL_SCALE;
    }
    return (uint16_t)(((uint32_t)raw - white) * B3_CAL_FULL_SCALE /
        ((uint32_t)black - white));
}

uint16_t
b3_line_cal_update (B3_LINE_CAL * cal, uint16_t raw)
{
    uint32_t white;
    uint32_t black;

    if (cal == NULL)
    {
        return 0U;
    }
    raw &= 0x0FFFU;

    /* Confirm extremes with a rolling window, rather than disjoint blocks
     * that can split three real surface samples across two blocks. */
    cal->window[cal->window_index] = raw;
    cal->window_index =
        (uint8_t)((cal->window_index + 1U) % B3_CAL_BLOCK_SAMPLES);
    if (cal->block_count < B3_CAL_BLOCK_SAMPLES)
    {
        ++cal->block_count;
    }
    if (cal->block_count >= B3_CAL_BLOCK_SAMPLES)
    {
        cal->block_min = 0x0FFFU;
        cal->block_max = 0U;
        for (unsigned i = 0U; i < B3_CAL_BLOCK_SAMPLES; ++i)
        {
            if (cal->window[i] < cal->block_min)
            {
                cal->block_min = cal->window[i];
            }
            if (cal->window[i] > cal->block_max)
            {
                cal->block_max = cal->window[i];
            }
        }
        if (cal->block_min > cal->black)
        {
            cal->black = cal->block_min;
        }
        if (cal->block_max < cal->white)
        {
            cal->white = cal->block_max;
        }
    }

    if (b3_line_cal_ready(cal))
    {
        white = cal->white;
        black = cal->black;
    }
    else
    {
        white = B3_CAL_DEFAULT_WHITE;
        black = B3_CAL_DEFAULT_BLACK;
    }

    /* Pololu readCalibrated(): scale to 0..1000 and clamp. */
    if (raw <= white)
    {
        return 0U;
    }
    if (raw >= black)
    {
        return (uint16_t)B3_CAL_FULL_SCALE;
    }
    return (uint16_t)(((uint32_t)raw - white) * B3_CAL_FULL_SCALE /
        (black - white));
}

/*** end of file ***/

