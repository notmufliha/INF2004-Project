/** @file fault.c
 * @brief Shared fault bitmask (see fault.h).  flg_faults is created in
 * subsystem1_init().  The kernel updates the pattern atomically, so
 * these functions are safe from any task or interrupt handler.
 */

#include "fault.h"

ID flg_faults = 0; /* 0 until subsystem1_init() creates it */

/** @brief Set the given fault bit(s); other bits are untouched. */
void
fault_raise (uint32_t bits)
{
    if (flg_faults > 0)
    {
        (void)tk_set_flg(flg_faults, bits & FAULT_ALL);
    }
}

/** @brief Clear the given fault bit(s); other bits are untouched. */
void
fault_clear (uint32_t bits)
{
    if (flg_faults > 0)
    {
        /* tk_clr_flg() ANDs the pattern with its argument. */
        (void)tk_clr_flg(flg_faults, ~(bits & FAULT_ALL));
    }
}

/** @brief Current mask (8 bits); 0 if the flag does not exist yet. */
uint8_t
fault_get (void)
{
    T_RFLG  rflg;
    uint8_t mask = 0U;

    if ((flg_faults > 0) && (E_OK == tk_ref_flg(flg_faults, &rflg)))
    {
        /* Masked to 8 bits first, so the narrowing keeps every bit. */
        mask = (uint8_t)(rflg.flgptn & FAULT_ALL);
    }

    return (mask);
}

/*** end of file ***/

