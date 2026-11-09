/**
 * @file    barcode.h
 * @brief   Barcode task entry points and diagnostics.
 * @owner   Buddy 3 (line and barcode)
 * @kind    task
 */

#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

extern ER   barcode_init(void);
extern ER   barcode_start(void);
extern uint32_t barcode_edge_count(void); /* accepted DO edges, wraps    */
extern uint32_t barcode_do_level(void);   /* DO now: 1 = black, 0 = white */

#endif /* BARCODE_H */

/*** end of file ***/
