/**
 * @file    cfg_barcode.h
 * @brief   Configuration constants and compile-time checks for barcode decode.
 * @owner   Buddy 3 (line, barcode)
 * @kind    cfg
 */

#ifndef CFG_BARCODE_H
#define CFG_BARCODE_H

#define BAR_DO_BLACK_IS_HIGH      (true)  /* MEASURE: A3 */
#define BAR_EDGE_REJECT_US        (1000u) /* chatter filter */
#define BAR_ELEMENTS              (29u) /* 3 chars x 9 + 2 gaps */
#define BAR_WIDE_RATIO_X10_MIN    (18u) /* wide >= 1.8 x narrow */
#define APP_C39_CHAR_COUNT        (43u) /* data characters; '*' is sentinel */

#endif /* CFG_BARCODE_H */

/*** end of file ***/