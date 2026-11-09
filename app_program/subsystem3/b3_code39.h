#ifndef B3_CODE39_H
#define B3_CODE39_H

#include <stdbool.h>
#include <stdint.h>

#define B3_CODE39_RUN_COUNT (29U)

typedef struct {
    uint32_t widths_us[B3_CODE39_RUN_COUNT];
    uint8_t levels[B3_CODE39_RUN_COUNT];
    uint32_t run_start_us;
    uint8_t run_count;
    uint8_t current_level;
    uint8_t have_level;
    uint8_t in_barcode;
    uint8_t payload_valid;
    char payload;
} B3_CODE39_DECODER;

void b3_code39_reset(B3_CODE39_DECODER *decoder);
bool b3_code39_feed(B3_CODE39_DECODER *decoder, uint8_t level,
                    uint32_t timestamp_us, char *symbol);

#endif
