/**
 * @file b3_buzzer.h
 * @brief Barcode feedback beeps on the Robo Pico piezo buzzer (GP22).
 *
 * One high beep: a barcode was read.  Two low beeps: bars were crossed
 * but the scan was rejected.  The board's buzzer mute switch must be on.
 * Non-blocking: b3_buzzer_poll() (called every 1 ms by the barcode task)
 * ends each tone.
 */
#ifndef B3_BUZZER_H
#define B3_BUZZER_H

#include <stdint.h>

/* Set to 0 to build without the buzzer (pin left untouched). */
#ifndef B3_BUZZER_ENABLE
#define B3_BUZZER_ENABLE (1)
#endif
#define B3_BUZZER_PIN (22U)

typedef enum {
	B3_BEEP_READ = 0,   /* one 150 ms high beep */
	B3_BEEP_REJECT      /* two 80 ms low beeps  */
} B3_BEEP;

void b3_buzzer_init(void);
void b3_buzzer_beep(B3_BEEP pattern, uint32_t now_us);
void b3_buzzer_poll(uint32_t now_us);

#endif /* B3_BUZZER_H */
