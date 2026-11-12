# Black-line startup regression

The supplied COM6 capture runs firmware built at 23:17:46. It shows valid
low-is-black raw readings being discarded by the old 300-count cutoff:
GP26=242, GP27=156 becomes norm26=0, norm27=0. Earlier GP26=350,
GP27=166 becomes norm26=1000, norm27=0, commanding a right turn.

Changed `B3_LINE_RAW_LOST_BELOW` from 300 to 1. With the configured
low-is-black polarity, nonzero readings below 300 now represent black
instead of lost line. Zero remains excluded. The existing fixed 1500/1999
black/white endpoints are unchanged; white-surface measurements are still
needed to validate these endpoints on the actual course.

Regression tests replay six raw sensor pairs from the capture, assert
balanced black detection, retain white/zero/ADC-failure coverage, and verify
that the controller commands equal positive wheel targets on balanced black
without a queued navigation instruction. The complete host runner passed;
the controller suite passed again after adding the startup assertion.

The network firmware compiled and generated
`build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
SHA256: `3E55EC979A99F76615CB1EC874A27C1C99D08A2F7C73BD929A4DDCBB67F58CEC`.

The capture also shows a separate ultrasonic stop: repeated 31–34 cm
returns, a scan with no observable clear boundary, then `FINE_SCAN` and
`SAFE_STOP`. This change does not authorize driving through that range.
Reboot with the corrected firmware and clear more than 45 cm ahead for
the startup test. If the same close echoes occur in physically open space,
check sensor mounting and the source of those echoes; do not label them
false or disable obstacle stopping based on this log alone.

Host results do not certify physical tracking or obstacle clearance.
