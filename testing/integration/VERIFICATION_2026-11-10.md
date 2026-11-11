# Host verification, 2026-11-10

Source: working tree based on `62467ca5a8e84b14ebbcc3d1cef91a188af7dbb2`,
including the uncommitted line calibration correction and test runner.
The assignment PDF was unavailable; requirement compliance remains pending.

Update after the PDF was supplied: the ten-page briefing has now been reviewed;
see [requirement mapping](../../BRIEFING_COMPLIANCE.md). The original host run
below remains unchanged. Physical calibration and demonstration evidence are
still pending.

[Saved host transcript](logs/2026-11-10_143429_host.txt).

| Check | Result |
|---|---|
| B1 reconnect policy | 47 checks, zero failures |
| B1 C/Python wire contract | 20 frames; 14 valid and 10 invalid commands passed |
| Dashboard decoder | 20 frames, zero failures |
| B2 motion | 1,239 assertions passed |
| B2 legacy/binary integration | Both passed |
| Vehicle Controller | Passed commands, curves, B5 handoff, boot readiness, hazards and adapter tests |
| Code 39 | Passed full table, both directions/polarities, drift, chatter, wrap, rejection |
| IR calibration | S3-C01 through S3-C06 passed |
| B5 | All 16 host test groups passed |
| USB-only firmware | Compiled, linked and converted to UF2 |
| WiFi/lwIP/MQTT firmware | Compiled, linked and converted to UF2 |

Firmware SHA256:

```text
mtk3pico_smp0_usb_cdc.uf2
D4FE916765D040BFF7F9C0105769C23893378F505D7B2273DA0DDD2FE4691116

mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2
9BE8C51A694DAC02AC28506B25DF9AF8F584E5B35CCDDF9F021216807C1EC801
```

The linker reports unimplemented newlib syscall stub warnings. Build exit
status was zero; these are not a warning-free build claim. USB console uses
the project's own console path. No COM port was visible, so no firmware was
flashed and no hardware result is claimed. Physical wheel/encoder calibration,
IR polarity, tracking accuracy, stopping distance and bypass still need the
tests in the root `ROBOT_TEST_STEPS.md`.
