# Buddy 5 only: Barr-C-oriented changes and Tribunal CR1

**Scope:** This patch modifies ONLY `app_program/subsystem5/` and adds host tests under `testing/subsystem5/`. It does not change Buddy 1–4, the vehicle controller, global configuration, build scripts, or shared fault handling.

## Implemented

- Explicit 42-bit B5 data payload: obstacle distance (9 bits), width (7), left clearance (9), right clearance (9), servo angle (8). A four-bit protocol version and two padding bits produce a six-byte message. The byte order and bit order are defined explicitly, not dependent on C compiler bit-field layout.
- Reserved invalid codes: 511 for distances/clearances, 127 for width, 255 for servo angle. Valid ranges remain 0–400 cm, 0–100 cm, and 0–180 degrees.
- B5 packs a completed obstacle profile after scanning; the bytes are currently printed for inspection. No B1 MQTT publisher or team-wide queue is implemented here.
- B5-owned local two-bit fault flags (no echo and invalid clearance), updated by the B5 task alone. **This is NOT the team shared eight-bit fault mask**. Integrating the shared mask would require an agreed team interface and changes outside B5, which are intentionally excluded.
- Barr-C-oriented style cleanup in selected B5 source files; not a complete certification of every line against a particular Barr-C edition.
- Existing safe forward/stop behaviour is retained. Missing/invalid ultrasonic measurements still prevent movement; physical bypass/line reacquisition remain disabled pending B2/B3 integration.

## Tests

From the project root:

```bash
bash testing/subsystem5/run_host_tests.sh
bash testing/subsystem5/run_packed_tests.sh
```

To build on your own WSL environment, run your team **unchanged** build command from `build_make/`. The Make configuration uses a wildcard for application `.c` files, so it should compile the new `s5_packed.c` without changing build scripts. Confirm with a clean build.

## Required Week 13 evidence not fabricated

- Measure **before/after** RAM from the two actual linker map files on the same toolchain; the six-byte payload is a wire format, not automatically six bytes of total RAM.
- Compare six-byte packed B5 message with the old application message format actually used by your team (if one exists).
- Record measured timing and robot behaviour at known distances, including cold boot without BOOTSEL and invalid echo.
- Confirm the final shared fault-bit allocation with the vehicle-controller owner; local B5 fault flags are not a substitute for the shared interface.

## Install

Copy only `app_program/subsystem5/` and, optionally, `testing/subsystem5/` from the patch into your team current project. Do not replace files from other subsystems.
