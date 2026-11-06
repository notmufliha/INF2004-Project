# Buddy 5 verification - 6 November 2026

Current default: start with line following, avoid a valid profiled obstacle,
reacquire the original line, restore heading and resume following. The next
obstacle starts a new avoidance cycle. `ROBOT_ENABLE_BYPASS=1` selects this
flow and the encoder PI controller; no calibration override is needed to build.
`ROBOT_BYPASS_CALIBRATED` remains 0 because hardware calibration is unverified.

| Verification | Result |
|---|---|
| `python testing/subsystem5/run_complete_tests.py` | PASS, 14 groups including 15 controller/profile checks |
| Startup off-line, including a close obstacle | PASS, waits without driving and without an original-line reference |
| Startup on-line with confirmed clearance | PASS, establishes the line reference and starts following |
| Bypass, original-line recovery and heading restoration | PASS in maneuver and B2/B3 adapter host tests |
| Resume following, then encounter a second obstacle | PASS, resumes DRIVE then stops and starts another avoidance cycle |
| Default autonomous USB firmware, SMP=0 | PASS, ELF linked and UF2 generated from checked-in configuration |
| Default autonomous USB firmware, SMP=1 | PASS, ELF linked and UF2 generated from checked-in configuration |
| Legacy forward/STOP profile remains selectable | PASS, compile-time profile checks with ROBOT_ENABLE_BYPASS=0 |
| `git diff --check` | PASS |
| Flashing and physical floor tests | Not performed |

Firmware builds used Pico SDK 2.2.0 with its pinned TinyUSB submodule,
CONSOLE=usb_cdc, CRASH_REPORT=0 and WIFI=none. No autonomous or calibration
compiler overrides were used for the current standard images. The final
object tree uses the default single-core autonomous USB configuration.

Current artifacts:

- `build_make/mtk3pico_smp0_usb_cdc.uf2`
- `build_make/mtk3pico_smp1_usb_cdc.uf2`

Earlier `*_b5_autonomous_validation.uf2` images are previous build-check
artifacts. Detailed current build logs are retained in the ignored files
`.codex_tmp/b5_line_follow_default_smp0.log` and
`.codex_tmp/b5_line_follow_default_smp1.log`.

Other behavioral coverage includes size-dependent clearance, clipped fine
scan fallback, oversized/malformed silhouettes, servo/recenter failure,
command IDs and cancellation, stale line/motor data, invalid front echoes,
bounded search and deadlines, duplicate/junction/early-line rejection,
independent rear permission, timer rollover, B3 ADC pairs, packed profiles,
B1 telemetry and the legacy guarded-drive code.

Physical calibration and obstacle-course validation remain unverified.
Visible width cannot measure hidden depth: the configured course-depth bound,
IR threshold, chassis dimensions and encoder turn constants need physical
checks. Unknown or unsafe routes stop. No rear sensor is defined, so the
default adapter rejects reverse. See
`app_program/subsystem5/README_BUDDY5.md` for configuration and limitations.
