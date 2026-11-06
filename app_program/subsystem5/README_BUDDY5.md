# Buddy 5 implementation

Scope: **Project Briefing.pdf, page 8**, “Adaptive Ultrasonic Scanning and
Obstacle Profiling.” The document supplies the subsystem requirements;
the implementation uses this repository's motor/sensor interfaces.

| Briefing requirement | Implementation |
|---|---|
| Coarse scan, then fine scan around obstacles | `obstacle_scan.c`: 70/80/90/100/110 degrees, then five 5-degree rays around the nearest hit, clamped to the safe mount limits |
| Estimate location, width and clearance | `s5_logic.c`: projected nearest point, visible silhouette boundaries and midpoint, small/large/oversized classification, measured side ranges and lateral clearance screening |
| Plan stop, turn or reverse actions | `s5_controller_step()` chooses STOP, a size-dependent left/right route, or one bounded reverse-and-rescan attempt; oversized/unknown profiles stop |
| Avoidance algorithm | `s5_maneuver.c` builds and executes turn-out, lateral offset, heading restoration and forward passage legs through the B2 adapter |
| Recover and reacquire the original line | Turn inward, search in bounded 2 cm steps, require three distinct fresh line samples near the expected return position, stop, restore the original heading and hand back to line following |
| Integration | `s5_navigation.c`, `s5_bridge.c`, robot startup and the B3 line sampler; B1 profile/error/state telemetry retains its existing wire format |

The narrow fine window may clip a larger object. If every fine echo is valid
but no fine silhouette is bracketed, profiling falls back to a bracketed
coarse silhouette. A failed echo cannot trigger that fallback. Objects whose
edges remain outside the complete safe scan stay unknown and stopped.

## Motion and recovery

The lateral move depends on half the measured visible width, chassis half
width, obstacle-centre offset and the clearance margin. Side rays are
projected to lateral space before screening the recommendation. Telemetry
continues to carry radial side ranges for compatibility.

Passage distance includes the measured forward obstacle location, configured
sensor offset, an explicit obstacle-depth bound, chassis half length and
margin. A frontal ultrasonic scan **cannot measure hidden obstacle depth**.
The default 40 cm depth is a provisional test-course constraint; verify or
replace it before enabling physical bypass. This implementation is for a
bounded obstacle course, not arbitrary unmeasured obstacles/environments.

B2 remains the only PWM owner. Every MOVE/TURN is correlated with its assigned
command ID. Queue acceptance, a different command's completion, cancellation,
timeout or fault cannot advance a leg. Scans require B2's current STOP
generation to be acknowledged and an encoder-confirmed physical stop.

The return search rejects stale/duplicate line samples, junctions, and
detections before departing the original line. Detection must occur within
the configured window around the predicted return distance. A final
encoder-confirmed turn restores the original heading and must still see the
line before resuming. This uses odometry and line geometry; it cannot identify
an unrelated parallel line inside the same tolerance window.

Invalid/too-close front echoes, stale B3/B2 data, failed commands, stalled
motion, excessive search travel or elapsed time stop the maneuver. Autonomous
mode always requires real front echoes, even when the guarded forward demo's
`ROBOT_NO_TARGET_FORWARD` option is enabled. No echo is unknown clearance.

Reverse is implemented as a bounded B2 MOVE and monitored every tick, but
requires a separately installed rear-clearance provider through
`s5_navigation_set_rear_sensor()`. No rear sensor is defined in this project,
so the default adapter rejects reverse and stops. A front echo never proves
rear clearance.

## Configuration and hardware

`app_program/robot_config.h` contains the size limits, chassis dimensions,
sensor offset, motion speed, depth bound, clearance margin, search window and
deadlines. `ROBOT_ENABLE_BYPASS` defaults to **1**. After sensor warm-up the
robot waits for three distinct fresh line samples, follows the line when
front clearance is confirmed, stops/scans an obstacle, executes a valid bypass,
reacquires the line and resumes following. The controller rearms for subsequent
obstacles. A robot initially placed off the line waits instead of beginning
an avoidance maneuver without an original-line reference.

Set `ROBOT_ENABLE_BYPASS=0` to select the legacy guarded forward/STOP profile.
The autonomous default selects the encoder PI controller and installs the
B2/B3 adapter. `ROBOT_BYPASS_CALIBRATED` remains zero as an honest record of
pending physical calibration; it no longer blocks compilation or selects the
startup mode. Check actual dimensions, stopping/turning behavior, course depth
bound and line threshold against the robot. Motor self-tests/demos remain
disabled. The project's `testing/subsystem2/MOTION_ACCURACY_REPORT.md` records
which physical measurements still need verification.

| Hardware | Mapping found in project sources/history |
|---|---|
| Ultrasonic trigger / echo / servo | GP16 / GP17 / GP12 |
| Safe servo travel | 70–110 degrees; verify the mount before changing |
| B3 left / centre line input | GP26 ADC0 / GP27 ADC1 |
| B3 line polarity / threshold / period | ADC <=2048 means line; 20 ms sampling |
| Motors | Existing B2 GP8/9 and GP10/11 configuration |
| Encoders | Existing B2 GP0/1 and GP2/3 configuration |

The line mapping and sampling behavior were extracted from the repository's
B3 implementation at commit `912c369`, without importing its unrelated barcode
or mission-controller changes. B3 owns the ADC and publishes complete,
timestamped pairs under its mutex. The original teaching consumer remains
available outside the autonomous profile.

Use the **USB console, CRASH_REPORT=0** for the physical robot: UART0 console
uses GP0/GP1, which conflict with these encoder pins. UART builds are compile
checks only for this wiring.

## Reproduce verification

```sh
python testing/subsystem5/run_complete_tests.py
make -C build_make -B SMP=0 CONSOLE=usb_cdc WIFI=none CRASH_REPORT=0 PICO_SDK_PATH=/path/to/pico-sdk
```

The repository specifies Pico SDK 2.2.0. During this implementation it was
downloaded with its pinned TinyUSB submodule into the ignored local directory
`.codex_tmp/pico-sdk`. In this Windows/MSYS environment the equivalent SDK
argument is `PICO_SDK_PATH=../.codex_tmp/pico-sdk`; add
`E2U=tools/elf2uf2.exe` to select the Windows converter built by make.

The 14 host test groups cover controller scenarios, size-specific routing,
coarse/fine scanning, servo failures, the actual B2/B3 adapter, B3 ADC sampling,
left/right bypass/recovery sequences, independent rear permission, command
IDs/cancellation, stale or malformed inputs, line-search bounds, timeouts,
timer rollover, startup/profile checks, packed data, B1 telemetry, and the existing
guarded forward/STOP path. Host assertions remain enabled. Unix ultrasonic
driver tests retain undefined-behavior sanitization; MinGW lacks libubsan.

The standard `build_make/mtk3pico_smp0_usb_cdc.uf2` and
`build_make/mtk3pico_smp1_usb_cdc.uf2` images are now built from the default
autonomous configuration. Earlier `*_b5_autonomous_validation.uf2` artifacts
were build checks with explicit overrides. No image was flashed and no
physical calibration or floor trial was performed by this implementation.
