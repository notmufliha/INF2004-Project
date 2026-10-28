# Buddy 5: integration with Buddy 1's Wi-Fi/MQTT branch

**Source branch reviewed:** `notmufliha/INF2004-Project`, `b1-wifi-mqtt`, commit `913c3df86d8a4fb5a790b65e7f9ed584f44bb6d3` (2026-10-28 snapshot).

**Scope:** `app_program/subsystem5/task_obstacle.c` only, plus this B5 note and a B5-only host test. No B1/B2/B3/B4/Vehicle Controller, shared header, task priority, or laptop decoder is modified.

## What is implemented

- Preserve B1's existing `comm_set_obstacle_dist()`, `comm_report_obstacle()`, `fault_raise()` and `fault_clear()` calls.
- Saturate valid B5 distances/clearances at 400 cm and widths at 100 cm; keep the B1 `COMM_CM_INVALID` sentinel for invalid/no-echo measurements. Values are rounded to 1 cm.
- Raise `FAULT_NO_ECHO` when the ultrasonic measurement is invalid, clear only on a valid reading. No other subsystem's bit is touched.
- Send one `ERR_S5(S5_ERR_FRONT_SENSOR)` event when a new no-echo episode begins (detail 45 = 90-degree forward servo angle / 2). Do not flood the event queue every poll.
- Send `ERR_S5(S5_ERR_STOP_TIMEOUT)` when the scan's stop-acknowledgement check fails. This is an acknowledgement failure, not a measured timeout interval.
- Send a safe `AVOID_STOP` obstacle event even when the coarse scan has no valid nearest point, rather than silently dropping the event.
- Send `ERR_S5(S5_ERR_NO_CLEAR_ROUTE)` when the profile is invalid or there is no safe side. It is a scan/planning error, **not** a shared fault flag (none is allocated to this condition).
- In motion-bridge mode, publish front-distance/no-echo status and send profile events for successful bypass decisions and unsafe profile halts.
- Keep all B5 motor safety checks and the disabled physical bypass gate unchanged.

## Existing shared team APIs (not modified)

`subsystem1/subsystem1.h` provides:

```c
comm_set_obstacle_dist(UH dist_cm);
comm_report_obstacle(UH dist_cm, UH width_cm, UH left_cm,
                     UH right_cm, AVOID_ACTION action);
comm_report_error(UB err_code, UB detail);
fault_raise(FAULT_NO_ECHO);
fault_clear(FAULT_NO_ECHO);
```

B1 already bit-packs the fields on the telemetry/event wire. Do **not** add another incompatible 6-byte B5 payload to MQTT. B1's current obstacle event contains 9+7+9+9+3 = 37 application bits (distance, width, two clearances, action); **servo angle is not currently sent by B1**. B5's proposed 42-bit five-field design is a separate layout, not the currently integrated B1 wire format. Discuss any servo-angle addition with B1 first.

## Error code disagreement that requires a team decision

The current shared `error_codes.h` defines `ERR_S5(S5Error)`, and `tools/s1_wire.py` already includes all existing S5 error names. Under that contract:

- `0xA1` = front sensor failure (no echo), detail 45 for forward-facing servo;
- `0xA2` = stop acknowledgement timeout;
- `0xA3` = scan timeout;
- `0xA4` = no clear route;
- `0xA5` = fine-scan failure.

The Tribunal table listed `0xA2` as no safe clearance and `0xA3` as servo not settled. **Do not silently change these shared codes**: the B1 decoder and existing tests use the current meanings. Ask the team to agree whether to update the Tribunal table or jointly change both the shared header and laptop decoder. This patch uses the existing team code mapping and does not claim servo-settling detection that is not implemented.

## Integration and verification

1. Check out the team's `b1-wifi-mqtt` branch or a merge that contains the same public B1 API. Do not copy the file onto an unrelated older branch without reviewing the diff.
2. Replace only `app_program/subsystem5/task_obstacle.c`. Keep all other source files unchanged.
3. Run `python3 testing/subsystem5/test_b1_contract.py` (requires a host C compiler). Existing B5 host tests are in `testing/subsystem5/run_host_tests.sh`.
4. Rebuild firmware with the **team's** Pico/μT-Kernel toolchain; check for warnings and link errors. Test with wheels raised before floor testing.
5. With B1/MQTT running, check the laptop receives front distance, a scan profile, a no-echo fault bit, and one front-sensor error on a transition into no echo. Restore sensor and confirm the bit clears.
6. Validate B2 STOP acknowledgement and real sensor/servo timing on hardware. Physical bypass and line recovery remain gated off until B2/B3/VC integration is safe.

### Remaining team-owned work (not changed by this B5 patch)

- The **Vehicle Controller** must eventually own creation of the shared fault event flag; B1 currently creates it.
- **B1** owns Wi-Fi task priority and the MQTT decoder/wire format. B5 cannot lower B1's priority by changing its own task.
- The shared error-code header and `tools/s1_wire.py` are team-owned and already have B5 mappings; any renumbering requires coordinated edits.
- Week 13 requires measured before/after RAM, message size, correctness and timing evidence. Host checks are not physical hardware proof.
