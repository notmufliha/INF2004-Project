# Buddy 5 completion after B1 merge to origin/main

Base: `origin/main` at `0850acf` (B1 merge present). The uploaded ZIP
was checked out at `buddy_5` (`d5f68cb`), where B1 APIs were not present.
This package takes the B5 source from `buddy_5` and places it on the
merged `origin/main` tree, preserving the merged team's other subsystems.

## Scope
- Buddy 5 application source and Buddy 5 tests.
- Only the B5 block of `app_program/error_codes.h`.
- Only the B5 entries of `tools/s1_wire.py`.
- **No** changes to B1, B2, B3, B4, the Vehicle Controller, Wi-Fi
  priorities, common fault flag implementation, or MQTT wire format.

## Completed B5 integration
1. Front distance -> `comm_set_obstacle_dist()` (cm, rounded; 0..400;
   `COMM_CM_INVALID` for missing/invalid).
2. Scan profiles -> `comm_report_obstacle()` (width 0..100 cm;
   clearances 0..400 cm; invalid profile forces `AVOID_STOP`).
3. Raise and clear **only** `FAULT_NO_ECHO`; publish one no-echo error
   on transition into unavailable state rather than every sample.
4. Publish stop-ack and unsafe-route error events; live bridge also
   publishes no-echo and profile events.
5. Add explicit B5 error aliases in `error_codes.h` and synchronize
   the descriptions in `s1_wire.py` without renumbering existing codes.
6. B5's optional independent 42-bit packed format is retained for its
   own host tests; B1's existing MQTT encoder remains authoritative.

## Team error-code mapping (current implementation)
- 0xA1: no reliable ultrasonic echo/front sensor (detail 45 = 90 / 2)
- 0xA2: motor stop acknowledgement failed
- 0xA3: scan timeout
- 0xA4: no safe obstacle bypass route
- 0xA5: fine scan failed
- 0xA6: motion timeout
- 0xA7: line timeout
- 0xA8: insufficient clearance
- 0xA9: reverse timeout

**Tribunal discrepancy:** The earlier table labels A2 "no safe clearance"
and A3 "servo not settled". These meanings cannot be substituted for
A2/A3 without changing the S5Error enum, firmware emitters and laptop
decoder together. Current code has no verified servo-settling fault detector;
do not claim that one is implemented. Ask the team whether to update the
Tribunal table or agree on a coordinated protocol revision.

## Ownership of Wi-Fi priority
The Wi-Fi task still requires a team-wide priority review; B5 does not
change B1 or kernel scheduling. B1/VC should check numerical priority
ordering and ensure Wi-Fi has the *lowest scheduling priority*.

## Testing
From the repository root:

```sh
bash testing/subsystem5/run_host_tests.sh
bash testing/subsystem5/run_packed_tests.sh
python3 testing/subsystem5/test_b1_contract.py
python3 testing/subsystem5/test_b5_error_contract.py
python3 -m py_compile tools/s1_wire.py
```

A successful host test is not a full Pico firmware build or physical
robot validation. Rebuild the complete firmware, verify USB/MQTT output,
raise the wheels for first motion tests, and confirm STOP acknowledgement.
For Week 13, record actual RAM usage and timing from hardware builds.
