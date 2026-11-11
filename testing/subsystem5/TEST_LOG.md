# Buddy 5 test evidence (software-only)

## Host tests

Run `bash testing/subsystem5/run_host_tests.sh` from the repository root.

- H01: left route, coarse + fine scan, line recovery -> DONE
- H02: right route -> DONE
- H03: both sides blocked -> NO_CLEAR_ROUTE safe halt
- H04: left coarse reading invalid -> chooses verified right side
- H05: stop acknowledgement missing -> STOP_TIMEOUT safe halt
- H06: motor completion missing -> MOTION_TIMEOUT safe halt
- H07: line never found -> LINE_TIMEOUT safe halt
- H08: no obstacle -> FOLLOW
- H09: invalid front readings -> FRONT_SENSOR safe halt
- H10: invalid fine scan -> FINE_SCAN safe halt

## Pico firmware unit checks (TEST=1)

- S5-U01 echo pulse to centimetres conversion
- S5-U02 servo centre pulse calculation
- S5-U03 successful left route
- S5-U04 successful right route
- S5-U05 blocked route
- S5-U06 invalid left ray rejected
- S5-U07 stop timeout
- S5-U08 movement timeout
- S5-U09 line recovery timeout
- S5-U10 clear path
- S5-U11 front sensor fault
- S5-U12 invalid fine scan
- S5-U13 width estimate exists

The tests use **synthetic** sensor/actuator events; no physical robot was tested.

## Bypass and continuation search, 2026-11-10

Source: working tree based on `549442f`, retaining the PD integration.

| ID | Case | Expected result | Host result |
|---|---|---|---|
| S5-BYP-01 | Left/right detour, crossing and forward alignment | Confirm line after each side; complete handoff | PASS |
| S5-BYP-02 | Continuation lies beyond the old predicted crossing window | Continue bounded search and stop on confirmed black | PASS |
| S5-BYP-03 | Broad line sets junction flag | Accept fresh black samples after departing original line | PASS |
| S5-BYP-04 | Sensors rotate off tape during forward alignment | Sweep toward line and reconfirm before completion | PASS |
| S5-BYP-05 | Duplicate snapshots during alignment | Never count same sample twice or complete early | PASS |
| S5-BYP-06 | No line throughout corridor/alignment sweep | Stop with LINE_TIMEOUT | PASS |
| S5-BYP-07 | Real B2/B3 adapter through alignment sweep and second obstacle | Track motion IDs, restore clearance, resume, stop again | PASS |

Evidence: [full host transcript](../integration/logs/2026-11-10_162050_host.txt),
`test_maneuver.c` and `test_navigation.c`. All 16 B5 groups and the full
requested-subsystem host suite passed. Firmware build succeeded with existing
newlib syscall warnings. UF2 SHA256:
`98628CB798C4DCAEE35101810D3C702FC702F72A2FBCE94EF320AF6ED1FEB9E4`.

Hardware results remain pending. The single front sensor does not measure
hidden obstacle depth or identify which black marking is the original line.
The course must have the expected continuation in the recovery corridor;
configured chassis, width, depth, encoder and turn calibration must be validated.
