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
