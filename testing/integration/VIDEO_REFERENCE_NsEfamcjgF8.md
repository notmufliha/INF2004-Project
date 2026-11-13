# Line following and obstacle avoidance video reference

Source: [Arduino self-driving Line follower and obstacle avoidance robot](https://www.youtube.com/shorts/NsEfamcjgF8), victor gofigan.

The actual MP4 was retrieved and visually reviewed at half-second intervals
across its approximately 21-second duration on 11 November 2026. Temporary
video, extracted frames and a contact sheet are under
`.codex_tmp/video-reference/`; these are not source artifacts.

## Observed behavior

| Approximate time | Visible behavior | Project implementation |
| --- | --- | --- |
| 0–3 s | Follows black tape towards the upright sign. | Vehicle controller steers from the two normalized IR readings; B5 monitors front range. |
| 3–6 s | Turns off the tape, passes on the sign's left in the camera view, and returns to tape beyond it. | B5 selects an available side, turns outward, offsets, passes the obstacle, and turns inward for a bounded line search. |
| 6–11 s | Tracks the angled tape towards the blue box. | IR steering resumes after line and heading confirmation. |
| 11–14 s | Leaves the tape around the blue box on the open floor side and rejoins beyond it. | A completed handoff rearms the existing B5 controller for another obstacle. |
| 14–end | Continues following tape through subsequent corners towards the sign again. | Vehicle controller follows fresh sensor readings, with bounded lost-line recovery. |

The clip shows the desired external behavior. It does not reveal source code,
exact turn angles, distances, sensor thresholds, or whether its avoidance is
timed or feedback controlled. Those details cannot be inferred reliably from
the video. This project retains encoder-controlled legs and front monitoring
rather than copying guessed timings from a different chassis.

## Changes following visual review

- Brake at the first fresh tape detection during the inward search, instead
  of driving through three obstacle-task observations before braking.
- Require three distinct fresh tape samples while physically stopped before
  aligning. If tape disappears, resume the remaining bounded search.
- Retain post-alignment confirmation and bounded heading sweeps before
  returning command ownership to line following.
- Clear pre-obstacle steering/speed history and restart the lost-line search
  window after B5 hands control back (implemented in the preceding change).

## Verification and physical limits

Host tests exercise both avoidance sides, immediate braking, stopped line
confirmation, duplicate timestamps, transient detections, heading recovery,
command ownership, a second obstacle, and search exhaustion. The complete
subsystem test runner passed; the B5 suite passed again after adding an
explicit navigation assertion for first-contact braking. Pico W network
firmware compiled, linked and produced the UF2. The linker emitted its
existing newlib syscall-stub warnings for `_close`, `_lseek`, `_read`, and
`_write`; no compile or link errors occurred.

Firmware: `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
SHA256: `3C773A428F239E73560B5A8A518A3865D27D1EF1E32113B8940287794A04CED8`.

No physical robot result is claimed. Current configured geometry and wheel
turn calibration remain provisional. The existing course bounds are obstacle
width at most 30 cm and depth at most 40 cm; the servo scan is limited to
70–110 degrees. A later course-bound fallback handles clipped profiles
without increasing servo travel, using configured size bounds and a fresh
clearance check after turning. No observable side clearance causes a stop.
Use section 6 of `ROBOT_TEST_STEPS.md` for physical acceptance, including two
successive obstacles and STOP/PAUSE/E-STOP during avoidance.
