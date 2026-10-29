# Buddy 5: Barr-C-oriented style and Tribunal minimum-bit change

This folder contains **B5-only application changes** carried forward from the
`buddy_5` branch onto the team's B1-merged `origin/main`.

## Current, integrated team interfaces

- B5 sends the front distance in cm using `comm_set_obstacle_dist()`.
- B5 reports a scan using `comm_report_obstacle()`; B1 packs the binary
  obstacle event as 9 + 7 + 9 + 9 + 3 = **37 application bits**.
- B5 raises and clears **only** `FAULT_NO_ECHO` using the kernel-backed
  shared fault interface (`fault_raise`/`fault_clear`). B5 does not modify
  the event-flag implementation or any other subsystem's fault bit.
- B5 reports its existing error numbers through `comm_report_error()`;
  the B5 block of `error_codes.h` and B5 names in `s1_wire.py` are aligned.

## Separate optional 42-bit B5 representation

`S5PackedProfile` encodes 9-bit obstacle distance, 7-bit width, 9-bit left
clearance, 9-bit right clearance, and 8-bit servo angle (42 data bits), plus
4-bit version and 2 pad bits in **6 bytes**. Reserved invalid values are
511, 127 and 255 respectively. It is an independent host-tested format,
**not** the MQTT format currently sent by B1. Do not replace B1's MQTT
frames with this format without an agreed interface change.

## Barr-C scope

Selected B5 files have explicit braces, clearer fixed-width representation
and improved formatting. This is not a certification against every Barr-C
rule or a claim that all legacy B5 lines have been reformatted.

## Safety and verification

Existing STOP-on-unknown behaviour and the physical bypass gate remain.
The real encoder/line-reacquisition integration, firmware build on the Pico
SDK, and hardware performance/RAM measurements must be tested by the team.
See `README_B5_MAIN_INTEGRATION.md` for the merged B1 interface and the
error-code discrepancy with the original Tribunal table.
