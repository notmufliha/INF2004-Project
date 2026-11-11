# Echo/dashboard verification, 2026-11-10

Working tree based on `62467ca5a8e84b14ebbcc3d1cef91a188af7dbb2`.

Confirmed software issues corrected:

- Live B5 stopped sampling when the mission was disabled, leaving an old
  distance visible in fresh telemetry. It now samples while STOP/PAUSE is active.
- No-target clearance was presented as a measured 400 cm. Front telemetry now
  uses code 510 for the explicit assumption; both laptop decoders recognize it.
- The dashboard no longer draws assumed/invalid distances as measurements or
  joins measured chart segments across those gaps.
- Unmeasured profile values start invalid rather than as zero-width/clearance.
- The driver enforces 65 ms between trigger starts for every caller.
- Front error events include raw failure status; serial diagnostics include
  captured pulse width. Stuck-high, unfinished and malformed returns stay faults.

Host results: B1 protocol/dashboard decoder and actual UI handler tests pass;
B2's 1,239 assertions, controller/line tests and all 16 B5 groups pass.
The driver test checks minimum spacing on every trigger and repeated empty
path cases (no rise, completed 38 ms and 55 ms pulses).

[Full suite transcript](logs/2026-11-10_151427_host.txt).

Network firmware built successfully with linker syscall warnings; SHA256:
`7A4D125ADDE7E95A8B2293C4F089BEAB52F29B80EFCD4AE53B27090D3087D5D8`.

Passive broker inspection accepted a local subscription, but the retained car
status was offline and no live robot frames arrived. No motion commands were
sent. Browser rendering and physical empty-path reliability remain to be
verified by flashing, hard-refreshing the dashboard, and testing a real box.
The screenshot's historical IMU fault and line LOST state are separate motion
blockers, not proof that the ultrasonic event caused SAFE_STOP.
