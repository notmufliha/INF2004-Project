# PD branch integration and startup calibration

Current branch: `robot-fixes`, base `549442f` plus working-tree changes.
Line-following source: `line-follow-pd`, commit `503ed6e`.

Ported the weighted line position, PD controller, remembered-side recovery,
and confirmed-extreme calibration. Preserved the current timestamp ordering,
gentle low-speed steering, dashboard and ultrasonic fixes. No branch switch,
commit or merge was performed. Removed the unused fixed-trim setting.

## Calibration corrections

- Use a 64-count confirmed contrast instead of the branch's 300-count cutoff.
  Low-output sensors otherwise remain on provisional normalization despite
  having observed distinct white and black surfaces.
- Use overlapping three-sample windows, so three consecutive surface readings
  qualify regardless of their phase relative to task startup. An isolated
  spike cannot widen either endpoint.
- Preserve the branch's provisional 220..620 ADC range (raw 301 is line-active
  immediately), normalized detection >=201, and position noise threshold 50.
  These are defaults, not measured calibration for this robot.
- Emit one `[B3-CAL] learned GP26 white=... black=...; GP27 ...` line once both
  sensors have learned sufficient contrast. Keep the same operating height;
  lifting the sensor for several samples can change its learned black level.

## Verification

The full host suite passed: dashboard decoding/UI, B2's 1,239 assertions,
controller/barcode/line tests and all 16 B5 groups.
[Saved transcript](logs/2026-11-10_155401_host.txt).

Additional targeted controller run passed after adding the end-to-end
calibration/autostart regression. With otherwise fresh, authorized inputs,
three white samples followed by three low-output black samples produce the
first DRIVE on the third black sample. This does not bypass the real IMU
startup gate, B5 warmup, clearance confirmation or STOP.

PD tests verify initial proportional-only response, derivative damping as
error shrinks, response as error grows, both-direction saturation, recovery,
and retained producer-snapshot timing regression. `S3-PD-C01..C10` label the
imported/new calibration cases separately from historical `S3-C*` results.

Firmware: `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
SHA256: `5C2E1BB668BCCCA9A9681A2D95C56557EEDA42F8031984F9091F3FBDE0C0FB8A`.
Build succeeded with existing newlib syscall warnings.

Physical white/black readings were requested but not supplied in this session.
No motor commands, flashing or physical calibration were performed here.
On the robot, press dashboard STOP, sweep both surfaces under each sensor
at driving height for at least 0.1 s each, confirm the B3 calibration log,
then START with motor power on. Measure straight and curved tracking before
claiming physical completion or tuning the PD gains further.
