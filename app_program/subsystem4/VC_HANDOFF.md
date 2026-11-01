# B4 observations for the Vehicle Controller

Include `../subsystem4/imu_vc.h`. B4 initializes this interface in
`subsystem4_init()` and publishes from its analysis task. Exactly one controller
task may consume it after successful B4 initialization. No B2 or controller
implementation has been changed.

```c
imu_notice_t notice = {0};
ER error = imu_vc_receive(&notice, 0); /* Poll; positive milliseconds may wait. */
if (E_OK == error)
{
    /* Controller policy interprets notice.events and notice.snapshot. */
}
```

`E_TMOUT` means no message arrived within the requested time; it does not mean
an IMU fault. Negative timeouts and null destinations return `E_PAR`.
The destination is zeroed on receive errors. Do not call `imu_vc_publish()`
from another task: its tracker belongs exclusively to B4's analysis task.

| Flag | Observation |
|---|---|
| `IMU_NOTICE_READY` | Usable IMU data became available. |
| `IMU_NOTICE_UNAVAILABLE` | Data became unusable, including an initial invalid sample. |
| `IMU_NOTICE_HUMP_ENTER` | Confirmed terrain traversal became active. |
| `IMU_NOTICE_HUMP_DONE` | Traversal completed; height may still be unavailable. |
| `IMU_NOTICE_HUMP_ABORT` | Active traversal ended without completion. |
| `IMU_NOTICE_TILT_ON`, `IMU_NOTICE_TILT_OFF` | Tilt entered or cleared on usable data. |
| `IMU_NOTICE_IMPACT` | Suspected impact began; valid quiet data rearms it. |
| `IMU_NOTICE_FAULT`, `IMU_NOTICE_RECOVERED` | B4's latched sensor fault began or valid recovery occurred. |
| `IMU_NOTICE_STATE` | Initial or periodic state snapshot. |
| `IMU_NOTICE_OVERFLOW` | At least one send failed; pending observations were coalesced. |

Flags describe observations, not speed or stop commands. B4 retains ownership of
IMU fault raising/clearing and existing B1 telemetry. The Vehicle Controller owner
must implement any coordinated motion response. This interface alone does not
stop the car on runtime faults.

`snapshot` contains the latest state at successful publication. Always check
`snapshot.b_valid`; unavailable data must not be interpreted as level or safe.
Invalid data does not generate a tilt-clear transition. STATE is generated after
one second without another observation, provided analysis keeps receiving data.
The controller should check unsigned snapshot age against its chosen timeout;
silence can also indicate a stalled publisher. Delivery delay must be considered.

The queue holds eight copied notices. Sending always polls, so a missing or slow
controller never blocks sampling/analysis. On a failed send, B4 retains event bits
and retries on the next processed sample. Multiple edges may share one notice;
both onset and clearing bits can be present. They are not an ordered event log.
Use the latest valid snapshot for current state and treat retained hazard bits
according to controller policy. Buffered messages can themselves be old.

`first_sequence`/`last_sequence` and their timestamps bound the observations
coalesced into a notice. `snapshot.timestamp_ms` may be newer.
`failed_sends` counts cumulative failed attempts, not lost events. Sequence,
timestamp and failure counters wrap modulo 2^32. If multiple completions coalesce,
only the latest completion's details remain. `completion_timestamp_ms`,
`completed_height_cm`, and `b_completed_height_valid` retain that completion even
when the latest snapshot is quiet. Read height only when HUMP_DONE is present
and its dedicated validity flag is true. Height units are centimetres.

On the current ARM ABI each notice is 88 bytes; eight aligned kernel slots use
736 bytes, excluding kernel control blocks. The producer also holds a small
fixed tracker and temporary stack copies. This is additional RAM, not a RAM
optimization. No heap allocation or unbounded history is introduced.

Run `testing/subsystem4/startup/run_tests.ps1` for production interface tests
with a native bounded-queue stub, including saturation, retries, fault recovery,
completion retention, malformed receives and timestamp wrap. Run the existing
host suite and ARM configuration checker as well. On-car scheduling, real kernel
queue pressure and the controller's eventual motion policy need hardware tests.

