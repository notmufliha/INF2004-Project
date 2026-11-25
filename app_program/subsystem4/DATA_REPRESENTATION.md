# Buddy 4 minimum-bit data representation

The November 2026 change requirement applies to internal application data and
MQTT. The tribunal identifies a three-bit motion state and calls for preserving
IMU measurement precision until a narrower encoding is validated. The packing
here applies to bounded states and flags. The sensor, filter and hump estimator
retain their existing numerical precision.

| Field or group | Valid range or meanings | Bits per field | Representation |
| --- | --- | ---: | --- |
| Motion state in `imu_event_t` | 0–7, stationary through unknown | 3 | Enum bit-field |
| Event flags | Hump active/completed, tilt, reading valid, height/peak valid, sensor fault/recovery, turn rate valid, heading valid/calibrated, impact and lift | 1 | 13 Boolean bit-fields |
| Sample flags | Reading, turn rate, magnetics, heading, calibrated heading and odometry validity | 1 | 6 Boolean bit-fields |
| Odometry reasons | Six independent reasons, mask 0–63 | 6 | Unsigned bit-field used as a mask |
| Timing availability | Odometry age/encoder/context, accelerometer/magnetometer reads and deferred magnetics, mask 0–63 | 6 | Unsigned bit-field used as a mask |
| B2 motion context flags | Valid, running, forward, reverse, turning, stopped and fault | 1 | 7 Boolean bit-fields |
| Controller notice flags | 12 independent observations plus queue overflow, mask 0–8191 | 13 | Unsigned bit-field used as a mask |
| Controller completion height valid | False/true | 1 | Boolean bit-field alongside the notice mask |
| Notice tracker flags | Seen, valid, hump active/completed, tilt known/active, impact, lift, fault and recovery | 1 | 10 Boolean bit-fields |
| Analysis roll state / terrain stage | 0–2 / 0–3 | 2 each | Enum bit-fields |
| Analysis and processing state flags | False/true per flag | 1 | Boolean bit-fields |
| Sensor, travel and height calculations | Finite sensor values and calibrated angles; tilt ±180°, heading 0–360° with wrap; height from configured wheel spacing | 32 retained | `float32_t` for calibration, filtering and geometric estimation |
| Timestamps, IDs and sequence/error counters | 0–4,294,967,295 with existing wrap semantics | 32 retained | `uint32_t` |
| Encoder counts and signed B2 speed snapshots | −2,147,483,648–2,147,483,647 at the input interface | 32 retained | `int32_t` |
| Diagnostic timing durations | 0–65,535 ms, saturating | 16 | `uint16_t` |
| MQTT motion / lift / height validity | Motion 0–7, lift and validity false/true | 3 / 1 / 1 | Explicit bit packing under the existing telemetry mutex |
| MQTT pitch / roll | ±51.1° / ±180.0°, 0.1° steps | 10 / 12 | Signed two's complement |
| MQTT latest / peak hump height | 0–126 mm each | 7 each | Unsigned, millimetre resolution |

Queue messages are copied by T-Kernel. The sampling task owns its outgoing
sample and analysis owns its local copies, event and notice tracker. Bit-field
updates therefore occur on task-owned storage. The controller receives its own
copy. Shared MQTT snapshot writes and reads use the existing B1 mutex. Native
bit-field layout is specific to the build ABI. MQTT continues to use masks and
shifts, with 30-byte telemetry and a 6-byte hump event.

## Cortex-M0+ size measurements

Baseline is the build before the minimum-bit change. Sizes include compiler padding. Queue
storage derives from the measured payload sizes and the kernel slot formula
`depth × (4 + align4(payload))`, where 4 bytes hold the message length.

| Payload, object or queue storage | Before (B) | After (B) | Saved (B) |
| --- | ---: | ---: | ---: |
| IMU motion context, included in a sample | 36 | 32 | 4 |
| Sampling → analysis message | 128 | 112 | 16 |
| Analysis event, included in a controller notice | 52 | 32 | 20 |
| Analysis → controller notice | 88 | 64 | 24 |
| Static notice tracker | 108 | 76 | 32 |
| Analysis task context | 176 | 156 | 20 |
| Processing context, including its previous sample | 160 | 144 | 16 |
| Sampling queue, 8 slots | 1,056 | 928 | 128 |
| Controller notice queue, 8 slots | 736 | 544 | 192 |
| **Release queue reservations plus static notice tracker** | **1,900** | **1,548** | **352** |
| Additional `APP_TEST` event queue, 32 slots | 1,792 | 1,152 | 640 |

The release subtotal counts the tracker and the two queue reservations once.
It excludes kernel control blocks, fixed stack reservations, other subsystem
RAM and peak stack usage. The analysis and processing context reductions
reduce the size of those local objects within the existing stack reservation.

To reproduce, compile `testing/subsystem4/measure_sizes.c` with each version's
headers and compare symbol sizes. This is a measurement object only.

```sh
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -Iapp_program/subsystem4 \
  -c testing/subsystem4/measure_sizes.c -o b4_sizes.o
arm-none-eabi-nm -S -t d b4_sizes.o
```

## Verification

The B4 host suites exercise time gaps, wraparound, roll classification, hump
height, impact and lift. The startup suite exercises encoder pairing,
controller notice delivery, queue saturation and recovery. The ARM syntax
checks cover eight combinations of robot, mock and test configurations. The
B4 matrix disables B5 bypass so it can exercise standalone configurations.
Calculations, task periods and queue depths retain the existing settings.
The tests cover timing logic with simulated timestamps. Measuring actual
sampling cadence and motor-loop timing under MQTT load requires running the
revised firmware on the robot.
