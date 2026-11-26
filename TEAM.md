# Team and subsystems

**Team:** CSC25 (Group 25)

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Member | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|---|
| 1 | Wi-Fi / MQTT: communication, commands and telemetry | Asyha Mufliha | 2502999 | notmufliha | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control: motor PWM, wheel encoders and PI speed control | Goh Yi Xuan | 2502313 | xuanz-latte | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line following sensors and Code 39 barcode reading | Jerlyn Ho | 2503088 | Jerlynoo | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU / terrain: hump detection and height, tilt and lift detection, calibrated startup gate | Eng Li Qi | 2503027 | liqi-2503027 | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Obstacle detection and avoidance: ultrasonic ranging and servo scan | Lucas Ee | 2501152 | lucasee3 | `app_program/subsystem5/` | `testing/subsystem5/` |

The vehicle controller (`app_program/vehicle_controller.c`, line following in
`vc_follow.c`) and the run recorder (`run_log.c`) are shared code, changed by
team agreement.

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | B2 | Left encoder A; UART0 console disabled |
| GP1 | B2 | Left encoder B; UART0 console disabled |
| GP2 | B2 | Right encoder A |
| GP3 | B2 | Right encoder B |
| GP4 | B4 | GY-511 I2C0 SDA |
| GP5 | B4 | GY-511 I2C0 SCL |
| GP6 | | |
| GP7 | | |
| GP8 | B2 | Left motor reverse |
| GP9 | B2 | Left motor forward |
| GP10 | B2 | Right motor forward |
| GP11 | B2 | Right motor reverse |
| GP12 | B5 | Servo PWM slice 6 |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | B5 | Ultrasonic TRIG |
| GP17 | B5 | Ultrasonic ECHO, PWM slice 0 B input |
| GP18 | | |
| GP19 | S1 | WiFi/MQTT link-status LED (LED + 330 Ω to GND) |
| GP20 | | |
| GP21 | | |
| GP22 | B3 | Robo Pico piezo buzzer (barcode read / rejected beeps), PWM slice 3 A |
| GP26 | B3 | ADC0 line channel (mounted right) |
| GP27 | B3 | ADC1 line channel (mounted left) |
| GP28 | B3 | Digital barcode input |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | B4 | task_imu_sampling_task | GP4 SDA, GP5 SCL; kernel I2C driver enabled, 5 ms transaction-phase timeout |
| I2C1 | | | |
| SPI0 | | | |
| SPI1 | | | |
| ADC | B3 | b3_line_task | GP26 (ADC0, right line sensor), GP27 (ADC1, left line sensor); GP28 is read as a digital input |
| PWM slices (list) | B2: slices 4, 5 · B5: slice 6, slice 0 B input · B3: slice 3 A | B2: pid_control_task · B5: s5_task · B3: b3 tasks | B2 motors GP8–GP11; B5 servo GP12, echo GP17; B3 buzzer GP22 |
| WiFi radio, TCP/IP, MQTT | *kernel* (S1 feeds it via `comm_bridge.h`) | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

Robot build (`ROBOT_MODE`=1, the default). The control tasks run above
their band so sensing and motor control are never starved by telemetry.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 12–13 | led_task – 12 – any; comm_task – 13 – any; demo_task – 12 – any (only while `S1_STANDALONE_DEMO`=1) |
| 2 | 14–15 | pid_control_task – 5 – core 1; b2_telemetry_task – 14 – core 1; (diagnostics and wheels-up test tasks – 14 – core 1, only when enabled) |
| 3 | 16–17 | b3_line_task – 16 – core 0; b3_barcode_task – 17 – core 0 |
| 4 | 18–19 | task_imu_sampling_task – 8 – any; task_imu_analysis_task – 9 – any (18 / 19 when `ROBOT_MODE`=0, plus task_monitor_task – 19) |
| 5 | 20–21 | s5_task – 7 – any (20 when `ROBOT_MODE`=0) |
| shared | 6 | vehicle_controller – 6 – core 0 |
| shared | 22 | run_log – 22 – any (run recorder dump, app_program/run_log.c) |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S2 → S3 | mailbox (owned by S3) | `mbx_samples` | `SAMPLE_MSG` in `subsystem2.h` | *example* |
| IF-02 | S3 → S2 | semaphore + fixed memory pool (owned by S2) | `sem_slots`, `mpf_samples` | receiver returns the block, then 1 slot | *example* |
| IF-03 | S3 → S4 | message buffer + event flag (owned by S4) | `gh_summary`, `gh_events` | text line ≤ `S4_SUMMARY_MAX` bytes, then `FLG_BATCH` | *example* |
| IF-04 | S4 → S3 | shared data under a mutex (owned by S3) | `app_stats`, `mtx_stats` | `APP_STATS`, read only while locked | *example* |
| IF-05 | S4 → S1 | task control | `tid_led` | `tk_sus_tsk` / `tk_rsm_tsk` / `tk_chg_pri` | *example* |
| IF-S1-TLM | any → S1 | shared snapshot under mutex (owned by S1) | `mtx_telem` via `comm_set_xxx()` | `COMM_TELEMETRY` in `subsystem1.h` | *proposed* |
| IF-S1-EVT | any → S1 | message buffer (owned by S1), `TMO_POL` | `mbf_events` via `comm_report_xxx()` | barcode / hump / obstacle / error | *proposed* |
| IF-S1-CMD | S1 → Vehicle Controller | message buffer (owned by S1) | `mbf_remote_cmd` | `REMOTE_CMD` in `subsystem1.h`, receive with `TMO_POL` | *proposed* |
| IF-S1-NET | S1 ↔ kernel WiFi task | message buffers `mbf_tx` / `mbf_rx`, `TMO_POL` only | `comm_bridge_xxx()` in `comm_bridge.h` | topic id + JSON / command text; see `subsystem1/MQTT.md` | *needs team + instructor OK (changes `mqtt_utk.c`)* |
| IF-06 | | | | | |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are each subsystem's usage in the robot build.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 2 (+1 demo) | 2 | 2 | 2 real / 3 demo | 1 |
| Semaphore | 16 | | 2 | | | |
| Event flag | 16 | 1 | 1 | | 1 | |
| Mailbox | 12 | | | | | |
| Mutex | 12 | 1 | | 1 | | |
| Message buffer | 12 | 4 | | 1 | 2 real (+1 demo, +1 with TEST=1) | |
| Fixed memory pool | 12 | | 1 | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | 1 | 1 | | | |
| Alarm handler | 12 | | | | 0 real / 1 demo | |

(The kernel's USB and WiFi service tasks also use a few of these.)

## Integrated motor handoff

VC owns normal DRIVE and barcode movement. B5 publishes BUSY before bypass
commands and restores FOLLOW only after line/heading recovery. Mission disable
is checked again inside B2 safety authorization and PWM application, so a delayed
B5 clearance update cannot override STOP. B3 remains the only ADC owner.
