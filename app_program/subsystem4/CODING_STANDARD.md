# B4 BARR-C:2018 audit

2026-11-01 snapshot update: the B4-owned `imu_motion` module consumes the existing
B2 read API. Its public header exposes only B4 fixed-width data and bool flags;
kernel/B2 enums stay inside the adapter with documented validation casts.
The new production code passes the same BARR-C checks and all eight ARM C99
configurations. Reading/event/motion/height context layouts are 116/52/36/92
bytes in the checked ARM layout. Historical milestone sizes below are retained.

2026-11-01 height update: `imu_height` is a pure C99 estimator and `imu_odom`
adapts the existing kernel/B2 interfaces. New state uses fixed-width counts,
Boolean prefixes, private static helpers and finite/range-checked calculations.
Tests cover analytic profiles, filtered terrain integration, acquisition races,
queue sequence loss and rejection/recovery. Reading/event queues use sizeof;
their checked native layouts are now 80/52 bytes, not the historical 56/48.

2026-11-01 startup update: production B4 uses C99 checks in eight robot/demo,
real/mock and test/normal configurations. Readiness uses bounded kernel waits,
unsigned masks and checked results. Native startup tests execute `subsystem4.c`
with a stub kernel. Stub symbols/types reproduce the external kernel/task ABI;
they are not production application interfaces or part of the C style scan.

Reviewed against the user's `barr_c_coding_standard_2018.pdf`, 2018 edition,
on 2026-10-28. PDF SHA-256:
`676CCCFD096C8AE563BB43BFBA4AA6F52537B671B2FE06118869532FB0791963`.

Scope: all C/header files in `app_program/subsystem4` and desktop C tests in
`testing/subsystem4/host`. Shared startup/B3 references and the B4 build rule
are updated where needed for renamed interfaces. Other subsystems, kernel,
SDK and device implementations are outside this audit.

The following record covers every numbered section, including 8.6. It combines
source review and automated evidence: the scanner alone cannot establish
compliance. No mandatory-rule finding remains open in this scoped review.
This is an engineering review, not independent certification or a whole-project
conformance claim.

## Rule-by-rule record

| Rule | Review result and evidence |
|---|---|
| 1.1 C dialect | B4 firmware objects and host tests use C99. Kernel/SDK retain their dialect. No B4 assembly, proprietary keywords or keyword-renaming macros. |
| 1.2 Width | Source/header lines stay within 80 columns; checked automatically. |
| 1.3 Braces | Braced control bodies and Allman layout, including braces before comments. |
| 1.4 Parentheses | Arithmetic grouping reviewed; complex logical operands parenthesized and scanned. |
| 1.5 Abbreviations | Versioned terms/units table below. |
| 1.6 Casts | Explanations accompany decoding, widening, kernel sizes/callbacks, display conversion and ignored callback arguments. Numeric display narrowing is bounded. |
| 1.7 Avoided keywords | No auto, register, goto or continue in scoped code. |
| 1.8 Qualifiers | Private data/helpers static, read-only inputs const, shared IDs/counter volatile. Statistics use the existing mutex; sensor state has one owner. |
| 2.1 Comment formats | No commented-out code or forbidden embedded tokens. Optional raw magnetic diagnostics are guarded by NDEBUG; functional telemetry remains available. |
| 2.2 Comment content | Doxygen module/function descriptions, units, conversion bounds, sensor assumptions and datasheet reference reviewed/updated. |
| 3.1 Spaces | Formatter controls operators, commas, pointers and keywords. Declarations have a space before the parameter list; calls do not. |
| 3.2 Alignment | Consecutive declarations, members and assignments align; preprocessor markers start in column one. |
| 3.3 Blank lines | One statement per line and separated natural blocks. EOF comment followed by a blank line in every scoped source/header. |
| 3.4 Indentation | Four-space blocks, aligned cases/breaks and readable continuations. |
| 3.5 Tabs | No source/header tabs. |
| 3.6 Control characters | LF endings; prohibited control characters checked as bytes. |
| 4.1 Modules | Lowercase and unique first eight characters. Driver renamed to gy511, type header to imu, onboard test to s4_test. Host entry filenames contain main. |
| 4.2 Headers | Paired guarded headers; no public variable storage/function bodies or public inclusion of private s4_runtime.h. Necessary shared declarations discussed below. |
| 4.3 Sources | Own-header includes, one module responsibility, declarations/prototypes/public/private ordering, relative includes and configuration-appropriate includes reviewed. |
| 4.4 Templates | Source/header templates added under testing/subsystem4/templates. |
| 5.1 Types | Lowercase typedef names with _t suffix and module prefixes. float32_t follows the explicit spelling in 5.4. |
| 5.2 Widths | Width-sensitive application data uses fixed-width integers; kernel ABI aliases remain at API boundaries. Plain char is for strings; main retains ISO-required int. |
| 5.3 Signedness | Unsigned decoding/masks; no signed bit fields or Boolean bitwise accumulation. Conversion/sign-conversion warnings are errors. Monitor division widens safely. |
| 5.4 Floating point | Binary32 checked at compile time; f-suffixed constants and tolerance comparisons. Computed tilt, magnetic values, heading/rate, filter output and calibration statistics checked for finiteness before acceptance. Rationale below. |
| 5.5 Layout | Peripheral values decoded byte by byte. Native structs are same-image queue payloads, never wire formats. sizeof/alignment and compile-time payload bounds determine storage. No bit fields. |
| 5.6 Booleans | bool status values; numeric masks converted through comparisons; Boolean pointer names use pb_. |
| 6.1 Procedures | Module-prefixed public functions, descriptive private names, uppercase macros and names at most 31 characters. ISO-required main keeps its mandated name. |
| 6.2 Functions | Own-header prototypes, static private helpers, named parameters and at most 100 lines after formatting. Preferred practices discussed below. |
| 6.3 Function macros | None owned by B4. Existing kernel/test macros are external framework interfaces. |
| 6.4 Threads | Sampling, analysis and monitor entry names end in _task. |
| 6.5 Interrupts | Private task_monitor_alarm_isr placed at module end and registered with TA_HLNG. Raw ISR attributes/vector defaults belong to the kernel wrapper. |
| 7.1 Variables | Lowercase descriptive names, 3..31 characters and global/handle/pointer/Boolean prefixes in the required order. Shared handles renamed gh_summary/gh_events with consumers updated. |
| 7.2 Initialization | State initialized before use; callback outputs consumed after success; context pointers initialized; file-global definitions grouped before functions. |
| 8.1 Declarations | One variable per declaration. |
| 8.2 Conditionals | At most two nested if/else levels, no test assignments/increments and final else on else-if chains. |
| 8.3 Switches | Default branches, aligned breaks and no undocumented fallthrough. Returning cases cannot fall through. |
| 8.4 Loops | Named/sizeof/context bounds, zero initialization, no controlling-test assignments, for (;;) for perpetual tasks and an explanatory event-drain comment. |
| 8.5 Jumps | No direct B4 abort/exit/setjmp/longjmp or goto. Host tests use the standard assertion facility. |
| 8.6 Equality | Constant-first ==, including cast sizeof constants. Variable-to-variable comparisons keep their natural order. |

## Preferred practices and platform rationale

- **Early returns (6.2.c):** retained for validation/errors to prevent invalid
  readings from reaching processing and avoid deep nesting. Single exit is
  preferred, not mandatory. Print-page placement (6.2.b) depends on printing;
  the source length limit is enforced without inserting form-feed characters.
- **Shared declarations (4.2.c.i):** B3/B4 communicate using existing kernel
  object IDs. Only integration handles are public; task-only declarations and
  configuration use a private header. The diagnostic counter is now static,
  read through `task_monitor_line_count()`. Avoiding every extern is preferred,
  not a prohibition.
- **Floating point (5.4):** tilt compensation and magnetic calibration retain
  binary32 trigonometry/scaling and existing precision. Fixed point needs its
  own error budget and hardware validation. No double dependency is added.
- **Kernel ABI:** kernel aliases and FP registration match micro T-Kernel's
  interface. Its assembly ISR wrapper saves interrupt context; applying a raw
  compiler interrupt attribute to a TA_HLNG C callback would be incorrect.
- **Concurrent access:** volatile is not a lock. Queue payloads are copied by
  the kernel; statistics use their existing mutex. IDs are initialized before
  task startup and then read. The diagnostic counter has one writer and aligned
  word snapshots on the cacheless RP2040; it synchronizes no other data.

The earlier legacy-name/filename exceptions are resolved, not carried forward
as pending approval. No mandatory-rule deviations are proposed for scoped B4.
Kernel/SDK implementation and whole-system race freedom remain outside scope.

## Project terms and units

| Term | Meaning |
|---|---|
| B4 / IMU | Buddy 4 / inertial measurement unit. |
| I2C / GPIO / GP | Inter-integrated circuit bus / general-purpose input/output pin. |
| RTOS / ABI / ISR | Real-time operating system / application binary interface / interrupt service routine. |
| ACC / MAG / XYZ | Accelerometer / magnetometer / three sensor or robot axes. |
| BDU / LSB | Block data update / least significant bit. |
| mg / uT / ms | Thousandths of standard gravity / microtesla / milliseconds. |
| dps / cm | Degrees per second / centimetres. |
| IF / S4 / TA_HLNG | Integration interface / subsystem 4 / high-level kernel callback attribute. |

## Repeatable evidence

Run from the repository root:

```powershell
python testing/subsystem4/check_barr_style.py
python testing/subsystem4/test_barr_checker.py
powershell -NoProfile -ExecutionPolicy Bypass -File testing/subsystem4/host/run_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File testing/subsystem4/check_arm.ps1
python testing/subsystem4/format_b4.py --clang-format <path-to-clang-format>
```

The scanner now also checks module collisions, public prefixes/prototypes,
nesting, logical operands, switch defaults, final else and constant-first
equality. Manual review remains recorded above. The formatter adds case-aligned
breaks and natural-block spacing to the checked-in profile.

B4 builds enforce C99 and strict conversion warnings without changing the
kernel/SDK mode. ARM syntax checks cover real/mock and test/normal configurations.
Host tests cover display bounds/finiteness and magnetic overflow rejection.
Saved results are linked in `testing/subsystem4/TEST_LOG.md`.

Use `git -c core.whitespace=-blank-at-eof diff --check`: the final blank line is
required by 3.3.c. Desktop/build evidence does not replace a fresh Pico test.
