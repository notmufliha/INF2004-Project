# B1 BARR-C:2018 review

Scope: `app_program/subsystem1/*`, `app_program/fault.[ch]`,
`app_program/fault_flags.h`, `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.[ch]`
and `testing/subsystem1/host/*`. Kernel, SDK and other buddies' code are out of
scope. The only lines changed in the template driver `cyw43_utk.c` are the re-join
fix; the rest of that file keeps the template's style.

## Evidence (run from the repository root)

```sh
python testing/subsystem1/check_barr_style.py     # B4's rule engine + line checks: PASS
sh testing/subsystem1/host/run_host_tests.sh      # host tests, C99 -Wconversion -Werror
```

The firmware build compiles every B1 file with
`-std=c99 -Wall -Wextra -Wconversion -Wsign-conversion -Werror`
(`build_make/mtkernel_3/app_program/subdir.mk`), the same profile as B4.
The scanner covers only selected rules; the table below is the manual review.

## Rule-by-rule

| Rule | How B1 meets it |
|---|---|
| 1.1 C dialect | C99 for every B1 object, warnings as errors. |
| 1.2 Width | 80 columns, checked. |
| 1.3 Braces | Allman, every if/else/for/while/switch body braced. |
| 1.4 Parentheses | Each operand of && / \|\| that is not a plain name is parenthesised (scanner). |
| 1.6 Casts | Every narrowing or enum cast has a comment saying why the value fits. |
| 1.7 Keywords | No goto, continue, register, auto; `for (;;)` for task loops. |
| 1.8 Qualifiers | File-private data and helpers `static`; read-only inputs `const`; single-writer link states and ISR-read data `volatile`. |
| 2.1, 2.2 Comments | `/* */` only, no code in comments; `@file` per module and `@brief` per function. |
| 3.1-3.6 White space | 4 spaces, no tabs, LF (`.gitattributes`), no trailing spaces, end-of-file marker. |
| 4.1 Modules | Lower-case names, unique in their first 8 characters within B1. |
| 4.2, 4.3 Headers/sources | One header per source, include guards, each .c includes its own header first; public functions defined before private ones. |
| 5.1 Types | New types are lower-case `module_xxx_t` (`comm_telemetry_t`, `s1_link_wifi_action_t`). |
| 5.2 Widths | `uint8_t`..`uint32_t`, `int16_t`, `int32_t`; kernel `ID`/`ER`/`INT` only at kernel calls. Bit-fields use `UINT` (C99 allows only int types for portable bit-fields). |
| 5.3 Signedness | No signed bit-fields: speeds and pitch are stored as two's complement patterns (`comm_pack_twos()` / `comm_pack_sign_extend()`); bit operations only on unsigned values. |
| 5.4 Floating point | None in B1. |
| 5.5 Structs | Bit-field structs never leave the Pico; the wire format is packed with shifts (`comm_pack.c`), so compiler layout cannot change it. |
| 5.6 Booleans | `bool` with `b_` prefix. |
| 6.1 Names | Public functions carry the module name (`comm_`, `comm_pack_`, `s1_link_`, `fault_`, `mqtt_utk_`); macros upper-case. |
| 6.2 Functions | Prototypes in the module's own header, `static` helpers, all under 100 lines. |
| 6.3 Macros | No function-like macros in B1 (the old `NAME_OF` became `name_of()`). |
| 6.4 Tasks | Task entries end in `_task`; the cyclic handler ends in `_isr`. |
| 7.1 Variables | At least 3 characters; `g_` file globals, `gh_` kernel handles, `p_`/`pp_` pointers, `b_`/`gb_` Booleans. |
| 7.2 Initialisation | Globals initialised at definition; locals before use. |
| 8.2 Conditionals | At most two nested ifs; every else-if chain ends in else; no assignments in conditions. |
| 8.3 Switch | Every switch has default; breaks aligned with their case. |
| 8.4 Loops | Named bounds; queue-drain loops bounded by the queue length. |
| 8.6 Equality | Constant on the left (`E_OK == result`). |

## Deviations (agreed, kept visible in the scanner's ALLOWED list)

| ID | Rule | Where | Why | Plan |
|---|---|---|---|---|
| DEV-1 | 5.1.c, 7.1, 6.1 | `subsystem1.h` aliases `ROBOT_STATE`, `REMOTE_CMD`, ...; globals `mbf_remote_cmd`, `flg_faults`, `tid_led`; functions `comm_set_xxx` (module `comm`) | B2, B3, B4, B5 and the Vehicle Controller use these names in about 15 files; renaming them mid-integration would break everyone's build | teammates may move to the `comm_xxx_t` names; delete the aliases once nothing uses them |
| DEV-2 | 4.2.a | `fault_flags.h` has no `fault_flags.c` | it is the team's bit-assignment table; B5's contract test reads it; functions are in `fault.h`/`fault.c` | none |
| DEV-3 | 4.1.b | `subsystem1.c` shares its first 8 characters with `subsystem2/4/5.c` | the `subsystemN` file names are the template's team-wide convention | team decision |
| DEV-4 | 1.6 | `(FP)` casts of task entries and the cyclic handler | the kernel's `T_CTSK`/`T_CCYC` take a generic `FP` | kernel API |

`error_codes.h` (shared with B2 and B5) keeps its `ERR_CODE()` macro: case
labels and `#define` constants need a constant expression, which a function
cannot provide (BARR-C 6.3 allows this).
