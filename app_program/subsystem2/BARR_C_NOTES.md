# Subsystem 2 coding conventions

The reference is Barr Group's Embedded C Coding Standard, BARR-C:2018.
This refactor applies its source conventions to subsystem 2 and updates
the consumers of renamed public types and resource handles.

Run the automated subset check from the repository root:

```text
node tools/check-subsystem2-style.cjs
```

The checker covers tabs, trailing whitespace, 80-column lines, prohibited
integer keywords, infinite-loop form, control-block braces and the
100-line function limit. It does not certify every rule. The local
`.clang-format` records the formatting settings for future edits.

## Integration constraints and deviations to review

- Keep the micro T-Kernel/BSP API types and constants (`INT`, `UINT`, `UW`,
  `ER`, `ID`, `T_MSG`, `EXPORT`, `IMPORT`, etc.). These definitions belong
  to the platform; renaming them locally would obscure the API contract.
  Numeric 64-bit application calculations now use `int64_t`/`uint64_t`.
- Keep the RTOS `TA_HLNG` interrupt installation mechanism. The kernel
  supplies the exception wrapper; the registered C handler is an ordinary
  C callback, rather than a compiler-specific naked interrupt entry.
- Keep the established `motor_`, `encoder_`, `pid_`, `motion_`, `b2_` and
  `subsystem2_` public API prefixes. Some differ from their file basenames.
  Test and subsystem filenames also share their first eight characters.
  These are existing project integration conventions (sections 4.1/6.1).
- Retain externally visible semaphore, pool and PI task handles where the
  current startup and consumer interfaces require them. They use `gh_`.
  Header-level extern declarations are discouraged, rather than forbidden,
  by section 4.2.c.i.
- `ENC_EDGE_MASK` remains a parameterized macro because it is used to
  construct a compile-time register mask. A function call cannot provide
  the same integer constant expression. Locking macros were replaced by
  private functions.
- Register addresses and `tm_printf` casts follow the BSP interface.
  The BSP's console signature accepts `UB *`; diagnostic literals are
  passed to it using the existing API convention.
- Forward error jumps in the PI task converge on motor shutdown. Early
  returns elsewhere keep validation and cleanup paths readable; a single
  return is a preference in section 6.2.c.

These notes explain integration constraints; they do not assert project
manager approval of deviations or full BARR-C certification. Review them
when deciding which project-specific conventions to adopt.

Local units: `ms` means milliseconds, `cps` means encoder counts per
second, `prc` means processor, `stacd` means RTOS task start code, and
`exinf` means RTOS extended information. Message structures remain native
in-process mailbox data, rather than serialized wire formats.

PI gains, PWM bounds, command validation, watchdog thresholds and safety
generation rules are intended to retain their existing behavior. Hardware
timing and motor/encoder behavior must still be checked on the robot.
