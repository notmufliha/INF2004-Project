"""m0_cycles.py - count Cortex-M0+ instructions and cycles of one function.

Usage: python m0_cycles.py <elf> <function>|@<variable> ...

Loads the ELF's segments into a Unicorn ARM (Thumb, Cortex-M0) emulator,
calls each named function with no arguments (it returns to a stop address)
and counts every executed instruction.  Cycles use the RP2040 (Cortex-M0+)
timings from the ARM Cortex-M0+ Technical Reference Manual, table 3-1:
  1 cycle   most data-processing instructions, MULS (RP2040 has the fast
            single-cycle multiplier)
  2 cycles  loads and stores (LDR/STR and variants), taken branches B / Bcc,
            BX / BLX register
  3 cycles  BL
  1+N       PUSH/POP, LDM/STM with N registers; POP that loads PC +2
Not-taken conditional branches are 1 cycle.  Flash wait states / XIP cache
misses are not modelled: the result is the zero-wait-state figure (code in
SRAM or cache hit), the same for before and after, so the comparison is fair.

Requires: pip install unicorn capstone pyelftools
"""

import sys

from capstone import CS_ARCH_ARM, CS_MODE_THUMB, CS_MODE_MCLASS, Cs
from elftools.elf.elffile import ELFFile
from unicorn import UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB, UC_HOOK_CODE, Uc
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_SP

STOP = 0x0F000000
STACK_TOP = 0x20F00000
MCYCLE_HZ = 125_000_000


def load(elf_path):
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    syms = {}
    with open(elf_path, "rb") as f:
        elf = ELFFile(f)
        for seg in elf.iter_segments():
            if seg["p_type"] != "PT_LOAD":
                continue
            base = seg["p_paddr"] & ~0xFFF
            end = (seg["p_paddr"] + seg["p_memsz"] + 0xFFF) & ~0xFFF
            try:
                uc.mem_map(base, end - base)
            except Exception:
                pass  # overlapping page already mapped
            uc.mem_write(seg["p_paddr"], seg.data())
        for s in elf.get_section_by_name(".symtab").iter_symbols():
            if s["st_info"]["type"] in ("STT_FUNC", "STT_OBJECT"):
                syms[s.name] = s["st_value"] & ~1
    uc.mem_map(STACK_TOP - 0x10000, 0x10000)
    uc.mem_map(STOP, 0x1000)
    # RP2040 peripherals (IO_BANK0, ADC, ...) and SIO as plain RAM, so code
    # that reads GPIO levels or acknowledges interrupts can run.
    uc.mem_map(0x40000000, 0x100000)
    uc.mem_map(0xD0000000, 0x1000)
    return uc, syms


def cycles_of(insn, next_addr):
    m = insn.mnemonic.split(".")[0]
    ops = insn.op_str
    if m == "bl":
        return 3
    if m in ("bx", "blx"):
        return 2
    if m == "b":
        return 2  # unconditional, always taken
    if m.startswith("b") and len(m) == 3 and m not in ("bic", "bkpt"):
        # conditional branch: taken if the next executed address is the target
        target = int(ops.lstrip("#"), 16)
        return 2 if next_addr == target else 1
    if m in ("push", "pop", "ldm", "stm", "ldmia", "stmia"):
        n = ops.count(",") + 1 if "{" in ops else 1
        if "-" in ops:
            a, b = ops.strip("{}").split("-")
            n = int(b.strip()[1:]) - int(a.strip()[1:]) + 1
        extra = 2 if (m == "pop" and "pc" in ops) else 0
        return 1 + n + extra
    if m.startswith("ldr") or m.startswith("str"):
        return 2
    return 1


def run(uc, addr):
    md = Cs(CS_ARCH_ARM, CS_MODE_THUMB | CS_MODE_MCLASS)
    trace = []

    def hook(u, a, size, _):
        trace.append((a, size))

    h = uc.hook_add(UC_HOOK_CODE, hook)
    uc.reg_write(UC_ARM_REG_SP, STACK_TOP)
    uc.reg_write(UC_ARM_REG_LR, STOP | 1)
    uc.emu_start(addr | 1, STOP, count=5_000_000)
    uc.hook_del(h)
    cyc = 0
    for i, (a, size) in enumerate(trace):
        code = bytes(uc.mem_read(a, size))
        insn = next(md.disasm(code, a))
        nxt = trace[i + 1][0] if i + 1 < len(trace) else STOP
        cyc += cycles_of(insn, nxt)
    return len(trace), cyc


def main():
    uc, syms = load(sys.argv[1])
    for name in sys.argv[2:]:
        if name.startswith("@"):  # print a 32-bit variable, e.g. @result
            v = int.from_bytes(uc.mem_read(syms[name[1:]], 4), "little", signed=True)
            print(f"{name[1:]:32s} = {v}")
            continue
        n, c = run(uc, syms[name])
        print(f"{name:32s} {n:7d} instr {c:7d} cycles {c * 1e6 / MCYCLE_HZ:8.2f} us @125 MHz")


if __name__ == "__main__":
    main()
