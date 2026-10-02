# qemu-superh

A fork of [QEMU](https://www.qemu.org/) for SuperH (SH-4/SH-4A), made to
test the Linux kernel on SuperH without hardware. It is not tracking
upstream QEMU: it branched from QEMU master in September 2026
(efa3b9d5ac) and is developed here only.

Most of this work was written with an LLM agent. QEMU does not accept
AI-generated contributions, so QEMU bugs found here are reported
upstream as issues with reproducers, not as patches.

## What it adds

- **SH7785LCR board** (`-M sh7785lcr`): SH7785 (SH-4A) with TLB extended
  mode, 32-bit address mode with the PMB (`-M sh7785lcr,boot32=on`),
  the SH7785 interrupt controller, SCIF0-5, TMU0-5, 512 MiB DDR2, CFI
  flash, SM501 display, PCA9564 I2C with an R2025SD RTC, R8A66597 USB,
  and the PCI host controller with an RTL8169SC NIC and SiI3512 SATA.
  Mainline Linux (`sh7785lcr_defconfig`, `sh7785lcr_32bit_defconfig`)
  and U-Boot boot on it.
- **SH-4 fixes** on `r2d` and in the CPU: TLB entry flushing, associative
  TLB writes, 1 KiB pages, interrupt delivery after RTE, SH7750 INTC
  priorities, MMUCR, timers, serial input, reset vector, CVR cache sizes.
- **Cache models for finding kernel bugs**. QEMU keeps caches coherent,
  real SH-4 does not. These report what silicon would get wrong, with
  `-d exact`:
  - `-global superh-cpu.x-exact-icache=on`: code rewritten and executed
    without invalidating the instruction cache
  - `-global superh-cpu.x-exact-dcache=on`: code executed while still
    dirty in the operand cache
  - `-d exact` alone also checks LDTLB operands (wrong page sizes)
- `-kernel` takes an ELF vmlinux or a zImage.

## Quick start

    ./configure --target-list=sh4-softmmu && make
    qemu-system-sh4 -M sh7785lcr -nographic -serial stdio \
        -kernel vmlinux -append console=ttySC0,115200

## Linux bugs found with it

Heap overflow in the SH interrupt setup, double mmap_lock release in SH
page faults, I-cache invalidation missing with 8K/64K pages and on
SH-X3, port I/O on SH PCI boards, SLUB debug regression, SH7785
interrupt table errors. Patches go to the linux-sh list.

Testing on real SuperH hardware is welcome.

The original QEMU README is in `README.rst`.
