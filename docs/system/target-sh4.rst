.. _sh4-system-emulator:

SH4 System emulator
===================

QEMU emulates the SH7751R based ``r2d`` board and the SH7785 based
``sh7785lcr`` board.  Both use the ``qemu-system-sh4`` executable.

SH7785LCR board
---------------

The ``sh7785lcr`` machine models the R0P7785LC0011RL evaluation board with
512 MiB of DDR2.  The default 29-bit address mode exposes RAM at
``0x08000000``.  ``-M sh7785lcr,boot32=on`` selects the board's 32-bit boot
mode and exposes RAM at ``0x40000000`` through the SH7785 PMB.

The first QEMU serial backend is connected to SCIF1, matching connector CN5
on the board.  A directly loaded Linux kernel therefore uses
``console=ttySC1``.  For example::

  qemu-system-sh4 -M sh7785lcr -nographic \
      -kernel arch/sh/boot/zImage -append console=ttySC1

The machine includes these board devices:

* two interleaved x16 S29GL256P-compatible NOR devices on a 32-bit bus;
* the board PLD, including its LEDs, power-off register, version register,
  and the ``dipsw`` machine property;
* an SM501 revision C0 graphics controller;
* a PCA9564 I2C controller with an R2025S/D real-time clock;
* the SH7785 PCI controller with an RTL8169 in slot 0 and a SiI3512 in
  slot 1;
* an external R8A66597 two-port USB host controller; and
* a CG200-V2 dual SD/SDIO controller.

The R8A66597 has a USB keyboard attached by default.  A removable disk with
the identity used by a Kingston DataTraveler 2.0 can be selected with::

  -M sh7785lcr,usb-device=storage \
  -drive if=none,id=usbdisk,file=disk.img,format=raw

Use ``usb-device=none`` to leave both root ports empty.  The model also
exposes the controller's two DREQ outputs, two DACK inputs, and bidirectional
DEND signals as named GPIOs for DMA-controller integration.

SD images can be attached to the two CG200 slots as drive indexes 0 and 1::

  -drive if=sd,index=0,file=slot0.img,format=raw \
  -drive if=sd,index=1,file=slot1.img,format=raw

``-kernel`` accepts a zImage or an ELF vmlinux.  The default zImage boot
parameter page offset is ``0x1000``.  Kernels built with 8 KiB or 64 KiB
pages require ``zero-page-offset=0x2000`` or ``zero-page-offset=0x10000``.
An ELF vmlinux does not require this option because QEMU locates its
``boot_params_page`` symbol.

The SH7785 DMAC, R8A66597 peripheral mode, and cycle-accurate USB FIFO
arbitration are not modelled.
