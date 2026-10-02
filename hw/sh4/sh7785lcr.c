/*
 * Renesas SH7785LCR (R0P7785LC0011RL) evaluation board
 *
 * Memory map (arch/sh/include/mach-common/mach/sh7785lcr.h in Linux, and
 * the SH7785 hardware manual 1.5 for the areas):
 *   0x00000000 NOR flash (CS0, 64 MiB, 32-bit bus), -drive if=pflash
 *   0x04000000 PLD registers (CS1)
 *   0x06000000 PCA9564 I2C (CS1)             - not modelled yet
 *   0x10000000 SM107 graphics (CS4)          - not modelled yet
 *   0x40000000 DDR2 SDRAM, 512 MiB, in the 32-bit physical space
 *              (DBSC0 to DBSC7). In 29-bit mode area 2 (0x08000000) and
 *              area 3 (0x0c000000) show DBSC2 and DBSC3, i.e. DDR2 offsets
 *              0x08000000 and 0x0c000000. Area 3 is DDR2 for every
 *              MMSELR.AREASEL value used here; area 2 only for AREASEL
 *              010, 011 and 100 (hardware manual figure 1.4).
 *
 * The board runs in clock mode 16 with a 33.33 MHz EXTAL (the factory DIP
 * switch settings Linux assumes in sh7785lcr_mode_pins()): FRQMR1 reads
 * H'1225 2448 (hardware manual table 15.3) and Pck is 50 MHz.
 *
 * -M sh7785lcr,boot32=on selects 32-bit boot with the mode pins: the CPU
 * starts with PASCR.SE set and P1/P2 mapped to the flash by the PMB.
 *
 * With -kernel the board sets up what the boot firmware would: in 29-bit
 * mode MMSELR.AREASEL = 010, in 32-bit mode PMB entries mapping P1
 * (cached) and P2 (uncached) to the 512 MiB of DDR2 at 0x40000000. A
 * zImage is loaded at MEMORY_START + 8 MiB and entered through P2, an ELF
 * vmlinux where it is linked. Boot parameters go to the zero page: for an
 * ELF vmlinux where its boot_params_page symbol says, for a zImage at
 * MEMORY_START + zero-page-offset (MEMORY_START is 0x08000000 or
 * 0x40000000). The offset is the kernel's CONFIG_ZERO_PAGE_OFFSET, which
 * follows the page size: 0x1000 (the default), 0x2000 for 8 KiB pages,
 * 0x10000 for 64 KiB pages.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "cpu.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "exec/tswap.h"
#include "elf.h"
#include "hw/block/flash.h"
#include "system/blockdev.h"
#include "hw/sh4/sh.h"
#include "hw/sh4/sh7785.h"
#include "hw/pci-host/sh7785_pcic.h"
#include "hw/pci/pci.h"
#include "hw/core/sysbus.h"
#include "hw/usb/hcd-r8a66597.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "system/system.h"

#define DDR_BASE            0x40000000
#define DDR_SIZE            (512 * MiB)
#define AREA2_BASE          0x08000000
#define AREA3_BASE          0x0c000000
#define AREA_SIZE           (64 * MiB)

#define LINUX_LOAD_OFFSET   0x00800000
#define INITRD_LOAD_OFFSET  0x01800000

#define FLASH_BASE          0x00000000
#define FLASH_SIZE          (64 * MiB)
#define PLD_BASE            0x04000000
#define PLD_POFCR           0x06    /* write 1: power off */
#define PLD_VERSR           0x0c

#define USB_32BIT_BASE      0x08000000
#define USB_29BIT_BASE      0x14000000

#define FRQMR1              0xffc80014
#define FRQMR1_MODE16       0x12252448
#define PCLK_HZ             50000000

#define TYPE_SH7785LCR_MACHINE MACHINE_TYPE_NAME("sh7785lcr")
OBJECT_DECLARE_SIMPLE_TYPE(SH7785LCRMachineState, SH7785LCR_MACHINE)

struct SH7785LCRMachineState {
    MachineState parent_obj;

    bool boot32;            /* mode pins: 32-bit boot */
    uint32_t zero_page;     /* zImage: CONFIG_ZERO_PAGE_OFFSET */
    SuperHCPU *cpu;
    SH7785State *soc;
    uint32_t vector;
    bool kernel;            /* -kernel: leave the firmware's state */
    MemoryRegion area2, area3;
    MemoryRegion *pci_mem1;
};

static void sh7785lcr_areasel(void *opaque, int areasel)
{
    SH7785LCRMachineState *s = opaque;

    memory_region_set_enabled(&s->area2, areasel >= 2 && areasel <= 4);
    /* Figure 1.4: area 4 is PCI memory space 1 for AREASEL 001, 011, 111 */
    memory_region_set_enabled(s->pci_mem1,
                              areasel == 1 || areasel == 3 || areasel == 7);
}

static void main_cpu_reset(void *opaque)
{
    SH7785LCRMachineState *s = opaque;
    CPUSH4State *env = &s->cpu->env;

    cpu_reset(CPU(s->cpu));
    if (s->boot32) {
        sh7785_reset_32bit_boot(s->soc);
    }
    if (s->kernel && s->boot32) {
        /* What 32-bit boot firmware leaves for the kernel */
        env->pmb[0] = (pmb_t) { .vpn = 0x80, .ppn = DDR_BASE >> 24, .v = 1,
                                .sz = 3, .c = 1, .ub = 0, .wt = 0 };
        env->pmb[1] = (pmb_t) { .vpn = 0xa0, .ppn = DDR_BASE >> 24, .v = 1,
                                .sz = 3, .c = 0, .ub = 0, .wt = 0 };
    }
    env->pc = s->vector;
}

/* vmlinux is linked at P1 addresses */
static uint64_t sh7785lcr_elf_to_phys(void *opaque, uint64_t addr)
{
    SH7785LCRMachineState *s = opaque;

    return s->boot32 ? DDR_BASE + (addr & 0x1fffffff) : addr & 0x1fffffff;
}

/* PLD: only the registers the kernel and the test rig use */
static uint64_t pld_read(void *opaque, hwaddr addr, unsigned size)
{
    uint16_t *regs = opaque;

    if (addr == PLD_VERSR) {
        return 0x0001;
    }
    return addr < 0x10 ? regs[addr / 2] : 0;
}

static void pld_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    uint16_t *regs = opaque;

    if (addr == PLD_POFCR && (val & 1)) {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        return;
    }
    if (addr < 0x10) {
        regs[addr / 2] = val;
    }
}

static const MemoryRegionOps pld_ops = {
    .read = pld_read,
    .write = pld_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 2,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
};

static struct QEMU_PACKED {
    int32_t mount_root_rdonly;
    int32_t ramdisk_flags;
    int32_t orig_root_dev;
    int32_t loader_type;
    int32_t initrd_start;
    int32_t initrd_size;
    char pad[232];
    char kernel_cmdline[256] QEMU_NONSTRING;
} boot_params;

/* head_32.S: the boot parameters the kernel reads, at _text */
static uint64_t boot_params_sym;

static void sh7785lcr_elf_sym(const char *name, int info, uint64_t value,
                              uint64_t size)
{
    if (!strcmp(name, "boot_params_page")) {
        boot_params_sym = value;
    }
}

static void sh7785lcr_load_kernel(SH7785LCRMachineState *s,
                                  MachineState *machine)
{
    hwaddr mem_start = s->boot32 ? DDR_BASE : AREA2_BASE;
    bool elf_kernel;
    uint64_t entry;
    hwaddr zero_page;
    void *params;

    s->kernel = true;
    /*
     * An ELF vmlinux is loaded where it is linked, which avoids the zImage
     * decompressor's size limit. Anything else is a zImage, run from
     * BOOT_LINK_OFFSET.
     */
    boot_params_sym = 0;
    elf_kernel = load_elf_ram_sym(machine->kernel_filename, NULL,
                                  sh7785lcr_elf_to_phys, s, &entry, NULL,
                                  NULL, NULL, ELFDATA2LSB, EM_SH, 0, 0,
                                  NULL, true, sh7785lcr_elf_sym) > 0;
    if (elf_kernel) {
        s->vector = entry;
    } else if (load_image_targphys(machine->kernel_filename,
                                   mem_start + LINUX_LOAD_OFFSET,
                                   INITRD_LOAD_OFFSET - LINUX_LOAD_OFFSET,
                                   NULL) < 0) {
        error_report("could not load kernel '%s'", machine->kernel_filename);
        exit(1);
    } else {
        s->vector = 0xa0000000 | ((mem_start + LINUX_LOAD_OFFSET) &
                                  0x1fffffff);
    }
    if (!s->boot32) {
        /* what 29-bit boot firmware leaves behind */
        sh7785_preset_mmselr(s->soc, 2);
    }
    /*
     * U-Boot sets PCIMBAR0 so PCI bus addresses equal local ones, and
     * Linux (pci-sh7780.c) rewrites PCILAR0/PCILSR0 but relies on it.
     */
    sh7785_pcic_preset_target(sh7785_pcic(s->soc), mem_start);

    memset(&boot_params, 0, sizeof(boot_params));
    if (machine->initrd_filename) {
        int size = load_image_targphys(machine->initrd_filename,
                                       mem_start + INITRD_LOAD_OFFSET,
                                       128 * MiB - INITRD_LOAD_OFFSET, NULL);
        if (size < 0) {
            error_report("could not load initrd '%s'",
                         machine->initrd_filename);
            exit(1);
        }
        boot_params.loader_type = tswap32(1);
        boot_params.initrd_start = tswap32(INITRD_LOAD_OFFSET);
        boot_params.initrd_size = tswap32(size);
    }
    if (machine->kernel_cmdline) {
        strncpy(boot_params.kernel_cmdline, machine->kernel_cmdline,
                sizeof(boot_params.kernel_cmdline));
    }
    /*
     * The zero page is part of an ELF kernel image, so write the boot
     * parameters into the loaded image instead of adding an overlapping
     * ROM blob.
     */
    zero_page = elf_kernel && boot_params_sym ?
                sh7785lcr_elf_to_phys(s, boot_params_sym) :
                mem_start + s->zero_page;
    params = elf_kernel ? rom_ptr(zero_page, sizeof(boot_params)) : NULL;
    if (params) {
        memcpy(params, &boot_params, sizeof(boot_params));
    } else {
        rom_add_blob_fixed("boot_params", &boot_params, sizeof(boot_params),
                           zero_page);
    }
}

static void sh7785lcr_init(MachineState *machine)
{
    MachineClass *mc = MACHINE_GET_CLASS(machine);
    SH7785LCRMachineState *s = SH7785LCR_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *pld = g_new(MemoryRegion, 1);
    DeviceState *usb;
    SysBusDevice *usb_sbd;
    DriveInfo *dinfo;

    if (machine->ram_size != DDR_SIZE) {
        error_report("sh7785lcr has %d MiB of RAM", (int)(DDR_SIZE / MiB));
        exit(1);
    }

    s->cpu = SUPERH_CPU(cpu_create(machine->cpu_type));
    /* Power-on reset starts at H'A000 0000 (P2 view of the boot flash) */
    s->vector = 0xa0000000;
    qemu_register_reset(main_cpu_reset, s);

    memory_region_add_subregion(sysmem, DDR_BASE, machine->ram);
    memory_region_init_alias(&s->area2, NULL, "sh7785lcr.sdram-area2",
                             machine->ram, AREA2_BASE, AREA_SIZE);
    memory_region_init_alias(&s->area3, NULL, "sh7785lcr.sdram-area3",
                             machine->ram, AREA3_BASE, AREA_SIZE);
    memory_region_add_subregion(sysmem, AREA2_BASE, &s->area2);
    memory_region_add_subregion(sysmem, AREA3_BASE, &s->area3);

    s->soc = sh7785_init(s->cpu, sysmem, PCLK_HZ);
    sh7785_set_reg(s->soc, FRQMR1, FRQMR1_MODE16);
    s->pci_mem1 = sysbus_mmio_get_region(SYS_BUS_DEVICE(sh7785_pcic(s->soc)),
                                         SH7785_PCIC_MMIO_MEM1);
    memory_region_add_subregion_overlap(sysmem, 0x10000000, s->pci_mem1, 1);
    memory_region_set_enabled(s->pci_mem1, false);
    sh7785_set_mmselr_hook(s->soc, sh7785lcr_areasel, s);
    pci_init_nic_devices(PCI_HOST_BRIDGE(sh7785_pcic(s->soc))->bus,
                         mc->default_nic);

    /*
     * NOR flash: Linux registers it as physmap-flash with bankwidth 4.
     * TODO(manual): the exact part is not verified; AMD command set with
     * Spansion S29GL512 IDs is assumed.
     */
    dinfo = drive_get(IF_PFLASH, 0, 0);
    pflash_cfi02_register(FLASH_BASE, "sh7785lcr.flash", FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          128 * KiB, 1, 4, 0x0001, 0x227e, 0x2223, 0x2201,
                          0x555, 0x2aa, 0);

    memory_region_init_io(pld, NULL, &pld_ops, g_new0(uint16_t, 8),
                          "sh7785lcr-pld", 0x10);
    memory_region_add_subregion(sysmem, PLD_BASE, pld);

    usb = qdev_new(TYPE_R8A66597_USB_HOST);
    usb_sbd = SYS_BUS_DEVICE(usb);
    sysbus_realize_and_unref(usb_sbd, &error_fatal);
    memory_region_add_subregion_overlap(
        sysmem, s->boot32 ? USB_32BIT_BASE : USB_29BIT_BASE,
        sysbus_mmio_get_region(usb_sbd, 0), 2);
    sysbus_connect_irq(usb_sbd, 0, sh7785_irq_pin(s->soc, 0));
    usb_create_simple(r8a66597_usb_bus(R8A66597_USB_HOST(usb)), "usb-kbd");

    if (machine->kernel_filename) {
        sh7785lcr_load_kernel(s, machine);
    }
}

static void sh7785lcr_get_zero_page(Object *obj, Visitor *v,
                                    const char *name, void *opaque,
                                    Error **errp)
{
    visit_type_uint32(v, name, &SH7785LCR_MACHINE(obj)->zero_page, errp);
}

static void sh7785lcr_set_zero_page(Object *obj, Visitor *v,
                                    const char *name, void *opaque,
                                    Error **errp)
{
    visit_type_uint32(v, name, &SH7785LCR_MACHINE(obj)->zero_page, errp);
}

static bool sh7785lcr_get_boot32(Object *obj, Error **errp)
{
    return SH7785LCR_MACHINE(obj)->boot32;
}

static void sh7785lcr_set_boot32(Object *obj, bool value, Error **errp)
{
    SH7785LCR_MACHINE(obj)->boot32 = value;
}

static void sh7785lcr_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Renesas SH7785LCR (SH-4A)";
    mc->init = sh7785lcr_init;
    mc->default_cpu_type = TYPE_SH7785_CPU;
    mc->default_ram_size = DDR_SIZE;
    mc->default_ram_id = "sh7785lcr.sdram";
    mc->default_nic = "rtl8139";   /* e1000 needs a kernel fix (port I/O) */
    object_class_property_add_bool(oc, "boot32", sh7785lcr_get_boot32,
                                   sh7785lcr_set_boot32);
    object_class_property_set_description(oc, "boot32",
        "Mode pins select 32-bit boot (PMB, 32-bit physical addresses)");
    object_class_property_add(oc, "zero-page-offset", "uint32",
                              sh7785lcr_get_zero_page,
                              sh7785lcr_set_zero_page, NULL, NULL);
    object_class_property_set_description(oc, "zero-page-offset",
        "Boot parameter page of a zImage kernel: its CONFIG_ZERO_PAGE_OFFSET"
        " (an ELF vmlinux is looked up by symbol)");
}

static void sh7785lcr_instance_init(Object *obj)
{
    SH7785LCR_MACHINE(obj)->zero_page = 0x1000;
}

static const TypeInfo sh7785lcr_info = {
    .name = TYPE_SH7785LCR_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(SH7785LCRMachineState),
    .instance_init = sh7785lcr_instance_init,
    .class_init = sh7785lcr_class_init,
};

static void sh7785lcr_register_types(void)
{
    type_register_static(&sh7785lcr_info);
}

type_init(sh7785lcr_register_types)
