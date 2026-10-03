/*
 * Renesas SH7785LCR (R0P7785LC0011RL) evaluation board
 *
 * Memory map (arch/sh/include/mach-common/mach/sh7785lcr.h in Linux, and
 * the SH7785 hardware manual 1.5 for the areas):
 *   0x00000000 NOR flash (CS0, 64 MiB, 32-bit bus), -drive if=pflash
 *   0x04000000 PLD registers (CS1)
 *   0x06000000 PCA9564 I2C (CS1)
 *   0x10000000 SM107/SM501 graphics (CS4)
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
#include "hw/ide/pci.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/pca9564.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/led.h"
#include "hw/scsi/scsi.h"
#include "hw/sd/cg200.h"
#include "hw/sd/sd.h"
#include "hw/usb/hcd-r8a66597.h"
#include "hw/usb/msd.h"
#include "migration/vmstate.h"
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
#define PLD_PCICR           0x00
#define PLD_LCD_BK_CONTR    0x02
#define PLD_LOCALCR         0x04
#define PLD_POFCR           0x06    /* write 1: power off */
#define PLD_LEDCR           0x08
#define PLD_SWSR            0x0a
#define PLD_VERSR           0x0c
#define PLD_MMSR            0x0e

#define PCA9564_BASE        0x06000000
#define SM501_VRAM_BASE     0x10000000
#define SM501_MMIO_BASE     0x13e00000
#define SM501_VRAM_SIZE     (4 * MiB)

#define USB_32BIT_BASE      0x08000000
#define USB_29BIT_BASE      0x14000000
#define CG200_32BIT_BASE    0x0c000000
#define CG200_29BIT_BASE    0x18000000

#define FRQMR1              0xffc80014
#define FRQMR1_MODE16       0x12252448
#define PCLK_HZ             50000000

#define TYPE_SH7785LCR_MACHINE MACHINE_TYPE_NAME("sh7785lcr")
OBJECT_DECLARE_SIMPLE_TYPE(SH7785LCRMachineState, SH7785LCR_MACHINE)

#define TYPE_SH7785LCR_PLD "sh7785lcr-pld"
OBJECT_DECLARE_SIMPLE_TYPE(SH7785LCRPLDState, SH7785LCR_PLD)

struct SH7785LCRPLDState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    uint16_t regs[8];
    uint8_t dipsw;
    uint16_t version;
    LEDState *led[8];
    LEDState *backlight;
};

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
    uint8_t dipsw;
    char *usb_device;
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

static uint16_t sh7785lcr_pld_reg(SH7785LCRPLDState *s, hwaddr addr)
{
    switch (addr & ~1) {
    case PLD_SWSR:
        return s->dipsw & 0x0f;
    case PLD_VERSR:
        return s->version;
    default:
        return s->regs[(addr & 0xf) / 2];
    }
}

static void sh7785lcr_pld_update_outputs(SH7785LCRPLDState *s)
{
    uint16_t ledcr = s->regs[PLD_LEDCR / 2];

    for (unsigned int i = 0; i < ARRAY_SIZE(s->led); i++) {
        led_set_state(s->led[i], ledcr & BIT(i));
    }
    led_set_state(s->backlight, s->regs[PLD_LCD_BK_CONTR / 2] != 0);
}

static uint64_t sh7785lcr_pld_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7785LCRPLDState *s = opaque;
    uint16_t value = sh7785lcr_pld_reg(s, addr);

    return size == 1 ? extract16(value, (addr & 1) * 8, 8) : value;
}

static void sh7785lcr_pld_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    SH7785LCRPLDState *s = opaque;
    unsigned int reg = (addr & 0xf) / 2;
    uint16_t value;

    if ((addr & ~1) == PLD_SWSR || (addr & ~1) == PLD_VERSR) {
        return;
    }
    value = s->regs[reg];
    if (size == 1) {
        value = deposit32(value, (addr & 1) * 8, 8, val);
    } else {
        value = val;
    }
    s->regs[reg] = value;

    if ((addr & ~1) == PLD_POFCR && (value & 1)) {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        return;
    }
    if ((addr & ~1) == PLD_LEDCR ||
        (addr & ~1) == PLD_LCD_BK_CONTR) {
        sh7785lcr_pld_update_outputs(s);
    }
}

static const MemoryRegionOps sh7785lcr_pld_ops = {
    .read = sh7785lcr_pld_read,
    .write = sh7785lcr_pld_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 2,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
};

static void sh7785lcr_pld_reset(DeviceState *dev)
{
    SH7785LCRPLDState *s = SH7785LCR_PLD(dev);

    memset(s->regs, 0, sizeof(s->regs));
    sh7785lcr_pld_update_outputs(s);
}

static int sh7785lcr_pld_post_load(void *opaque, int version_id)
{
    sh7785lcr_pld_update_outputs(opaque);
    return 0;
}

static const VMStateDescription vmstate_sh7785lcr_pld = {
    .name = TYPE_SH7785LCR_PLD,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = sh7785lcr_pld_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, SH7785LCRPLDState, 8),
        VMSTATE_END_OF_LIST()
    },
};

static const Property sh7785lcr_pld_properties[] = {
    DEFINE_PROP_UINT8("dipsw", SH7785LCRPLDState, dipsw, 0),
    DEFINE_PROP_UINT16("version", SH7785LCRPLDState, version, 1),
};

static void sh7785lcr_pld_realize(DeviceState *dev, Error **errp)
{
    SH7785LCRPLDState *s = SH7785LCR_PLD(dev);

    for (unsigned int i = 0; i < ARRAY_SIZE(s->led); i++) {
        g_autofree char *name = g_strdup_printf("LED%u", i + 3);

        s->led[i] = led_create_simple(OBJECT(dev), GPIO_POLARITY_ACTIVE_LOW,
                                      LED_COLOR_GREEN, name);
    }
    s->backlight = led_create_simple(OBJECT(dev), GPIO_POLARITY_ACTIVE_LOW,
                                     LED_COLOR_YELLOW, "LCD backlight");
    sh7785lcr_pld_update_outputs(s);
}

static void sh7785lcr_pld_init(Object *obj)
{
    SH7785LCRPLDState *s = SH7785LCR_PLD(obj);

    memory_region_init_io(&s->mmio, obj, &sh7785lcr_pld_ops, s,
                          TYPE_SH7785LCR_PLD, 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void sh7785lcr_pld_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sh7785lcr_pld_realize;
    device_class_set_legacy_reset(dc, sh7785lcr_pld_reset);
    device_class_set_props(dc, sh7785lcr_pld_properties);
    dc->vmsd = &vmstate_sh7785lcr_pld;
}

static const TypeInfo sh7785lcr_pld_info = {
    .name = TYPE_SH7785LCR_PLD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SH7785LCRPLDState),
    .instance_init = sh7785lcr_pld_init,
    .class_init = sh7785lcr_pld_class_init,
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

static void sh7785lcr_attach_usb_storage(USBBus *bus)
{
    DriveInfo *dinfo = drive_get(IF_NONE, 0, 0);
    DeviceState *bot;
    DeviceState *disk;

    if (!dinfo) {
        error_report("usb-device=storage requires an if=none drive");
        exit(1);
    }

    bot = qdev_new("usb-bot");
    qdev_prop_set_string(bot, "serial", "1");
    qdev_realize_and_unref(bot, BUS(bus), &error_fatal);

    disk = qdev_new("scsi-hd");
    qdev_prop_set_uint32(disk, "scsi-id", 0);
    qdev_prop_set_uint32(disk, "lun", 0);
    qdev_prop_set_bit(disk, "removable", true);
    qdev_prop_set_int32(disk, "scsi_version", 0);
    qdev_prop_set_string(disk, "vendor", "Kingston");
    qdev_prop_set_string(disk, "product", "DataTraveler 2.0");
    qdev_prop_set_string(disk, "ver", "PMAP");
    qdev_prop_set_drive_err(disk, "drive", blk_by_legacy_dinfo(dinfo),
                            &error_fatal);
    qdev_realize_and_unref(disk, &USB_STORAGE_DEV(bot)->bus.qbus,
                           &error_fatal);
}

static void sh7785lcr_init(MachineState *machine)
{
    MachineClass *mc = MACHINE_GET_CLASS(machine);
    SH7785LCRMachineState *s = SH7785LCR_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    PCIBus *pci_bus;
    PCIIDEState *sata;
    DeviceState *dev;
    SysBusDevice *sbd;
    I2CBus *i2c_bus;
    DeviceState *usb;
    SysBusDevice *usb_sbd;
    DeviceState *cg200;
    SysBusDevice *cg200_sbd;
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

    /* CN5, the board's physical serial console, is connected to SCIF1. */
    s->soc = sh7785_init(s->cpu, sysmem, PCLK_HZ, 1);
    sh7785_set_reg(s->soc, FRQMR1, FRQMR1_MODE16);
    s->pci_mem1 = sysbus_mmio_get_region(SYS_BUS_DEVICE(sh7785_pcic(s->soc)),
                                         SH7785_PCIC_MMIO_MEM1);
    memory_region_add_subregion_overlap(sysmem, 0x10000000, s->pci_mem1, 1);
    memory_region_set_enabled(s->pci_mem1, false);
    sh7785_set_mmselr_hook(s->soc, sh7785lcr_areasel, s);
    pci_bus = PCI_HOST_BRIDGE(sh7785_pcic(s->soc))->bus;

    /* Adrian's board has the RTL8169SC in slot 0 and SiI3512 in slot 1. */
    pci_init_nic_in_slot(pci_bus, mc->default_nic, NULL, "0");
    sata = PCI_IDE(pci_create_simple(pci_bus, PCI_DEVFN(1, 0), "sii3512"));
    dinfo = drive_get_by_index(IF_IDE, 0);
    if (dinfo) {
        /* The SSD in Adrian's boot log is connected to the second port. */
        ide_bus_create_drive(&sata->bus[1], 0, dinfo);
    }
    dinfo = drive_get_by_index(IF_IDE, 1);
    if (dinfo) {
        ide_bus_create_drive(&sata->bus[0], 0, dinfo);
    }
    pci_init_nic_devices(pci_bus, mc->default_nic);

    dev = qdev_new("sysbus-sm501");
    sbd = SYS_BUS_DEVICE(dev);
    qdev_prop_set_uint32(dev, "vram-size", SM501_VRAM_SIZE);
    qdev_prop_set_uint8(dev, "revision", 0xc0);
    qdev_prop_set_uint64(dev, "dma-offset", SM501_VRAM_BASE);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map_overlap(sbd, 0, SM501_VRAM_BASE, 2);
    sysbus_mmio_map_overlap(sbd, 1, SM501_MMIO_BASE, 2);
    sysbus_connect_irq(sbd, 0, sh7785_irq_pin(s->soc, 4));

    dev = qdev_new(TYPE_PCA9564);
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, PCA9564_BASE);
    sysbus_connect_irq(sbd, 0, sh7785_irq_pin(s->soc, 5));
    i2c_bus = I2C_BUS(qdev_get_child_bus(dev, "i2c"));
    i2c_slave_create_simple(i2c_bus, "r2025sd", 0x32);

    /* Two S29GL256P-compatible x16 devices on a 32-bit bank. */
    dinfo = drive_get(IF_PFLASH, 0, 0);
    pflash_cfi02_register_with_device_width(
        FLASH_BASE, "sh7785lcr.flash", FLASH_SIZE,
        dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
        256 * KiB, 1, 4, 2, 0x0001, 0x2201, 0, 0,
        0x555, 0x2aa, 0);

    dev = qdev_new(TYPE_SH7785LCR_PLD);
    qdev_prop_set_uint8(dev, "dipsw", s->dipsw);
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, PLD_BASE);

    usb = qdev_new(TYPE_R8A66597_USB_HOST);
    object_property_add_child(OBJECT(machine), "usb-host", OBJECT(usb));
    usb_sbd = SYS_BUS_DEVICE(usb);
    sysbus_realize_and_unref(usb_sbd, &error_fatal);
    memory_region_add_subregion_overlap(
        sysmem, s->boot32 ? USB_32BIT_BASE : USB_29BIT_BASE,
        sysbus_mmio_get_region(usb_sbd, 0), 2);
    sysbus_connect_irq(usb_sbd, 0, sh7785_irq_pin(s->soc, 0));
    if (!strcmp(s->usb_device, "keyboard")) {
        usb_create_simple(r8a66597_usb_bus(R8A66597_USB_HOST(usb)),
                          "usb-kbd");
    } else if (!strcmp(s->usb_device, "storage")) {
        sh7785lcr_attach_usb_storage(
            r8a66597_usb_bus(R8A66597_USB_HOST(usb)));
    }

    cg200 = qdev_new(TYPE_CG200);
    object_property_add_child(OBJECT(machine), "cg200", OBJECT(cg200));
    cg200_sbd = SYS_BUS_DEVICE(cg200);
    sysbus_realize_and_unref(cg200_sbd, &error_fatal);
    memory_region_add_subregion_overlap(
        sysmem, s->boot32 ? CG200_32BIT_BASE : CG200_29BIT_BASE,
        sysbus_mmio_get_region(cg200_sbd, 0), 2);
    sysbus_connect_irq(cg200_sbd, 0, sh7785_irq_pin(s->soc, 1));
    for (unsigned int i = 0; i < 2; i++) {
        DeviceState *card = qdev_new(TYPE_SD_CARD);
        DriveInfo *sd_dinfo = drive_get(IF_SD, 0, i);

        qdev_prop_set_drive_err(card, "drive",
                                sd_dinfo ? blk_by_legacy_dinfo(sd_dinfo) : NULL,
                                &error_fatal);
        qdev_realize_and_unref(card,
                               BUS(cg200_get_bus(CG200(cg200), i)),
                               &error_fatal);
    }

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

static char *sh7785lcr_get_usb_device(Object *obj, Error **errp)
{
    return g_strdup(SH7785LCR_MACHINE(obj)->usb_device);
}

static void sh7785lcr_set_usb_device(Object *obj, const char *value,
                                     Error **errp)
{
    SH7785LCRMachineState *s = SH7785LCR_MACHINE(obj);

    if (strcmp(value, "keyboard") && strcmp(value, "storage") &&
        strcmp(value, "none")) {
        error_setg(errp, "usb-device must be keyboard, storage, or none");
        return;
    }
    g_free(s->usb_device);
    s->usb_device = g_strdup(value);
}

static void sh7785lcr_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Renesas SH7785LCR (SH-4A)";
    mc->init = sh7785lcr_init;
    mc->default_cpu_type = TYPE_SH7785_CPU;
    mc->default_ram_size = DDR_SIZE;
    mc->default_ram_id = "sh7785lcr.sdram";
    mc->block_default_type = IF_IDE;
    mc->default_nic = "rtl8169";
    object_class_property_add_bool(oc, "boot32", sh7785lcr_get_boot32,
                                   sh7785lcr_set_boot32);
    object_class_property_set_description(oc, "boot32",
        "Mode pins select 32-bit boot (PMB, 32-bit physical addresses)");
    object_class_property_add_uint8_ptr(oc, "dipsw",
        offsetof(SH7785LCRMachineState, dipsw), OBJ_PROP_FLAG_READWRITE);
    object_class_property_set_description(oc, "dipsw",
        "Four-bit SW4 DIP switch value exposed by the board PLD");
    object_class_property_add_str(oc, "usb-device",
                                  sh7785lcr_get_usb_device,
                                  sh7785lcr_set_usb_device);
    object_class_property_set_description(oc, "usb-device",
        "Default R8A66597 attachment: keyboard, storage, or none");
    object_class_property_add(oc, "zero-page-offset", "uint32",
                              sh7785lcr_get_zero_page,
                              sh7785lcr_set_zero_page, NULL, NULL);
    object_class_property_set_description(oc, "zero-page-offset",
        "Boot parameter page of a zImage kernel: its CONFIG_ZERO_PAGE_OFFSET"
        " (an ELF vmlinux is looked up by symbol)");
}

static void sh7785lcr_instance_init(Object *obj)
{
    SH7785LCRMachineState *s = SH7785LCR_MACHINE(obj);

    s->zero_page = 0x1000;
    s->usb_device = g_strdup("keyboard");
}

static void sh7785lcr_instance_finalize(Object *obj)
{
    g_free(SH7785LCR_MACHINE(obj)->usb_device);
}

static const TypeInfo sh7785lcr_info = {
    .name = TYPE_SH7785LCR_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(SH7785LCRMachineState),
    .instance_init = sh7785lcr_instance_init,
    .instance_finalize = sh7785lcr_instance_finalize,
    .class_init = sh7785lcr_class_init,
};

static void sh7785lcr_register_types(void)
{
    type_register_static(&sh7785lcr_pld_info);
    type_register_static(&sh7785lcr_info);
}

type_init(sh7785lcr_register_types)
