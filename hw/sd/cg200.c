/*
 * D-Broad CG200-V2 dual SD/SDIO host controller
 *
 * The CG200 implements two SD Host Controller Specification 1.0 slots.
 * Its local-bus window exposes the slots 0x400 bytes apart, matching the
 * layout also visible through the CG200 PCI interface (1947:4743).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/sd/cg200.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define CG200_NUM_SLOTS       2
#define CG200_SLOT_STRIDE     0x400
#define CG200_MMIO_SIZE       0x10000

/* 50 MHz, 512-byte FIFO, high-speed, SDMA, and 3.3 V signaling. */
#define CG200_CAPABILITIES    0x01603232

struct CG200State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SDHCIState slot[CG200_NUM_SLOTS];
    qemu_irq irq;
    uint8_t irq_level;
};

static void cg200_set_irq(void *opaque, int n, int level)
{
    CG200State *s = opaque;

    s->irq_level = deposit32(s->irq_level, n, 1, level);
    qemu_set_irq(s->irq, s->irq_level != 0);
}

static void cg200_realize(DeviceState *dev, Error **errp)
{
    CG200State *s = CG200(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    for (unsigned int i = 0; i < CG200_NUM_SLOTS; i++) {
        DeviceState *slot = DEVICE(&s->slot[i]);
        SysBusDevice *slot_sbd = SYS_BUS_DEVICE(slot);

        qdev_prop_set_uint8(slot, "sd-spec-version", 1);
        qdev_prop_set_uint64(slot, "capareg", CG200_CAPABILITIES);
        if (!sysbus_realize(slot_sbd, errp)) {
            return;
        }
        memory_region_add_subregion(&s->iomem, i * CG200_SLOT_STRIDE,
                                    sysbus_mmio_get_region(slot_sbd, 0));
        sysbus_connect_irq(slot_sbd, 0, qdev_get_gpio_in(dev, i));
    }

    sysbus_init_irq(sbd, &s->irq);
}

static void cg200_reset_hold(Object *obj, ResetType type)
{
    CG200State *s = CG200(obj);

    s->irq_level = 0;
    qemu_set_irq(s->irq, 0);
}

static int cg200_post_load(void *opaque, int version_id)
{
    CG200State *s = opaque;

    qemu_set_irq(s->irq, s->irq_level != 0);
    return 0;
}

static const VMStateDescription vmstate_cg200 = {
    .name = TYPE_CG200,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = cg200_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(irq_level, CG200State),
        VMSTATE_END_OF_LIST()
    },
};

static void cg200_init(Object *obj)
{
    CG200State *s = CG200(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init(&s->iomem, obj, TYPE_CG200, CG200_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    qdev_init_gpio_in(dev, cg200_set_irq, CG200_NUM_SLOTS);

    for (unsigned int i = 0; i < CG200_NUM_SLOTS; i++) {
        g_autofree char *name = g_strdup_printf("slot%u", i);

        object_initialize_child(obj, name, &s->slot[i], TYPE_SYSBUS_SDHCI);
    }
}

SDBus *cg200_get_bus(CG200State *s, unsigned int slot)
{
    g_assert(slot < CG200_NUM_SLOTS);
    return SD_BUS(qdev_get_child_bus(DEVICE(&s->slot[slot]), "sd-bus"));
}

static void cg200_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = cg200_realize;
    dc->vmsd = &vmstate_cg200;
    rc->phases.hold = cg200_reset_hold;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
}

static const TypeInfo cg200_info = {
    .name = TYPE_CG200,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CG200State),
    .instance_init = cg200_init,
    .class_init = cg200_class_init,
};

static void cg200_register_types(void)
{
    type_register_static(&cg200_info);
}

type_init(cg200_register_types)
