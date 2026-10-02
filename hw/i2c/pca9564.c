/*
 * NXP PCA9564 parallel bus to I2C controller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/pca9564.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(PCA9564State, PCA9564)

#define PCA_STA 0
#define PCA_TO  0
#define PCA_DAT 1
#define PCA_ADR 2
#define PCA_CON 3

#define CON_AA      BIT(7)
#define CON_ENSIO   BIT(6)
#define CON_STA     BIT(5)
#define CON_STO     BIT(4)
#define CON_SI      BIT(3)
#define CON_CR_MASK 0x07

#define STA_START       0x08
#define STA_RESTART     0x10
#define STA_ADDR_W_ACK  0x18
#define STA_ADDR_W_NACK 0x20
#define STA_DATA_W_ACK  0x28
#define STA_DATA_W_NACK 0x30
#define STA_ADDR_R_ACK  0x40
#define STA_ADDR_R_NACK 0x48
#define STA_DATA_R_ACK  0x50
#define STA_DATA_R_NACK 0x58
#define STA_IDLE        0xf8

struct PCA9564State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;

    uint8_t status;
    uint8_t timeout;
    uint8_t data;
    uint8_t address;
    uint8_t control;
    bool active;
};

static void pca9564_complete(PCA9564State *s, uint8_t status)
{
    s->status = status;
    s->control &= CON_AA | CON_ENSIO | CON_CR_MASK;
    s->control |= CON_SI;
    qemu_irq_pulse(s->irq);
}

static void pca9564_stop(PCA9564State *s)
{
    if (s->active || i2c_bus_busy(s->bus)) {
        i2c_end_transfer(s->bus);
    }
    s->active = false;
    s->status = STA_IDLE;
    s->control &= CON_AA | CON_ENSIO | CON_CR_MASK;
    qemu_irq_lower(s->irq);
}

static void pca9564_advance(PCA9564State *s, uint8_t value)
{
    bool ack;

    s->control = value & (CON_AA | CON_ENSIO | CON_STA | CON_STO |
                          CON_CR_MASK);

    if (!(s->control & CON_ENSIO)) {
        pca9564_stop(s);
        s->control = value & (CON_AA | CON_CR_MASK);
        return;
    }

    if (s->control & CON_STO) {
        pca9564_stop(s);
        return;
    }

    if (s->control & CON_STA) {
        pca9564_complete(s, s->active ? STA_RESTART : STA_START);
        return;
    }

    if (s->status == STA_IDLE) {
        return;
    }

    switch (s->status) {
    case STA_START:
    case STA_RESTART:
        ack = i2c_start_transfer(s->bus, s->data >> 1, s->data & 1) == 0;
        s->active = ack;
        if (s->data & 1) {
            pca9564_complete(s, ack ? STA_ADDR_R_ACK : STA_ADDR_R_NACK);
        } else {
            pca9564_complete(s, ack ? STA_ADDR_W_ACK : STA_ADDR_W_NACK);
        }
        break;
    case STA_ADDR_W_ACK:
    case STA_DATA_W_ACK:
        ack = i2c_send(s->bus, s->data) == 0;
        pca9564_complete(s, ack ? STA_DATA_W_ACK : STA_DATA_W_NACK);
        break;
    case STA_ADDR_R_ACK:
    case STA_DATA_R_ACK:
        s->data = i2c_recv(s->bus);
        if (s->control & CON_AA) {
            pca9564_complete(s, STA_DATA_R_ACK);
        } else {
            i2c_nack(s->bus);
            pca9564_complete(s, STA_DATA_R_NACK);
        }
        break;
    default:
        pca9564_complete(s, s->status);
        break;
    }
}

static uint64_t pca9564_read(void *opaque, hwaddr addr, unsigned size)
{
    PCA9564State *s = opaque;

    switch (addr) {
    case PCA_STA:
        return s->status;
    case PCA_DAT:
        return s->data;
    case PCA_ADR:
        return s->address;
    case PCA_CON:
        return s->control;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pca9564: read from invalid register 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0xff;
    }
}

static void pca9564_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    PCA9564State *s = opaque;

    switch (addr) {
    case PCA_TO:
        s->timeout = value;
        break;
    case PCA_DAT:
        s->data = value;
        break;
    case PCA_ADR:
        s->address = value;
        break;
    case PCA_CON:
        pca9564_advance(s, value);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pca9564: write to invalid register 0x%" HWADDR_PRIx
                      "\n", addr);
        break;
    }
}

static const MemoryRegionOps pca9564_ops = {
    .read = pca9564_read,
    .write = pca9564_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 1,
};

static void pca9564_reset(DeviceState *dev)
{
    PCA9564State *s = PCA9564(dev);

    if (s->active || i2c_bus_busy(s->bus)) {
        i2c_end_transfer(s->bus);
    }
    s->status = STA_IDLE;
    s->timeout = 0;
    s->data = 0;
    s->address = 0;
    s->control = 0;
    s->active = false;
    qemu_irq_lower(s->irq);
}

static const VMStateDescription pca9564_vmstate = {
    .name = TYPE_PCA9564,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(status, PCA9564State),
        VMSTATE_UINT8(timeout, PCA9564State),
        VMSTATE_UINT8(data, PCA9564State),
        VMSTATE_UINT8(address, PCA9564State),
        VMSTATE_UINT8(control, PCA9564State),
        VMSTATE_BOOL(active, PCA9564State),
        VMSTATE_END_OF_LIST()
    },
};

static void pca9564_init(Object *obj)
{
    PCA9564State *s = PCA9564(obj);

    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
    memory_region_init_io(&s->iomem, obj, &pca9564_ops, s,
                          TYPE_PCA9564, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void pca9564_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, pca9564_reset);
    dc->vmsd = &pca9564_vmstate;
    dc->desc = "PCA9564 I2C controller";
}

static const TypeInfo pca9564_info = {
    .name = TYPE_PCA9564,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PCA9564State),
    .instance_init = pca9564_init,
    .class_init = pca9564_class_init,
};

static void pca9564_register_types(void)
{
    type_register_static(&pca9564_info);
}

type_init(pca9564_register_types)
