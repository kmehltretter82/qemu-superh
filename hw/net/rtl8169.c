/*
 * Realtek RTL8169SC/RTL8110SC Gigabit Ethernet controller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "hw/core/qdev-properties.h"
#include "hw/net/mii.h"
#include "hw/pci/pci_device.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/system.h"

#include "net_tx_pkt.h"

#define TYPE_RTL8169 "rtl8169"
OBJECT_DECLARE_SIMPLE_TYPE(RTL8169State, RTL8169)

#define RTL8169_REGS_SIZE       0x100
#define RTL8169_DESC_SIZE       16
#define RTL8169_MAX_DESCS       256
#define RTL8169_TX_BUFFER_SIZE  65536

enum RTL8169Registers {
    MAC0                = 0x00,
    MAR0                = 0x08,
    CounterAddrLow      = 0x10,
    CounterAddrHigh     = 0x14,
    TxDescStartAddrLow  = 0x20,
    TxDescStartAddrHigh = 0x24,
    TxHDescStartAddrLow = 0x28,
    TxHDescStartAddrHigh = 0x2c,
    ChipCmd             = 0x37,
    TxPoll              = 0x38,
    IntrMask            = 0x3c,
    IntrStatus          = 0x3e,
    TxConfig            = 0x40,
    RxConfig            = 0x44,
    RxMissed            = 0x4c,
    Cfg9346             = 0x50,
    Config0             = 0x51,
    Config1             = 0x52,
    Config2             = 0x53,
    Config3             = 0x54,
    Config4             = 0x55,
    Config5             = 0x56,
    MultiIntr           = 0x5c,
    PHYAR               = 0x60,
    PHYstatus           = 0x6c,
    RxMaxSize           = 0xda,
    CPlusCmd            = 0xe0,
    IntrMitigate        = 0xe2,
    RxDescAddrLow       = 0xe4,
    RxDescAddrHigh      = 0xe8,
    EarlyTxThres        = 0xec,
};

enum RTL8169InterruptBits {
    SYSErr        = 0x8000,
    PCSTimeout    = 0x4000,
    SWInt         = 0x0100,
    TxDescUnavail = 0x0080,
    RxFIFOOver    = 0x0040,
    LinkChg       = 0x0020,
    RxOverflow    = 0x0010,
    TxErr         = 0x0008,
    TxOK          = 0x0004,
    RxErr         = 0x0002,
    RxOK          = 0x0001,
};

enum RTL8169CommandBits {
    CmdReset = 0x10,
    CmdRxEnb = 0x08,
    CmdTxEnb = 0x04,
};

enum RTL8169RxModeBits {
    AcceptBroadcast = 0x08,
    AcceptMulticast = 0x04,
    AcceptMyPhys    = 0x02,
    AcceptAllPhys   = 0x01,
};

enum RTL8169DescBits {
    DescOwn   = BIT(31),
    RingEnd   = BIT(30),
    FirstFrag = BIT(29),
    LastFrag  = BIT(28),
    TxLargeSend = BIT(27),
};

#define TX_IP_CHECKSUM   BIT(18)
#define TX_UDP_CHECKSUM  BIT(17)
#define TX_TCP_CHECKSUM  BIT(16)
#define TX_VLAN_TAG      BIT(17)
#define DESC_LEN_MASK    0xffff
#define RX_LEN_MASK      0x3fff
#define PHY_STATUS_UP    0x13

typedef struct RTL8169Counters {
    uint64_t tx_packets;
    uint64_t rx_packets;
    uint64_t tx_errors;
    uint32_t rx_errors;
    uint16_t rx_missed;
    uint16_t align_errors;
    uint32_t tx_one_collision;
    uint32_t tx_multi_collision;
    uint64_t rx_unicast;
    uint64_t rx_broadcast;
    uint32_t rx_multicast;
    uint16_t tx_aborted;
    uint16_t tx_underrun;
} RTL8169Counters;

struct RTL8169State {
    PCIDevice parent_obj;

    MemoryRegion io_bar;
    MemoryRegion mem_bar;
    NICState *nic;
    NICConf conf;

    uint8_t regs[RTL8169_REGS_SIZE];
    uint16_t phy[2][32];
    uint16_t phy_page;
    uint32_t tx_desc;
    uint32_t tx_high_desc;
    uint32_t rx_desc;
    uint32_t tx_len;
    uint32_t tx_opts1;
    uint32_t tx_opts2;
    uint8_t tx_buffer[RTL8169_TX_BUFFER_SIZE];
    RTL8169Counters counters;
    bool link_down;
    QEMUTimer *autoneg_timer;

    struct NetTxPkt *tx_pkt;
};

static uint16_t rtl8169_regw(RTL8169State *s, unsigned reg)
{
    return lduw_le_p(&s->regs[reg]);
}

static uint32_t rtl8169_regl(RTL8169State *s, unsigned reg)
{
    return ldl_le_p(&s->regs[reg]);
}

static uint64_t rtl8169_regq_pair(RTL8169State *s, unsigned low,
                                  unsigned high)
{
    return rtl8169_regl(s, low) | (uint64_t)rtl8169_regl(s, high) << 32;
}

static void rtl8169_setw(RTL8169State *s, unsigned reg, uint16_t value)
{
    stw_le_p(&s->regs[reg], value);
}

static void rtl8169_setl(RTL8169State *s, unsigned reg, uint32_t value)
{
    stl_le_p(&s->regs[reg], value);
}

static bool rtl8169_bus_master_enabled(RTL8169State *s)
{
    return pci_get_word(s->parent_obj.config + PCI_COMMAND) &
           PCI_COMMAND_MASTER;
}

static void rtl8169_update_irq(RTL8169State *s)
{
    pci_set_irq(&s->parent_obj,
                (rtl8169_regw(s, IntrStatus) &
                 rtl8169_regw(s, IntrMask)) != 0);
}

static void rtl8169_raise_interrupt(RTL8169State *s, uint16_t bits)
{
    rtl8169_setw(s, IntrStatus, rtl8169_regw(s, IntrStatus) | bits);
    rtl8169_update_irq(s);
}

static void rtl8169_autoneg_complete(void *opaque)
{
    RTL8169State *s = opaque;

    if (s->link_down) {
        return;
    }
    s->phy[0][MII_BMSR] |= MII_BMSR_LINK_ST | MII_BMSR_AN_COMP;
    s->regs[PHYstatus] = PHY_STATUS_UP;
    rtl8169_raise_interrupt(s, LinkChg);
    qemu_flush_queued_packets(qemu_get_queue(s->nic));
}

static void rtl8169_reset_phy(RTL8169State *s)
{
    timer_del(s->autoneg_timer);
    memset(s->phy, 0, sizeof(s->phy));
    s->phy_page = 0;
    s->phy[0][MII_BMCR] = MII_BMCR_AUTOEN | MII_BMCR_FD |
                          MII_BMCR_SPEED1000;
    s->phy[0][MII_BMSR] = MII_BMSR_100TX_FD | MII_BMSR_100TX_HD |
                          MII_BMSR_10T_FD | MII_BMSR_10T_HD |
                          MII_BMSR_EXTSTAT | MII_BMSR_AN_COMP |
                          MII_BMSR_AUTONEG | MII_BMSR_EXTCAP;
    s->phy[0][MII_PHYID1] = 0x001c;
    s->phy[0][MII_PHYID2] = 0xc912;
    s->phy[0][MII_ANAR] = 0x05e1;
    s->phy[0][MII_ANLPAR] = 0xc5e1;
    s->phy[0][MII_ANER] = MII_ANER_NWAY;
    s->phy[0][MII_CTRL1000] = MII_CTRL1000_FULL;
    s->phy[0][MII_STAT1000] = MII_STAT1000_LOK | MII_STAT1000_ROK |
                              MII_STAT1000_FULL;
    s->phy[0][MII_EXTSTAT] = MII_EXTSTAT_1000T_FD |
                             MII_EXTSTAT_1000T_HD;
    if (!s->link_down) {
        s->phy[0][MII_BMSR] |= MII_BMSR_LINK_ST;
    }
}

static void rtl8169_core_reset(RTL8169State *s)
{
    memcpy(s->regs + MAC0, s->conf.macaddr.a, ETH_ALEN);
    memset(s->regs + MAR0, 0xff, 8);
    memset(s->regs + CounterAddrLow, 0,
           RTL8169_REGS_SIZE - CounterAddrLow);

    /* XID 0x180 identifies the RTL8169SC/RTL8110SC revision D. */
    rtl8169_setl(s, TxConfig, 0x18000000);
    s->regs[Config1] = 0x0d; /* PMEnable, IOMAP and MEMMAP */
    s->regs[PHYstatus] = s->link_down ? 0 : PHY_STATUS_UP;
    rtl8169_setw(s, RxMaxSize, 0x2000);

    s->tx_desc = 0;
    s->tx_high_desc = 0;
    s->rx_desc = 0;
    s->tx_len = 0;
    s->tx_opts1 = 0;
    s->tx_opts2 = 0;
    rtl8169_reset_phy(s);
    rtl8169_update_irq(s);
}

static void rtl8169_reset(DeviceState *dev)
{
    RTL8169State *s = RTL8169(dev);

    s->link_down = qemu_get_queue(s->nic)->link_down;
    rtl8169_core_reset(s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->regs + MAC0);
}

static void rtl8169_phy_write(RTL8169State *s, uint32_t value)
{
    unsigned reg = (value >> 16) & 0x1f;
    uint16_t data = value;

    if (reg == 31) {
        s->phy_page = data;
    } else {
        unsigned page = s->phy_page ? 1 : 0;

        if (page == 0 && reg == MII_BMCR && (data & MII_BMCR_RESET)) {
            rtl8169_reset_phy(s);
        } else {
            s->phy[page][reg] = data;
            if (page == 0 && reg == MII_BMCR &&
                (data & MII_BMCR_ANRESTART)) {
                s->phy[0][MII_BMSR] &= ~(MII_BMSR_LINK_ST |
                                          MII_BMSR_AN_COMP);
                s->phy[0][MII_BMCR] &= ~MII_BMCR_ANRESTART;
                s->regs[PHYstatus] = 0;
                timer_mod(s->autoneg_timer,
                          qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 500);
            }
        }
    }

    /* PHYAR bit 31 clears when a write has completed. */
    rtl8169_setl(s, PHYAR, value & ~BIT(31));
}

static void rtl8169_phy_read(RTL8169State *s, uint32_t value)
{
    unsigned reg = (value >> 16) & 0x1f;
    unsigned page = s->phy_page ? 1 : 0;
    uint16_t data;

    if (reg == 31) {
        data = s->phy_page;
    } else {
        data = s->phy[page][reg];
    }
    if (page == 0 && reg == MII_BMSR) {
        if (s->link_down) {
            data &= ~(MII_BMSR_LINK_ST | MII_BMSR_AN_COMP);
        }
    }

    /* PHYAR bit 31 sets when read data is ready. */
    rtl8169_setl(s, PHYAR, BIT(31) | (reg << 16) | data);
}

static void rtl8169_dump_counters(RTL8169State *s, uint64_t address)
{
    uint8_t data[64] = { 0 };

    stq_le_p(data + 0, s->counters.tx_packets);
    stq_le_p(data + 8, s->counters.rx_packets);
    stq_le_p(data + 16, s->counters.tx_errors);
    stl_le_p(data + 24, s->counters.rx_errors);
    stw_le_p(data + 28, s->counters.rx_missed);
    stw_le_p(data + 30, s->counters.align_errors);
    stl_le_p(data + 32, s->counters.tx_one_collision);
    stl_le_p(data + 36, s->counters.tx_multi_collision);
    stq_le_p(data + 40, s->counters.rx_unicast);
    stq_le_p(data + 48, s->counters.rx_broadcast);
    stl_le_p(data + 56, s->counters.rx_multicast);
    stw_le_p(data + 60, s->counters.tx_aborted);
    stw_le_p(data + 62, s->counters.tx_underrun);
    pci_dma_write(&s->parent_obj, address, data, sizeof(data));
}

static void rtl8169_counter_command(RTL8169State *s, uint32_t value)
{
    uint64_t address = ((uint64_t)rtl8169_regl(s, CounterAddrHigh) << 32) |
                       (value & ~0x3fU);

    if (value & BIT(0)) {
        memset(&s->counters, 0, sizeof(s->counters));
    }
    if (value & BIT(3)) {
        rtl8169_dump_counters(s, address);
    }
    rtl8169_setl(s, CounterAddrLow, value & ~(BIT(0) | BIT(3)));
}

static void rtl8169_tx_pkt_release(void *context, void *base, size_t len)
{
}

static bool rtl8169_send_tx_buffer(RTL8169State *s)
{
    bool tso = s->tx_opts1 & TxLargeSend;
    bool checksum = s->tx_opts1 &
                    (TX_IP_CHECKSUM | TX_UDP_CHECKSUM | TX_TCP_CHECKSUM);
    uint16_t mss = (s->tx_opts1 >> 16) & 0x7ff;
    bool sent = false;

    if (!s->tx_len ||
        !net_tx_pkt_add_raw_fragment(s->tx_pkt, s->tx_buffer, s->tx_len)) {
        goto out;
    }
    if (!net_tx_pkt_parse(s->tx_pkt)) {
        goto out;
    }
    if (s->tx_opts2 & TX_VLAN_TAG) {
        net_tx_pkt_setup_vlan_header(s->tx_pkt, bswap16(s->tx_opts2));
    }
    if (!net_tx_pkt_build_vheader(s->tx_pkt, tso, checksum, mss)) {
        goto out;
    }
    sent = net_tx_pkt_send(s->tx_pkt, qemu_get_queue(s->nic));

out:
    net_tx_pkt_reset(s->tx_pkt, rtl8169_tx_pkt_release, NULL);
    return sent;
}

static bool rtl8169_read_desc(RTL8169State *s, uint64_t ring, uint32_t index,
                              uint8_t desc[RTL8169_DESC_SIZE])
{
    if (!ring || !rtl8169_bus_master_enabled(s)) {
        return false;
    }
    return pci_dma_read(&s->parent_obj,
                        ring + (uint64_t)index * RTL8169_DESC_SIZE,
                        desc, RTL8169_DESC_SIZE) == MEMTX_OK;
}

static void rtl8169_transmit_ring(RTL8169State *s, bool high_priority)
{
    uint64_t ring = high_priority ?
                    rtl8169_regq_pair(s, TxHDescStartAddrLow,
                                     TxHDescStartAddrHigh) :
                    rtl8169_regq_pair(s, TxDescStartAddrLow,
                                     TxDescStartAddrHigh);
    uint32_t *index = high_priority ? &s->tx_high_desc : &s->tx_desc;
    bool completed = false;
    unsigned count;

    if (!(s->regs[ChipCmd] & CmdTxEnb)) {
        return;
    }

    for (count = 0; count < RTL8169_MAX_DESCS; count++) {
        uint8_t desc[RTL8169_DESC_SIZE];
        uint64_t desc_addr = ring + (uint64_t)*index * RTL8169_DESC_SIZE;
        uint32_t opts1;
        uint32_t opts2;
        uint64_t buffer;
        uint32_t length;

        if (!rtl8169_read_desc(s, ring, *index, desc)) {
            rtl8169_raise_interrupt(s, TxErr);
            break;
        }
        opts1 = ldl_le_p(desc);
        opts2 = ldl_le_p(desc + 4);
        buffer = ldq_le_p(desc + 8);
        if (!(opts1 & DescOwn)) {
            break;
        }

        if (opts1 & FirstFrag) {
            s->tx_len = 0;
            s->tx_opts1 = opts1;
            s->tx_opts2 = opts2;
        }
        length = opts1 & DESC_LEN_MASK;
        if (length > RTL8169_TX_BUFFER_SIZE - s->tx_len ||
            pci_dma_read(&s->parent_obj, buffer,
                         s->tx_buffer + s->tx_len, length) != MEMTX_OK) {
            s->counters.tx_errors++;
            rtl8169_raise_interrupt(s, TxErr);
            s->tx_len = 0;
        } else {
            s->tx_len += length;
        }

        stl_le_p(desc, opts1 & ~DescOwn);
        pci_dma_write(&s->parent_obj, desc_addr, desc, sizeof(desc));
        *index = opts1 & RingEnd ? 0 : *index + 1;
        completed = true;

        if (opts1 & LastFrag) {
            if (rtl8169_send_tx_buffer(s)) {
                s->counters.tx_packets++;
            } else {
                s->counters.tx_errors++;
            }
            s->tx_len = 0;
        }
    }

    if (completed) {
        rtl8169_raise_interrupt(s, TxOK);
    }
}

static bool rtl8169_packet_matches(RTL8169State *s, const uint8_t *buf,
                                   size_t size, uint32_t *status)
{
    uint32_t rx_config = rtl8169_regl(s, RxConfig);
    static const uint8_t broadcast[ETH_ALEN] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    };

    if (size < ETH_ALEN) {
        return false;
    }
    if (rx_config & AcceptAllPhys) {
        return true;
    }
    if (!memcmp(buf, broadcast, ETH_ALEN)) {
        if (!(rx_config & AcceptBroadcast)) {
            return false;
        }
        s->counters.rx_broadcast++;
        *status |= BIT(25);
        return true;
    }
    if (buf[0] & 1) {
        unsigned hash = net_crc32(buf, ETH_ALEN) >> 26;

        if (!(rx_config & AcceptMulticast) ||
            !(s->regs[MAR0 + (hash >> 3)] & BIT(hash & 7))) {
            return false;
        }
        s->counters.rx_multicast++;
        *status |= BIT(27);
        return true;
    }
    if (!memcmp(buf, s->regs + MAC0, ETH_ALEN)) {
        if (!(rx_config & AcceptMyPhys)) {
            return false;
        }
        s->counters.rx_unicast++;
        *status |= BIT(26);
        return true;
    }
    return false;
}

static bool rtl8169_rx_desc_available(RTL8169State *s)
{
    uint8_t desc[RTL8169_DESC_SIZE];
    uint64_t ring = rtl8169_regq_pair(s, RxDescAddrLow, RxDescAddrHigh);

    return rtl8169_read_desc(s, ring, s->rx_desc, desc) &&
           (ldl_le_p(desc) & DescOwn);
}

static bool rtl8169_can_receive(NetClientState *nc)
{
    RTL8169State *s = qemu_get_nic_opaque(nc);

    return (s->regs[PHYstatus] & BIT(1)) &&
           (s->regs[ChipCmd] & CmdRxEnb) &&
           rtl8169_rx_desc_available(s);
}

static ssize_t rtl8169_receive(NetClientState *nc, const uint8_t *buf,
                               size_t size)
{
    RTL8169State *s = qemu_get_nic_opaque(nc);
    uint64_t ring = rtl8169_regq_pair(s, RxDescAddrLow, RxDescAddrHigh);
    uint8_t desc[RTL8169_DESC_SIZE];
    uint64_t desc_addr = ring + (uint64_t)s->rx_desc * RTL8169_DESC_SIZE;
    uint32_t old_opts1;
    uint32_t status = FirstFrag | LastFrag;
    uint64_t buffer;
    uint32_t buffer_size;
    uint32_t fcs = 0;

    if (!rtl8169_packet_matches(s, buf, size, &status)) {
        return size;
    }
    if (!rtl8169_read_desc(s, ring, s->rx_desc, desc)) {
        return 0;
    }
    old_opts1 = ldl_le_p(desc);
    buffer = ldq_le_p(desc + 8);
    buffer_size = old_opts1 & RX_LEN_MASK;
    if (!(old_opts1 & DescOwn)) {
        return 0;
    }
    if (size + sizeof(fcs) > buffer_size || size + sizeof(fcs) > RX_LEN_MASK) {
        s->counters.rx_errors++;
        s->counters.rx_missed++;
        rtl8169_setl(s, RxMissed, rtl8169_regl(s, RxMissed) + 1);
        rtl8169_raise_interrupt(s, RxOverflow | RxErr);
        return size;
    }
    if (pci_dma_write(&s->parent_obj, buffer, buf, size) != MEMTX_OK ||
        pci_dma_write(&s->parent_obj, buffer + size, &fcs,
                      sizeof(fcs)) != MEMTX_OK) {
        s->counters.rx_errors++;
        rtl8169_raise_interrupt(s, RxErr);
        return size;
    }

    status |= old_opts1 & RingEnd;
    status |= size + sizeof(fcs);
    stl_le_p(desc, status);
    stl_le_p(desc + 4, 0);
    pci_dma_write(&s->parent_obj, desc_addr, desc, sizeof(desc));
    s->rx_desc = old_opts1 & RingEnd ? 0 : s->rx_desc + 1;
    s->counters.rx_packets++;
    rtl8169_raise_interrupt(s, RxOK);
    return size;
}

static void rtl8169_set_link_status(NetClientState *nc)
{
    RTL8169State *s = qemu_get_nic_opaque(nc);

    s->link_down = nc->link_down;
    if (s->link_down) {
        s->phy[0][MII_BMSR] &= ~(MII_BMSR_LINK_ST | MII_BMSR_AN_COMP);
        s->regs[PHYstatus] = 0;
    } else {
        s->phy[0][MII_BMSR] |= MII_BMSR_LINK_ST | MII_BMSR_AN_COMP;
        s->regs[PHYstatus] = PHY_STATUS_UP;
        qemu_flush_queued_packets(nc);
    }
    rtl8169_raise_interrupt(s, LinkChg);
}

static NetClientInfo rtl8169_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = rtl8169_can_receive,
    .receive = rtl8169_receive,
    .link_status_changed = rtl8169_set_link_status,
};

static uint64_t rtl8169_read(void *opaque, hwaddr addr, unsigned size)
{
    RTL8169State *s = opaque;
    uint64_t value = 0;
    unsigned i;

    if (addr + size > RTL8169_REGS_SIZE) {
        return -1;
    }
    for (i = 0; i < size; i++) {
        value |= (uint64_t)s->regs[addr + i] << (i * 8);
    }
    return value;
}

static bool rtl8169_access_covers(hwaddr addr, unsigned size, unsigned reg)
{
    return addr <= reg && addr + size > reg;
}

static void rtl8169_store(RTL8169State *s, hwaddr addr, uint64_t value,
                          unsigned size)
{
    unsigned i;

    for (i = 0; i < size; i++) {
        s->regs[addr + i] = value >> (i * 8);
    }
}

static void rtl8169_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    RTL8169State *s = opaque;

    if (addr + size > RTL8169_REGS_SIZE) {
        return;
    }

    if (rtl8169_access_covers(addr, size, IntrStatus)) {
        unsigned shift = (IntrStatus - addr) * 8;
        uint16_t clear = value >> shift;

        if (addr == IntrStatus + 1) {
            clear = (value & 0xff) << 8;
        }
        rtl8169_setw(s, IntrStatus,
                     rtl8169_regw(s, IntrStatus) & ~clear);
        rtl8169_update_irq(s);
        return;
    }

    if (addr == ChipCmd && size == 1) {
        if (value & CmdReset) {
            rtl8169_core_reset(s);
        } else {
            s->regs[ChipCmd] = value & (CmdRxEnb | CmdTxEnb);
            if (value & CmdRxEnb) {
                qemu_flush_queued_packets(qemu_get_queue(s->nic));
            }
        }
        return;
    }

    if (addr == TxPoll && size == 1) {
        if (value & BIT(0)) {
            rtl8169_raise_interrupt(s, SWInt);
        }
        if (value & BIT(7)) {
            rtl8169_transmit_ring(s, true);
        }
        if (value & BIT(6)) {
            rtl8169_transmit_ring(s, false);
        }
        return;
    }

    if (addr == PHYAR && size == 4) {
        if (value & BIT(31)) {
            rtl8169_phy_write(s, value);
        } else {
            rtl8169_phy_read(s, value);
        }
        return;
    }

    if (addr == CounterAddrLow && size == 4) {
        rtl8169_counter_command(s, value);
        return;
    }

    if (addr == TxConfig && size == 4) {
        uint32_t version = rtl8169_regl(s, TxConfig) & 0xfc800000;

        rtl8169_setl(s, TxConfig,
                     version | ((uint32_t)value & ~0xfc800000));
        return;
    }

    rtl8169_store(s, addr, value, size);

    if (rtl8169_access_covers(addr, size, IntrMask)) {
        rtl8169_update_irq(s);
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
    if (rtl8169_access_covers(addr, size, RxConfig) ||
        rtl8169_access_covers(addr, size, RxDescAddrLow) ||
        rtl8169_access_covers(addr, size, RxDescAddrHigh)) {
        if (rtl8169_access_covers(addr, size, RxDescAddrLow)) {
            s->rx_desc = 0;
        }
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
    if (rtl8169_access_covers(addr, size, TxDescStartAddrLow)) {
        s->tx_desc = 0;
    }
    if (rtl8169_access_covers(addr, size, TxHDescStartAddrLow)) {
        s->tx_high_desc = 0;
    }
    if (rtl8169_access_covers(addr, size, MAC0) ||
        rtl8169_access_covers(addr, size, MAC0 + 4)) {
        qemu_format_nic_info_str(qemu_get_queue(s->nic), s->regs + MAC0);
    }
}

static const MemoryRegionOps rtl8169_ops = {
    .read = rtl8169_read,
    .write = rtl8169_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void rtl8169_write_config(PCIDevice *pdev, uint32_t address,
                                 uint32_t value, int len)
{
    RTL8169State *s = RTL8169(pdev);

    pci_default_write_config(pdev, address, value, len);
    if (range_covers_byte(address, len, PCI_COMMAND) &&
        rtl8169_bus_master_enabled(s)) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
}

static int rtl8169_post_load(void *opaque, int version_id)
{
    RTL8169State *s = opaque;

    qemu_get_queue(s->nic)->link_down = s->link_down;
    rtl8169_update_irq(s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->regs + MAC0);
    return 0;
}

static const VMStateDescription vmstate_rtl8169 = {
    .name = TYPE_RTL8169,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = rtl8169_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, RTL8169State),
        VMSTATE_MACADDR(conf.macaddr, RTL8169State),
        VMSTATE_UINT8_ARRAY(regs, RTL8169State, RTL8169_REGS_SIZE),
        VMSTATE_UINT16_2DARRAY(phy, RTL8169State, 2, 32),
        VMSTATE_UINT16(phy_page, RTL8169State),
        VMSTATE_UINT32(tx_desc, RTL8169State),
        VMSTATE_UINT32(tx_high_desc, RTL8169State),
        VMSTATE_UINT32(rx_desc, RTL8169State),
        VMSTATE_UINT32(tx_len, RTL8169State),
        VMSTATE_UINT32(tx_opts1, RTL8169State),
        VMSTATE_UINT32(tx_opts2, RTL8169State),
        VMSTATE_UINT8_ARRAY(tx_buffer, RTL8169State,
                            RTL8169_TX_BUFFER_SIZE),
        VMSTATE_UINT64(counters.tx_packets, RTL8169State),
        VMSTATE_UINT64(counters.rx_packets, RTL8169State),
        VMSTATE_UINT64(counters.tx_errors, RTL8169State),
        VMSTATE_UINT32(counters.rx_errors, RTL8169State),
        VMSTATE_UINT16(counters.rx_missed, RTL8169State),
        VMSTATE_UINT16(counters.align_errors, RTL8169State),
        VMSTATE_UINT32(counters.tx_one_collision, RTL8169State),
        VMSTATE_UINT32(counters.tx_multi_collision, RTL8169State),
        VMSTATE_UINT64(counters.rx_unicast, RTL8169State),
        VMSTATE_UINT64(counters.rx_broadcast, RTL8169State),
        VMSTATE_UINT32(counters.rx_multicast, RTL8169State),
        VMSTATE_UINT16(counters.tx_aborted, RTL8169State),
        VMSTATE_UINT16(counters.tx_underrun, RTL8169State),
        VMSTATE_BOOL(link_down, RTL8169State),
        VMSTATE_TIMER_PTR(autoneg_timer, RTL8169State),
        VMSTATE_END_OF_LIST()
    },
};

static void rtl8169_realize(PCIDevice *pdev, Error **errp)
{
    RTL8169State *s = RTL8169(pdev);
    DeviceState *dev = DEVICE(pdev);

    pdev->config_write = rtl8169_write_config;
    pdev->config[PCI_INTERRUPT_PIN] = 1;
    pdev->config[PCI_CACHE_LINE_SIZE] = 0x08;
    pdev->config[PCI_LATENCY_TIMER] = 0x40;
    pci_set_word(pdev->config + PCI_STATUS,
                 pci_get_word(pdev->config + PCI_STATUS) |
                 PCI_STATUS_66MHZ);

    if (pci_pm_init(pdev, 0xdc, errp) < 0) {
        return;
    }
    pci_set_word(pdev->config + 0xdc + PCI_PM_PMC, 0x7621);

    memory_region_init_io(&s->io_bar, OBJECT(s), &rtl8169_ops, s,
                          "rtl8169-io", RTL8169_REGS_SIZE);
    memory_region_init_alias(&s->mem_bar, OBJECT(s), "rtl8169-mmio",
                             &s->io_bar, 0, RTL8169_REGS_SIZE);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_IO, &s->io_bar);
    pci_register_bar(pdev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mem_bar);
    if (!pci_register_erased_rom_bar(pdev, 128 * 1024, errp)) {
        return;
    }

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&rtl8169_net_info, &s->conf,
                          object_get_typename(OBJECT(s)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    s->autoneg_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                    rtl8169_autoneg_complete, s);
    net_tx_pkt_init(&s->tx_pkt, 1);
}

static void rtl8169_uninit(PCIDevice *pdev)
{
    RTL8169State *s = RTL8169(pdev);

    net_tx_pkt_uninit(s->tx_pkt);
    timer_free(s->autoneg_timer);
    qemu_del_nic(s->nic);
}

static void rtl8169_instance_init(Object *obj)
{
    RTL8169State *s = RTL8169(obj);

    device_add_bootindex_property(obj, &s->conf.bootindex,
                                  "bootindex", "/ethernet-phy@0",
                                  DEVICE(obj));
}

static const Property rtl8169_properties[] = {
    DEFINE_NIC_PROPERTIES(RTL8169State, conf),
};

static void rtl8169_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);

    pc->realize = rtl8169_realize;
    pc->exit = rtl8169_uninit;
    pc->vendor_id = PCI_VENDOR_ID_REALTEK;
    pc->device_id = 0x8169;
    pc->revision = 0x10;
    pc->class_id = PCI_CLASS_NETWORK_ETHERNET;
    pc->subsystem_vendor_id = PCI_VENDOR_ID_REALTEK;
    pc->subsystem_id = 0x8169;
    device_class_set_legacy_reset(dc, rtl8169_reset);
    device_class_set_props(dc, rtl8169_properties);
    dc->vmsd = &vmstate_rtl8169;
    dc->desc = "Realtek RTL8169SC Gigabit Ethernet controller";
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo rtl8169_info = {
    .name = TYPE_RTL8169,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(RTL8169State),
    .instance_init = rtl8169_instance_init,
    .class_init = rtl8169_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void rtl8169_register_types(void)
{
    type_register_static(&rtl8169_info);
}

type_init(rtl8169_register_types)
