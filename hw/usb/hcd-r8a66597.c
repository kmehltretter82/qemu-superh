/*
 * Renesas R8A66597 USB host controller
 *
 * The model implements the external controller's host mode, including its
 * two root ports, ten programmable pipes, and the CFIFO/D0FIFO/D1FIFO CPU
 * interfaces.  Peripheral mode and the external DMA handshake pins are not
 * modelled.  Shared FIFO allocation, double buffering, and split-transaction
 * timing are represented functionally rather than cycle accurately.
 *
 * Reference: R8A66597FP/DFP/BG Datasheet, Rev.1.01,
 * document REJ03F0229-0101, October 17, 2008.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/usb/hcd-r8a66597.h"
#include "qemu/module.h"
#include "qemu/timer.h"

#define R8A66597_MMIO_SIZE  0x100
#define R8A66597_NUM_PORTS  2
#define R8A66597_NUM_PIPES  10
#define R8A66597_FIFO_SIZE  4096
#define R8A66597_UFRAME_NS  (NANOSECONDS_PER_SECOND / 8000)

/* Register offsets */
#define SYSCFG0             0x00
#define SYSCFG1             0x02
#define SYSSTS0             0x04
#define SYSSTS1             0x06
#define DVSTCTR0            0x08
#define DVSTCTR1            0x0a
#define TESTMODE            0x0c
#define PINCFG              0x0e
#define DMA0CFG             0x10
#define DMA1CFG             0x12
#define CFIFO               0x14
#define D0FIFO              0x18
#define D1FIFO              0x1c
#define CFIFOSEL            0x20
#define CFIFOCTR            0x22
#define CFIFOSIE            0x24
#define D0FIFOSEL           0x28
#define D0FIFOCTR           0x2a
#define D1FIFOSEL           0x2c
#define D1FIFOCTR           0x2e
#define INTENB0             0x30
#define INTENB1             0x32
#define INTENB2             0x34
#define BRDYENB             0x36
#define NRDYENB             0x38
#define BEMPENB             0x3a
#define INTSTS0             0x40
#define INTSTS1             0x42
#define INTSTS2             0x44
#define BRDYSTS             0x46
#define NRDYSTS             0x48
#define BEMPSTS             0x4a
#define FRMNUM              0x4c
#define UFRMNUM             0x4e
#define USBADDR             0x50
#define USBREQ              0x54
#define USBVAL              0x56
#define USBINDX             0x58
#define USBLENG             0x5a
#define DCPCFG              0x5c
#define DCPMAXP             0x5e
#define DCPCTR              0x60
#define PIPESEL             0x64
#define PIPECFG             0x68
#define PIPEBUF             0x6a
#define PIPEMAXP            0x6c
#define PIPEPERI            0x6e
#define PIPE1CTR            0x70
#define PIPE9CTR            0x80
#define PIPE1TRE            0x90
#define PIPE5TRN            0xa2
#define DEVADD0             0xd0
#define DEVADDA             0xe4

/* SYSCFG */
#define XCKE                0x2000
#define PLLC                0x0800
#define SCKE                0x0400
#define HSE                 0x0080
#define DCFM                0x0040
#define DRPD                0x0020
#define USBE                0x0001

/* SYSSTS */
#define LNST                0x0003
#define FS_JSTS             0x0001
#define LS_JSTS             0x0002

/* DVSTCTR */
#define USBRST              0x0040
#define UACT                0x0010
#define RHST                0x0007
#define HSMODE              0x0003
#define FSMODE              0x0002
#define LSMODE              0x0001

/* FIFO select/control */
#define RCNT                0x8000
#define REW                 0x4000
#define DCLRM               0x2000
#define DREQE               0x1000
#define MBW                 0x0c00
#define MBW_16              0x0400
#define BIGEND              0x0100
#define ISEL                0x0020
#define CURPIPE             0x000f
#define BVAL                0x8000
#define BCLR                0x4000
#define FRDY                0x2000
#define DTLN                0x0fff

/* Interrupt enable/status bits */
#define BEMPE               0x0400
#define NRDYE               0x0200
#define BRDYE               0x0100
#define BEMP                0x0400
#define NRDY                0x0200
#define BRDY                0x0100
#define SOFR                0x2000
#define BCHG                0x4000
#define DTCH                0x1000
#define ATTCH               0x0800
#define SIGN                0x0020
#define SACK                0x0010

#define DEVSEL              0xf000
#define DCP_MAXP            0x007f
#define PIPE_MAXP           0x07ff
#define SUREQ               0x4000
#define SUREQCLR            0x0800
#define SQCLR               0x0100
#define SQSET               0x0080
#define SQMON               0x0040
#define PBUSY               0x0020
#define PID                 0x0003
#define PID_NAK             0x0000
#define PID_BUF             0x0001
#define PID_STALL           0x0002

/* PIPECFG */
#define TYPE                0xc000
#define TYPE_ISO            0xc000
#define TYPE_INT            0x8000
#define TYPE_BULK           0x4000
#define BFRE                0x0400
#define DBLB                0x0200
#define CNTMD               0x0100
#define SHTNAK              0x0080
#define PIPE_DIR_OUT        0x0010
#define EPNUM               0x000f

/* PIPExCTR */
#define BSTS                0x8000
#define INBUFM              0x4000
#define ACLRM               0x0200

/* PIPExTRE */
#define TRENB               0x0200
#define TRCLR               0x0100

typedef struct R8A66597State R8A66597State;

typedef struct R8A66597Pipe {
    R8A66597State *controller;
    unsigned int index;

    uint16_t cfg;
    uint16_t buf;
    uint16_t maxp;
    uint16_t peri;
    uint16_t ctr;
    uint16_t tre;
    uint16_t trn;
    uint16_t transaction_count;
    bool transaction_complete;

    uint8_t fifo[R8A66597_FIFO_SIZE];
    unsigned int fifo_len;
    unsigned int fifo_pos;
    bool fifo_ready;
    bool tx_valid;

    USBPacket packet;
    bool packet_active;
    int packet_token;
    unsigned int packet_len;
} R8A66597Pipe;

struct R8A66597State {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    USBBus bus;
    USBPort port[R8A66597_NUM_PORTS];
    QEMUTimer *frame_timer;

    uint16_t regs[R8A66597_MMIO_SIZE / sizeof(uint16_t)];
    bool connected[R8A66597_NUM_PORTS];
    bool resetting[R8A66597_NUM_PORTS];
    R8A66597Pipe pipe[R8A66597_NUM_PIPES];

    unsigned int control_length;
    unsigned int control_done;
};

static inline uint16_t *r8a66597_reg(R8A66597State *s, hwaddr addr)
{
    return &s->regs[addr >> 1];
}

static R8A66597Pipe *r8a66597_selected_pipe(R8A66597State *s,
                                            hwaddr select)
{
    unsigned int index = *r8a66597_reg(s, select) & CURPIPE;

    if (index >= R8A66597_NUM_PIPES) {
        return NULL;
    }
    return &s->pipe[index];
}

static R8A66597Pipe *r8a66597_pipe_from_ctr(R8A66597State *s, hwaddr addr)
{
    unsigned int index;

    if (addr < PIPE1CTR || addr > PIPE9CTR || (addr & 1)) {
        return NULL;
    }
    index = 1 + (addr - PIPE1CTR) / 2;
    return &s->pipe[index];
}

static R8A66597Pipe *r8a66597_pipe_from_tre(R8A66597State *s, hwaddr addr,
                                            bool *counter)
{
    unsigned int index;

    if (addr < PIPE1TRE || addr > PIPE5TRN || (addr & 1)) {
        return NULL;
    }
    index = 1 + (addr - PIPE1TRE) / 4;
    if (index > 5) {
        return NULL;
    }
    *counter = (addr & 2) != 0;
    return &s->pipe[index];
}

static hwaddr r8a66597_fifo_select(hwaddr fifo)
{
    switch (fifo) {
    case CFIFO:
        return CFIFOSEL;
    case D0FIFO:
        return D0FIFOSEL;
    case D1FIFO:
        return D1FIFOSEL;
    default:
        g_assert_not_reached();
    }
}

static hwaddr r8a66597_fifo_from_control(hwaddr control)
{
    switch (control) {
    case CFIFOCTR:
        return CFIFO;
    case D0FIFOCTR:
        return D0FIFO;
    case D1FIFOCTR:
        return D1FIFO;
    default:
        g_assert_not_reached();
    }
}

static unsigned int r8a66597_pipe_maxp(R8A66597State *s,
                                       R8A66597Pipe *pipe)
{
    unsigned int maxp;

    if (pipe->index == 0) {
        maxp = *r8a66597_reg(s, DCPMAXP) & DCP_MAXP;
    } else {
        maxp = pipe->maxp & PIPE_MAXP;
    }
    return maxp;
}

static bool r8a66597_pipe_out(R8A66597State *s, R8A66597Pipe *pipe,
                              hwaddr select)
{
    if (pipe->index == 0) {
        return (*r8a66597_reg(s, select) & ISEL) != 0;
    }
    return (pipe->cfg & PIPE_DIR_OUT) != 0;
}

static int r8a66597_port_speed(R8A66597State *s, unsigned int port)
{
    USBDevice *dev = s->port[port].dev;

    return dev && dev->attached ? dev->speed : USB_SPEED_FULL;
}

static uint16_t r8a66597_line_state(R8A66597State *s, unsigned int port)
{
    if (!s->connected[port]) {
        return 0;
    }
    return r8a66597_port_speed(s, port) == USB_SPEED_LOW ?
           LS_JSTS : FS_JSTS;
}

static uint16_t r8a66597_reset_state(R8A66597State *s, unsigned int port)
{
    if (!s->connected[port] || s->resetting[port]) {
        return 0;
    }

    switch (r8a66597_port_speed(s, port)) {
    case USB_SPEED_LOW:
        return LSMODE;
    case USB_SPEED_HIGH:
        return HSMODE;
    default:
        return FSMODE;
    }
}

static void r8a66597_update_irq(R8A66597State *s)
{
    uint16_t intsts0 = *r8a66597_reg(s, INTSTS0);
    bool level;

    intsts0 &= ~(BEMP | NRDY | BRDY);
    if (*r8a66597_reg(s, BEMPSTS) & *r8a66597_reg(s, BEMPENB)) {
        intsts0 |= BEMP;
    }
    if (*r8a66597_reg(s, NRDYSTS) & *r8a66597_reg(s, NRDYENB)) {
        intsts0 |= NRDY;
    }
    if (*r8a66597_reg(s, BRDYSTS) & *r8a66597_reg(s, BRDYENB)) {
        intsts0 |= BRDY;
    }
    *r8a66597_reg(s, INTSTS0) = intsts0;

    level = ((intsts0 & *r8a66597_reg(s, INTENB0)) != 0) ||
            ((*r8a66597_reg(s, INTSTS1) &
              *r8a66597_reg(s, INTENB1)) != 0) ||
            ((*r8a66597_reg(s, INTSTS2) &
              *r8a66597_reg(s, INTENB2)) != 0);
    qemu_set_irq(s->irq, level);
}

static USBDevice *r8a66597_find_device(R8A66597State *s, uint8_t addr)
{
    unsigned int i;

    for (i = 0; i < R8A66597_NUM_PORTS; i++) {
        USBDevice *dev = usb_find_device(&s->port[i], addr);

        if (dev) {
            return dev;
        }
    }
    return NULL;
}

static uint16_t *r8a66597_pipe_ctr(R8A66597State *s,
                                   R8A66597Pipe *pipe)
{
    if (pipe->index == 0) {
        return r8a66597_reg(s, DCPCTR);
    }
    return &pipe->ctr;
}

static void r8a66597_service_pipe(R8A66597State *s, R8A66597Pipe *pipe,
                                  bool periodic_tick);

static void r8a66597_cancel_pipe(R8A66597Pipe *pipe)
{
    uint16_t *ctr;

    if (!pipe->packet_active) {
        return;
    }

    usb_cancel_packet(&pipe->packet);
    usb_packet_cleanup(&pipe->packet);
    pipe->packet_active = false;
    ctr = r8a66597_pipe_ctr(pipe->controller, pipe);
    *ctr &= ~(PBUSY | INBUFM);
}

static void r8a66597_count_transaction(R8A66597State *s,
                                       R8A66597Pipe *pipe,
                                       unsigned int actual)
{
    if (!(pipe->tre & TRENB) || pipe->index > 5) {
        return;
    }

    pipe->transaction_count++;
    if (actual < r8a66597_pipe_maxp(s, pipe) ||
        pipe->transaction_count >= pipe->trn) {
        pipe->transaction_count = 0;
        pipe->transaction_complete = true;
        if (pipe->cfg & SHTNAK) {
            pipe->ctr = (pipe->ctr & ~PID) | PID_NAK;
        }
    }
}

static void r8a66597_packet_done(R8A66597State *s, R8A66597Pipe *pipe,
                                 int status, unsigned int actual)
{
    uint16_t *ctr = r8a66597_pipe_ctr(s, pipe);
    unsigned int bit = 1 << pipe->index;

    *ctr &= ~(PBUSY | INBUFM);

    if (status == USB_RET_SUCCESS) {
        *ctr ^= SQMON;
        switch (pipe->packet_token) {
        case USB_TOKEN_SETUP:
            pipe->fifo_len = 0;
            pipe->fifo_pos = 0;
            pipe->fifo_ready = false;
            pipe->tx_valid = false;
            *r8a66597_reg(s, INTSTS1) |= SACK;
            break;
        case USB_TOKEN_IN:
            pipe->fifo_len = actual;
            pipe->fifo_pos = 0;
            pipe->fifo_ready = true;
            pipe->tx_valid = false;
            if (pipe->index == 0 && pipe->packet_len) {
                s->control_done += actual;
            }
            if (pipe->index && actual < pipe->packet_len &&
                (pipe->cfg & SHTNAK)) {
                *ctr = (*ctr & ~PID) | PID_NAK;
            }
            r8a66597_count_transaction(s, pipe, actual);
            *r8a66597_reg(s, BRDYSTS) |= bit;
            break;
        case USB_TOKEN_OUT:
            if (pipe->index == 0 && pipe->packet_len) {
                s->control_done += pipe->packet_len;
            }
            pipe->fifo_len = 0;
            pipe->fifo_pos = 0;
            pipe->fifo_ready = false;
            pipe->tx_valid = false;
            *r8a66597_reg(s, BEMPSTS) |= bit;
            if (pipe->index) {
                *r8a66597_reg(s, BRDYSTS) |= bit;
            }
            break;
        default:
            g_assert_not_reached();
        }
    } else if (status != USB_RET_NAK) {
        if (pipe->packet_token == USB_TOKEN_SETUP) {
            *r8a66597_reg(s, INTSTS1) |= SIGN;
        } else {
            if (status == USB_RET_STALL) {
                *ctr = (*ctr & ~PID) | PID_STALL;
            }
            *r8a66597_reg(s, NRDYSTS) |= bit;
        }
    }

    r8a66597_update_irq(s);
}

static void r8a66597_submit_packet(R8A66597State *s,
                                   R8A66597Pipe *pipe, int token,
                                   unsigned int len)
{
    USBDevice *dev;
    USBEndpoint *ep;
    unsigned int epnum;
    uint8_t addr;

    if (pipe->packet_active || len > sizeof(pipe->fifo)) {
        return;
    }

    if (pipe->index == 0) {
        addr = (*r8a66597_reg(s, DCPMAXP) & DEVSEL) >> 12;
        epnum = 0;
    } else {
        addr = (pipe->maxp & DEVSEL) >> 12;
        epnum = pipe->cfg & EPNUM;
    }

    pipe->packet_token = token;
    pipe->packet_len = len;
    dev = r8a66597_find_device(s, addr);
    if (!dev) {
        r8a66597_packet_done(s, pipe, USB_RET_NODEV, 0);
        return;
    }

    ep = usb_ep_get(dev, token, epnum);
    usb_packet_init(&pipe->packet);
    usb_packet_setup(&pipe->packet, token, ep, 0, pipe->index,
                     false, true);
    if (len) {
        usb_packet_addbuf(&pipe->packet, pipe->fifo, len);
    }

    *r8a66597_pipe_ctr(s, pipe) |= PBUSY;
    if (token == USB_TOKEN_OUT && pipe->index) {
        pipe->ctr |= INBUFM;
    }
    usb_handle_packet(dev, &pipe->packet);
    if (pipe->packet.status == USB_RET_ASYNC) {
        pipe->packet_active = true;
        return;
    }

    r8a66597_packet_done(s, pipe, pipe->packet.status,
                         pipe->packet.actual_length);
    usb_packet_cleanup(&pipe->packet);
}

static void r8a66597_setup(R8A66597State *s)
{
    R8A66597Pipe *pipe = &s->pipe[0];
    uint16_t setup[] = {
        *r8a66597_reg(s, USBREQ),
        *r8a66597_reg(s, USBVAL),
        *r8a66597_reg(s, USBINDX),
        *r8a66597_reg(s, USBLENG),
    };

    r8a66597_cancel_pipe(pipe);
    stw_le_p(pipe->fifo, setup[0]);
    stw_le_p(pipe->fifo + 2, setup[1]);
    stw_le_p(pipe->fifo + 4, setup[2]);
    stw_le_p(pipe->fifo + 6, setup[3]);
    pipe->fifo_len = sizeof(setup);
    pipe->fifo_pos = 0;
    pipe->fifo_ready = false;
    pipe->tx_valid = false;
    s->control_length = setup[3];
    s->control_done = 0;
    r8a66597_submit_packet(s, pipe, USB_TOKEN_SETUP, sizeof(setup));
}

static void r8a66597_service_pipe(R8A66597State *s, R8A66597Pipe *pipe,
                                  bool periodic_tick)
{
    uint16_t *ctr = r8a66597_pipe_ctr(s, pipe);
    unsigned int length;
    bool request_in;
    bool out;

    if ((*ctr & PID) != PID_BUF || pipe->packet_active) {
        return;
    }
    if ((pipe->tre & TRENB) && pipe->transaction_complete) {
        return;
    }
    if (pipe->index && (pipe->cfg & TYPE) >= TYPE_INT && !periodic_tick) {
        return;
    }

    if (pipe->index == 0) {
        out = r8a66597_pipe_out(s, pipe, CFIFOSEL);
        request_in = (*r8a66597_reg(s, USBREQ) & USB_DIR_IN) != 0;

        if (out) {
            if (!pipe->tx_valid) {
                return;
            }
            r8a66597_submit_packet(s, pipe, USB_TOKEN_OUT,
                                   pipe->fifo_len);
            return;
        }

        if (pipe->fifo_ready) {
            return;
        }
        if (request_in && s->control_done < s->control_length) {
            length = MIN(r8a66597_pipe_maxp(s, pipe),
                         s->control_length - s->control_done);
        } else if (!request_in && s->control_done < s->control_length) {
            return;
        } else {
            length = 0;
        }
        r8a66597_submit_packet(s, pipe, USB_TOKEN_IN, length);
        return;
    }

    if (!(pipe->cfg & TYPE)) {
        return;
    }
    length = r8a66597_pipe_maxp(s, pipe);
    if (!length) {
        return;
    }
    if (pipe->cfg & PIPE_DIR_OUT) {
        if (!pipe->tx_valid) {
            return;
        }
        r8a66597_submit_packet(s, pipe, USB_TOKEN_OUT, pipe->fifo_len);
    } else if (!pipe->fifo_ready) {
        r8a66597_submit_packet(s, pipe, USB_TOKEN_IN, length);
    }
}

static uint16_t r8a66597_fifo_control_read(R8A66597State *s, hwaddr control)
{
    hwaddr fifo = r8a66597_fifo_from_control(control);
    hwaddr select = r8a66597_fifo_select(fifo);
    R8A66597Pipe *pipe = r8a66597_selected_pipe(s, select);
    bool out;

    if (!pipe) {
        return 0;
    }
    out = r8a66597_pipe_out(s, pipe, select);
    if (out) {
        return (!pipe->packet_active && !pipe->tx_valid) ? FRDY : 0;
    }
    if (!pipe->fifo_ready) {
        return 0;
    }
    if (*r8a66597_reg(s, select) & RCNT) {
        return FRDY | ((pipe->fifo_len - pipe->fifo_pos) & DTLN);
    }
    return FRDY | (pipe->fifo_len & DTLN);
}

static uint16_t r8a66597_fifo_read(R8A66597State *s, hwaddr fifo,
                                   unsigned int size)
{
    hwaddr select = r8a66597_fifo_select(fifo);
    R8A66597Pipe *pipe = r8a66597_selected_pipe(s, select);
    uint16_t value = 0;
    bool big_endian;

    if (!pipe || !pipe->fifo_ready || !pipe->fifo_len) {
        return 0;
    }

    big_endian = (*r8a66597_reg(s, select) & BIGEND) != 0;
    value = pipe->fifo[pipe->fifo_pos++];
    if (size == 2) {
        uint8_t second;

        if (pipe->fifo_pos < pipe->fifo_len) {
            second = pipe->fifo[pipe->fifo_pos++];
        } else {
            /*
             * The external controller still completes a 16-bit bus read
             * after an odd byte count.  Hardware observations on SH7785LCR
             * show the first FIFO byte on the unused lane.
             */
            second = pipe->fifo[0];
        }
        value = big_endian ? (value << 8) | second
                           : value | (second << 8);
    }

    if (pipe->fifo_pos >= pipe->fifo_len) {
        pipe->fifo_len = 0;
        pipe->fifo_pos = 0;
        pipe->fifo_ready = false;
        r8a66597_service_pipe(s, pipe, false);
    }
    return value;
}

static void r8a66597_fifo_write(R8A66597State *s, hwaddr fifo,
                                uint16_t value, unsigned int size)
{
    hwaddr select = r8a66597_fifo_select(fifo);
    R8A66597Pipe *pipe = r8a66597_selected_pipe(s, select);
    unsigned int maxp;
    bool big_endian;
    bool wide;

    if (!pipe || !r8a66597_pipe_out(s, pipe, select) ||
        pipe->packet_active || pipe->tx_valid) {
        return;
    }

    maxp = r8a66597_pipe_maxp(s, pipe);
    if (!maxp) {
        return;
    }
    big_endian = (*r8a66597_reg(s, select) & BIGEND) != 0;
    wide = (*r8a66597_reg(s, select) & MBW) == MBW_16;
    if (size == 1 || !wide) {
        if (pipe->fifo_len < sizeof(pipe->fifo)) {
            pipe->fifo[pipe->fifo_len++] = value;
        }
    } else if (pipe->fifo_len + 2 <= sizeof(pipe->fifo)) {
        pipe->fifo[pipe->fifo_len++] = big_endian ? value >> 8 : value;
        pipe->fifo[pipe->fifo_len++] = big_endian ? value : value >> 8;
    }

    if (pipe->fifo_len >= maxp) {
        pipe->tx_valid = true;
        r8a66597_service_pipe(s, pipe, false);
    }
}

static void r8a66597_fifo_control_write(R8A66597State *s, hwaddr control,
                                        uint16_t value)
{
    hwaddr fifo = r8a66597_fifo_from_control(control);
    hwaddr select = r8a66597_fifo_select(fifo);
    R8A66597Pipe *pipe = r8a66597_selected_pipe(s, select);

    if (!pipe) {
        return;
    }
    if (value & BCLR) {
        pipe->fifo_len = 0;
        pipe->fifo_pos = 0;
        pipe->fifo_ready = false;
        pipe->tx_valid = false;
    }
    if ((value & BVAL) && r8a66597_pipe_out(s, pipe, select)) {
        pipe->tx_valid = true;
        r8a66597_service_pipe(s, pipe, false);
    }
}

static uint16_t r8a66597_read16(R8A66597State *s, hwaddr addr)
{
    R8A66597Pipe *pipe;
    bool counter;

    switch (addr) {
    case SYSSTS0:
        return (*r8a66597_reg(s, addr) & ~LNST) |
               r8a66597_line_state(s, 0);
    case SYSSTS1:
        return (*r8a66597_reg(s, addr) & ~LNST) |
               r8a66597_line_state(s, 1);
    case DVSTCTR0:
        return (*r8a66597_reg(s, addr) & ~RHST) |
               r8a66597_reset_state(s, 0);
    case DVSTCTR1:
        return (*r8a66597_reg(s, addr) & ~RHST) |
               r8a66597_reset_state(s, 1);
    case CFIFO:
    case D0FIFO:
    case D1FIFO:
        return r8a66597_fifo_read(s, addr, 2);
    case CFIFOCTR:
    case D0FIFOCTR:
    case D1FIFOCTR:
        return r8a66597_fifo_control_read(s, addr);
    case PIPECFG:
    case PIPEBUF:
    case PIPEMAXP:
    case PIPEPERI:
        pipe = r8a66597_selected_pipe(s, PIPESEL);
        if (!pipe || pipe->index == 0) {
            return 0;
        }
        if (addr == PIPECFG) {
            return pipe->cfg;
        }
        if (addr == PIPEBUF) {
            return pipe->buf;
        }
        if (addr == PIPEMAXP) {
            return pipe->maxp;
        }
        return pipe->peri;
    default:
        pipe = r8a66597_pipe_from_ctr(s, addr);
        if (pipe) {
            return pipe->ctr;
        }
        pipe = r8a66597_pipe_from_tre(s, addr, &counter);
        if (pipe) {
            if (!counter) {
                return pipe->tre;
            }
            return (pipe->tre & TRENB) ? pipe->transaction_count : pipe->trn;
        }
        return *r8a66597_reg(s, addr);
    }
}

static uint64_t r8a66597_read(void *opaque, hwaddr addr, unsigned int size)
{
    R8A66597State *s = opaque;
    uint16_t value;

    if ((addr == CFIFO || addr == D0FIFO || addr == D1FIFO) && size == 1) {
        return r8a66597_fifo_read(s, addr, 1);
    }
    if (size == 2) {
        return r8a66597_read16(s, addr);
    }
    value = r8a66597_read16(s, addr & ~1);
    return (value >> ((addr & 1) * 8)) & 0xff;
}

static void r8a66597_attach(USBPort *port)
{
    R8A66597State *s = port->opaque;
    hwaddr intsts = port->index ? INTSTS2 : INTSTS1;

    if (!port->dev || !port->dev->attached) {
        return;
    }
    s->connected[port->index] = true;
    *r8a66597_reg(s, intsts) |= ATTCH;
    r8a66597_update_irq(s);
}

static void r8a66597_detach(USBPort *port)
{
    R8A66597State *s = port->opaque;
    hwaddr intsts = port->index ? INTSTS2 : INTSTS1;
    unsigned int i;

    s->connected[port->index] = false;
    s->resetting[port->index] = false;
    for (i = 0; i < R8A66597_NUM_PIPES; i++) {
        r8a66597_cancel_pipe(&s->pipe[i]);
    }
    *r8a66597_reg(s, intsts) |= DTCH;
    r8a66597_update_irq(s);
}

static void r8a66597_child_detach(USBPort *port, USBDevice *child)
{
}

static void r8a66597_wakeup(USBPort *port)
{
    R8A66597State *s = port->opaque;
    hwaddr intsts = port->index ? INTSTS2 : INTSTS1;

    *r8a66597_reg(s, intsts) |= BCHG;
    r8a66597_update_irq(s);
}

static void r8a66597_async_packet_complete(USBPort *port, USBPacket *packet)
{
    R8A66597State *s = port->opaque;
    R8A66597Pipe *pipe = NULL;
    unsigned int i;

    for (i = 0; i < R8A66597_NUM_PIPES; i++) {
        if (&s->pipe[i].packet == packet) {
            pipe = &s->pipe[i];
            break;
        }
    }
    g_assert(pipe && pipe->packet_active);

    if (packet->status == USB_RET_REMOVE_FROM_QUEUE) {
        r8a66597_cancel_pipe(pipe);
        return;
    }

    pipe->packet_active = false;
    r8a66597_packet_done(s, pipe, packet->status, packet->actual_length);
    usb_packet_cleanup(packet);
}

static void r8a66597_frame(void *opaque)
{
    R8A66597State *s = opaque;
    uint16_t frame = *r8a66597_reg(s, FRMNUM);
    uint16_t microframe = *r8a66597_reg(s, UFRMNUM);
    bool active;
    unsigned int i;

    active = (*r8a66597_reg(s, DVSTCTR0) & UACT) ||
             (*r8a66597_reg(s, DVSTCTR1) & UACT);
    if (active) {
        microframe = (microframe + 1) & 7;
        *r8a66597_reg(s, UFRMNUM) = microframe;
        if (!microframe) {
            frame = (frame + 1) & 0x07ff;
            *r8a66597_reg(s, FRMNUM) = frame;
            *r8a66597_reg(s, INTSTS0) |= SOFR;
        }

        for (i = 0; i < R8A66597_NUM_PIPES; i++) {
            R8A66597Pipe *pipe = &s->pipe[i];
            unsigned int interval;
            unsigned int tick;
            USBDevice *dev;
            uint8_t addr;

            if (i && (pipe->cfg & TYPE) >= TYPE_INT) {
                interval = 1 << (pipe->peri & 7);
                addr = (pipe->maxp & DEVSEL) >> 12;
                dev = r8a66597_find_device(s, addr);
                if (dev && dev->speed == USB_SPEED_HIGH) {
                    tick = frame * 8 + microframe;
                } else {
                    if (microframe) {
                        continue;
                    }
                    tick = frame;
                }
                if (tick & (interval - 1)) {
                    continue;
                }
            }
            r8a66597_service_pipe(s, pipe, true);
        }
        r8a66597_update_irq(s);
    }
    timer_mod(s->frame_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + R8A66597_UFRAME_NS);
}

static USBPortOps r8a66597_port_ops = {
    .attach = r8a66597_attach,
    .detach = r8a66597_detach,
    .child_detach = r8a66597_child_detach,
    .wakeup = r8a66597_wakeup,
    .complete = r8a66597_async_packet_complete,
};

static USBBusOps r8a66597_bus_ops = {
};

static void r8a66597_write_pipe_ctr(R8A66597State *s,
                                    R8A66597Pipe *pipe, uint16_t value)
{
    uint16_t *ctr = r8a66597_pipe_ctr(s, pipe);
    uint16_t read_only = *ctr & (SQMON | PBUSY | INBUFM);

    if (value & SQCLR) {
        read_only &= ~SQMON;
    } else if (value & SQSET) {
        read_only |= SQMON;
    }
    if (value & ACLRM) {
        pipe->fifo_len = 0;
        pipe->fifo_pos = 0;
        pipe->fifo_ready = false;
        pipe->tx_valid = false;
    }

    *ctr = read_only |
           (value & ~(BSTS | INBUFM | SUREQCLR | SQCLR | SQSET |
                      SQMON | PBUSY));
    if ((*ctr & PID) != PID_BUF) {
        r8a66597_cancel_pipe(pipe);
    } else {
        r8a66597_service_pipe(s, pipe, false);
    }
}

static void r8a66597_write16(R8A66597State *s, hwaddr addr, uint16_t value)
{
    uint16_t old = *r8a66597_reg(s, addr);
    R8A66597Pipe *pipe;
    bool counter;
    unsigned int port;

    switch (addr) {
    case SYSCFG0:
    case SYSCFG1:
        *r8a66597_reg(s, addr) = value;
        if ((value & XCKE) && !(old & XCKE)) {
            *r8a66597_reg(s, addr) |= SCKE;
        }
        break;
    case DVSTCTR0:
    case DVSTCTR1:
        port = addr == DVSTCTR0 ? 0 : 1;
        *r8a66597_reg(s, addr) = value & ~RHST;
        if (value & USBRST) {
            s->resetting[port] = true;
            if (s->port[port].dev) {
                usb_device_reset(s->port[port].dev);
            }
        } else if ((value & UACT) && s->resetting[port]) {
            s->resetting[port] = false;
        }
        break;
    case INTSTS0:
    case INTSTS1:
    case INTSTS2:
    case BRDYSTS:
    case NRDYSTS:
    case BEMPSTS:
        /* Interrupt status bits are cleared by writing zero. */
        *r8a66597_reg(s, addr) &= value;
        r8a66597_update_irq(s);
        break;
    case INTENB0:
    case BRDYENB:
    case NRDYENB:
    case BEMPENB:
        *r8a66597_reg(s, addr) = value;
        r8a66597_update_irq(s);
        break;
    case INTENB1:
    case INTENB2:
        port = addr == INTENB1 ? 0 : 1;
        *r8a66597_reg(s, addr) = value;
        if (s->connected[port] && (value & ATTCH)) {
            *r8a66597_reg(s, port ? INTSTS2 : INTSTS1) |= ATTCH;
        }
        r8a66597_update_irq(s);
        break;
    case CFIFOSEL:
    case D0FIFOSEL:
    case D1FIFOSEL:
        *r8a66597_reg(s, addr) = value &
            (RCNT | REW | DCLRM | DREQE | MBW | BIGEND | ISEL | CURPIPE);
        pipe = r8a66597_selected_pipe(s, addr);
        if (pipe && (value & REW)) {
            pipe->fifo_pos = 0;
            *r8a66597_reg(s, addr) &= ~REW;
        }
        break;
    case CFIFOCTR:
    case D0FIFOCTR:
    case D1FIFOCTR:
        r8a66597_fifo_control_write(s, addr, value);
        r8a66597_update_irq(s);
        break;
    case DCPCFG:
        *r8a66597_reg(s, addr) = value &
            (CNTMD | SHTNAK | PIPE_DIR_OUT);
        break;
    case DCPMAXP:
        *r8a66597_reg(s, addr) = value & (DEVSEL | DCP_MAXP);
        break;
    case DCPCTR:
        if (value & SUREQCLR) {
            r8a66597_cancel_pipe(&s->pipe[0]);
        }
        r8a66597_write_pipe_ctr(s, &s->pipe[0], value);
        if (value & SUREQ) {
            r8a66597_setup(s);
        }
        break;
    case CFIFO:
    case D0FIFO:
    case D1FIFO:
        r8a66597_fifo_write(s, addr, value, 2);
        break;
    case PIPESEL:
        *r8a66597_reg(s, addr) = value & CURPIPE;
        break;
    case PIPECFG:
    case PIPEBUF:
    case PIPEMAXP:
    case PIPEPERI:
        pipe = r8a66597_selected_pipe(s, PIPESEL);
        if (!pipe || pipe->index == 0) {
            break;
        }
        if (addr == PIPECFG) {
            pipe->cfg = value &
                (TYPE | BFRE | DBLB | CNTMD | SHTNAK |
                 PIPE_DIR_OUT | EPNUM);
        } else if (addr == PIPEBUF) {
            if (pipe->index < 6) {
                pipe->buf = value & 0x7cff;
            }
        } else if (addr == PIPEMAXP) {
            pipe->maxp = value & (DEVSEL | PIPE_MAXP);
        } else {
            pipe->peri = value & 0x1007;
        }
        break;
    default:
        pipe = r8a66597_pipe_from_ctr(s, addr);
        if (pipe) {
            r8a66597_write_pipe_ctr(s, pipe, value);
            break;
        }
        pipe = r8a66597_pipe_from_tre(s, addr, &counter);
        if (pipe) {
            if (counter) {
                pipe->trn = value;
                pipe->transaction_count = 0;
                pipe->transaction_complete = false;
            } else {
                if (value & TRCLR) {
                    pipe->transaction_count = 0;
                    pipe->transaction_complete = false;
                }
                pipe->tre = value & TRENB;
            }
            break;
        }
        *r8a66597_reg(s, addr) = value;
        break;
    }
}

static void r8a66597_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned int size)
{
    R8A66597State *s = opaque;
    hwaddr word_addr = addr & ~1;
    uint16_t word;

    if (word_addr == CFIFO || word_addr == D0FIFO || word_addr == D1FIFO) {
        if (addr == word_addr) {
            r8a66597_fifo_write(s, word_addr, value, size);
        }
        return;
    }
    if (size == 2) {
        r8a66597_write16(s, addr, value);
        return;
    }

    word = *r8a66597_reg(s, word_addr);
    if (addr & 1) {
        word = (word & 0x00ff) | ((value & 0xff) << 8);
    } else {
        word = (word & 0xff00) | (value & 0xff);
    }
    r8a66597_write16(s, word_addr, word);
}

static const MemoryRegionOps r8a66597_mmio_ops = {
    .read = r8a66597_read,
    .write = r8a66597_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 2,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
};

static void r8a66597_reset(DeviceState *dev)
{
    R8A66597State *s = R8A66597_USB_HOST(dev);
    unsigned int i;

    for (i = 0; i < R8A66597_NUM_PIPES; i++) {
        R8A66597Pipe *pipe = &s->pipe[i];

        r8a66597_cancel_pipe(pipe);
        pipe->cfg = 0;
        pipe->buf = i >= 6 ? i - 2 : 0;
        pipe->maxp = i ? 0x40 : 0;
        pipe->peri = 0;
        pipe->ctr = 0;
        pipe->tre = 0;
        pipe->trn = 0;
        pipe->transaction_count = 0;
        pipe->transaction_complete = false;
        pipe->fifo_len = 0;
        pipe->fifo_pos = 0;
        pipe->fifo_ready = false;
        pipe->tx_valid = false;
    }
    memset(s->regs, 0, sizeof(s->regs));
    *r8a66597_reg(s, DCPMAXP) = 0x40;
    s->control_length = 0;
    s->control_done = 0;
    for (i = 0; i < R8A66597_NUM_PORTS; i++) {
        s->connected[i] = s->port[i].dev && s->port[i].dev->attached;
        s->resetting[i] = false;
        if (s->connected[i]) {
            usb_device_reset(s->port[i].dev);
        }
    }
    r8a66597_update_irq(s);
    timer_mod(s->frame_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + R8A66597_UFRAME_NS);
}

static void r8a66597_realize(DeviceState *dev, Error **errp)
{
    R8A66597State *s = R8A66597_USB_HOST(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    unsigned int i;

    usb_bus_new(&s->bus, sizeof(s->bus), &r8a66597_bus_ops, dev);
    for (i = 0; i < R8A66597_NUM_PORTS; i++) {
        usb_register_port(&s->bus, &s->port[i], s, i, &r8a66597_port_ops,
                          USB_SPEED_MASK_LOW | USB_SPEED_MASK_FULL |
                          USB_SPEED_MASK_HIGH);
    }
    for (i = 0; i < R8A66597_NUM_PIPES; i++) {
        s->pipe[i].controller = s;
        s->pipe[i].index = i;
    }
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, r8a66597_frame, s);
    sysbus_init_irq(sbd, &s->irq);
}

static void r8a66597_unrealize(DeviceState *dev)
{
    R8A66597State *s = R8A66597_USB_HOST(dev);
    unsigned int i;

    timer_del(s->frame_timer);
    for (i = 0; i < R8A66597_NUM_PIPES; i++) {
        r8a66597_cancel_pipe(&s->pipe[i]);
    }
    timer_free(s->frame_timer);
    s->frame_timer = NULL;
}

static void r8a66597_init(Object *obj)
{
    R8A66597State *s = R8A66597_USB_HOST(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &r8a66597_mmio_ops, s,
                          "r8a66597", R8A66597_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->mmio);
}

USBBus *r8a66597_usb_bus(R8A66597State *s)
{
    return &s->bus;
}

static void r8a66597_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = r8a66597_realize;
    dc->unrealize = r8a66597_unrealize;
    device_class_set_legacy_reset(dc, r8a66597_reset);
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static const TypeInfo r8a66597_type_info = {
    .name = TYPE_R8A66597_USB_HOST,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(R8A66597State),
    .instance_init = r8a66597_init,
    .class_init = r8a66597_class_init,
};

static void r8a66597_register_types(void)
{
    type_register_static(&r8a66597_type_info);
}

type_init(r8a66597_register_types)
