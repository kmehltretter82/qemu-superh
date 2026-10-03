/*
 * QTest test cases for the Renesas SH7785LCR board
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "libqtest.h"

#define FLASH_BASE              0x00000000
#define FLASH_BANK_WIDTH        4

#define PLD_BASE                0x04000000
#define PLD_PCICR               0x00
#define PLD_SWSR                0x0a
#define PLD_VERSR               0x0c

#define SM501_MMIO_BASE         0x13e00000
#define SM501_DEVICEID          0x60

#define CG200_32BIT_BASE        0x0c000000
#define CG200_29BIT_BASE        0x18000000
#define CG200_SLOT_STRIDE       0x400
#define SDHCI_CAPABILITIES      0x40
#define SDHCI_HOST_VERSION      0xfe
#define CG200_CAPABILITIES      0x01603232
#define SDHCI_SPEC_100          0x2400

#define R8A66597_29BIT_BASE     0x14000000
#define DMA0CFG                 0x10
#define D0FIFO                  0x18
#define D0FIFOSEL               0x28
#define D0FIFOCTR               0x2a
#define PIPESEL                 0x64
#define PIPECFG                 0x68
#define PIPEMAXP                0x6c
#define PIPE1CTR                0x70

#define DMA_BURST               0x2000
#define DMA_DREQA               0x4000
#define DMA_DFORM_DACK          0x0100
#define DMA_DENDE               0x0010
#define DREQE                   0x1000
#define MBW_16                  0x0400
#define FRDY                    0x2000
#define PID_BUF                 0x0001
#define TYPE_BULK               0x4000
#define PIPE_DIR_OUT            0x0010

static QTestState *sh7785lcr_start(const char *machine_args)
{
    return qtest_initf("-M sh7785lcr,usb-device=none%s "
                       "-display none -nic none", machine_args);
}

static void test_board_devices(void)
{
    QTestState *qts = sh7785lcr_start(",dipsw=10");

    qtest_writel(qts, FLASH_BASE + 0x555 * FLASH_BANK_WIDTH, 0x00aa00aa);
    qtest_writel(qts, FLASH_BASE + 0x2aa * FLASH_BANK_WIDTH, 0x00550055);
    qtest_writel(qts, FLASH_BASE + 0x555 * FLASH_BANK_WIDTH, 0x00900090);
    g_assert_cmphex(qtest_readl(qts, FLASH_BASE), ==, 0x00010001);
    g_assert_cmphex(qtest_readl(qts, FLASH_BASE + FLASH_BANK_WIDTH),
                    ==, 0x22012201);
    qtest_writel(qts, FLASH_BASE, 0x00f000f0);

    qtest_writel(qts, FLASH_BASE + 0x55 * FLASH_BANK_WIDTH, 0x00980098);
    g_assert_cmphex(qtest_readl(qts, FLASH_BASE + 0x27 * FLASH_BANK_WIDTH),
                    ==, 0x00190019);
    qtest_writel(qts, FLASH_BASE, 0x00f000f0);

    g_assert_cmphex(qtest_readw(qts, PLD_BASE + PLD_SWSR), ==, 0x000a);
    g_assert_cmphex(qtest_readw(qts, PLD_BASE + PLD_VERSR), ==, 0x0001);
    qtest_writew(qts, PLD_BASE + PLD_PCICR, 0x5aa5);
    g_assert_cmphex(qtest_readw(qts, PLD_BASE + PLD_PCICR), ==, 0x5aa5);

    g_assert_cmphex(qtest_readl(qts, SM501_MMIO_BASE + SM501_DEVICEID),
                    ==, 0x050100c0);

    for (unsigned int slot = 0; slot < 2; slot++) {
        uint64_t base = CG200_29BIT_BASE + slot * CG200_SLOT_STRIDE;

        g_assert_cmphex(qtest_readl(qts, base + SDHCI_CAPABILITIES),
                        ==, CG200_CAPABILITIES);
        g_assert_cmphex(qtest_readw(qts, base + SDHCI_HOST_VERSION),
                        ==, SDHCI_SPEC_100);
    }

    qtest_quit(qts);
}

static void test_cg200_32bit_mapping(void)
{
    QTestState *qts = sh7785lcr_start(",boot32=on");

    for (unsigned int slot = 0; slot < 2; slot++) {
        uint64_t base = CG200_32BIT_BASE + slot * CG200_SLOT_STRIDE;

        g_assert_cmphex(qtest_readl(qts, base + SDHCI_CAPABILITIES),
                        ==, CG200_CAPABILITIES);
    }

    qtest_quit(qts);
}

static void setup_dma_out_pipe(QTestState *qts, uint16_t max_packet)
{
    uint64_t base = R8A66597_29BIT_BASE;

    qtest_writew(qts, base + PIPESEL, 1);
    qtest_writew(qts, base + PIPECFG, TYPE_BULK | PIPE_DIR_OUT | 1);
    qtest_writew(qts, base + PIPEMAXP, max_packet);
}

static void test_r8a66597_dma_dreq(void)
{
    QTestState *qts = sh7785lcr_start("");
    uint64_t base = R8A66597_29BIT_BASE;

    qtest_irq_intercept_out_named(qts, "/machine/usb-host", "dreq");
    setup_dma_out_pipe(qts, 4);

    /* DREQ is active low by default and asserts for the empty OUT FIFO. */
    qtest_writew(qts, base + D0FIFOSEL, DREQE | MBW_16 | 1);
    g_assert_false(qtest_get_irq(qts, 0));

    /* DREQA selects active high; disabling DREQE negates the line. */
    qtest_writew(qts, base + DMA0CFG, DMA_DREQA | DMA_BURST);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writew(qts, base + D0FIFOSEL, MBW_16 | 1);
    g_assert_false(qtest_get_irq(qts, 0));

    qtest_quit(qts);
}

static void test_r8a66597_dma_dend(void)
{
    QTestState *qts = sh7785lcr_start("");
    uint64_t base = R8A66597_29BIT_BASE;

    setup_dma_out_pipe(qts, 64);
    qtest_writew(qts, base + PIPE1CTR, PID_BUF);
    qtest_writew(qts, base + DMA0CFG, DMA_DENDE);
    qtest_writew(qts, base + D0FIFOSEL, MBW_16 | 1);
    qtest_writew(qts, base + D0FIFO, 0x1234);
    g_assert_cmphex(qtest_readw(qts, base + D0FIFOCTR) & FRDY, ==, FRDY);

    /* DEND is active low by default and terminates an OUT packet. */
    qtest_set_irq_in(qts, "/machine/usb-host", "dend-in", 0, 1);
    qtest_set_irq_in(qts, "/machine/usb-host", "dend-in", 0, 0);
    g_assert_cmphex(qtest_readw(qts, base + D0FIFOCTR) & FRDY, ==, 0);

    qtest_quit(qts);
}

static void test_r8a66597_dma_dack(void)
{
    QTestState *qts = sh7785lcr_start("");
    uint64_t base = R8A66597_29BIT_BASE;

    qtest_irq_intercept_out_named(qts, "/machine/usb-host", "dreq");
    setup_dma_out_pipe(qts, 8);
    qtest_writew(qts, base + DMA0CFG, DMA_DFORM_DACK);
    qtest_writew(qts, base + D0FIFOSEL, DREQE | MBW_16 | 1);
    g_assert_false(qtest_get_irq(qts, 0));

    /* An inactive DACK blocks the FIFO access and leaves DREQ asserted. */
    qtest_writew(qts, base + D0FIFO, 0x1234);
    g_assert_false(qtest_get_irq(qts, 0));

    /* One active DACK cycle transfers a word and negates DREQ. */
    qtest_set_irq_in(qts, "/machine/usb-host", "dack", 0, 0);
    qtest_writew(qts, base + D0FIFO, 0x1234);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_set_irq_in(qts, "/machine/usb-host", "dack", 0, 1);
    g_assert_false(qtest_get_irq(qts, 0));

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/sh7785lcr/board/devices", test_board_devices);
    qtest_add_func("/sh7785lcr/cg200/32bit-mapping",
                   test_cg200_32bit_mapping);
    qtest_add_func("/sh7785lcr/r8a66597/dma-dreq",
                   test_r8a66597_dma_dreq);
    qtest_add_func("/sh7785lcr/r8a66597/dma-dack",
                   test_r8a66597_dma_dack);
    qtest_add_func("/sh7785lcr/r8a66597/dma-dend",
                   test_r8a66597_dma_dend);

    return g_test_run();
}
