/* vxbTiK3Am335xSpi.h - Freescale AM335x SPI hardware defintions */

/*
 * Copyright (c) 2019 Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
modification history
--------------------
13jun19,jff  created (VXWPG-114)
*/

#ifndef __INCvxbTiK3Am335xSpih
#define __INCvxbTiK3Am335xSpih

#ifdef __cplusplus
extern "C"
{
#endif

/* defines */

#define TI_AM335X_SPI_NAME    "tiAm335xSpi"

#define SPI_WL_MAX              32
#define SPI_WL_MIN               4
#define MAX_DIVIDERATIO_1     4096
#define MAX_DIVIDERATIO_2    32768
#define SPI_TIMEOUT_US      500000

/* SPI built-in FIFO lenght */

#define SPI_FIFO_LEN           32u

/* Register Addresses */

#define MCSPI_REVISION       0x000
#define MCSPI_SYSCONFIG      0x110
#define MCSPI_SYSSTATUS      0x114
#define MCSPI_IRQSTATUS      0x118
#define MCSPI_IRQENABLE      0x11C
#define MCSPI_SYST           0x124
#define MCSPI_MODULCTRL      0x128
#define MCSPI_CH0CONF        0x12c
#define MCSPI_CH0STAT        0x130
#define MCSPI_CH0CTRL        0x134
#define MCSPI_TX0            0x138
#define MCSPI_RX0            0x13c
#define MCSPI_CH1CONF        0x140
#define MCSPI_CH1STAT        0x144
#define MCSPI_CH1CTRL        0x148
#define MCSPI_TX1            0x14c
#define MCSPI_RX1            0x150
#define MCSPI_CH2CONF        0x154
#define MCSPI_CH2STAT        0x158
#define MCSPI_CH2CTRL        0x15c
#define MCSPI_TX2            0x160
#define MCSPI_RX2            0x164
#define MCSPI_CH3CONF        0x168
#define MCSPI_CH3STAT        0x16c
#define MCSPI_CH3CTRL        0x170
#define MCSPI_TX3            0x174
#define MCSPI_RX3            0x178
#define MCSPI_XFERLEVEL      0x17c
#define MCSPI_DAFTX          0x180
#define MCSPI_DAFRX          0x1a0

#define MCSPI_CHCONF(x)        (MCSPI_CH0CONF + 0x14*(x))
#define MCSPI_CHCTRL(x)        (MCSPI_CH0CTRL + 0x14*(x))
#define MCSPI_CHSTAT(x)        (MCSPI_CH0STAT + 0x14*(x))
#define MCSPI_TX(x)            (MCSPI_TX0 + 0x14*(x))
#define MCSPI_RX(x)            (MCSPI_RX0 + 0x14*(x))
#define MCSPI_IRQENABLE_TX(x)  ((BIT(0) /*| BIT(1)*/) << (x) * 4)
#define MCSPI_IRQENABLE_RX(x)  (BIT(2) << (x) * 4)
#define IS_RX_INT(ch, val)     ((((val) >> 4 * (ch)) & BIT(2)) != 0)
#define IS_TX_INT(ch, val)     \
              ((((val) >> 4 * (ch)) & (BIT(0) | BIT(1))) != 0)

#ifdef BIT
#undef BIT
#endif

#define BIT(x)  ((UINT32)0x1 << (x))

/* MCSPI_SYSCONFIG */

#define  SPI_RESET                 (BIT(1))

/* MCSPI_SYSSTATUS */

#define  SPI_RESET_DONE            (BIT(0))

/* MCSPI_MODULCTRL */

#define MCSPI_MODULCTRL_MS            (BIT(2))
#define MCSPI_MODULCTRL_SINGLE        (BIT(0))

#define MCSPI_CH0CONF_FFER            (BIT(28))
#define MCSPI_CH0CONF_FFEW            (BIT(27))
#define MCSPI_CH0CONF_DMAR            (BIT(15))
#define MCSPI_CH0CONF_DMAW            (BIT(14))
#define MCSPI_CH0CONF_WL_SHIFT        (7)
#define MCSPI_CH0CONF_CLKG_SHIFT      (29)
#define MCSPI_CH0CONF_CLKD_SHIFT      (2)
#define MCSPI_CH0CONF_EPOL_SHIFT      (6)
#define MCSPI_CH0CONF_CLKD_MASK       (0xf)
#define MCSPI_CH0CONF_POL             (BIT(1))
#define MCSPI_CH0CONF_PHA             (BIT(0))
#define MCSPI_CH0CONF_DPE1            ((BIT(17)) | (BIT(18)))
#define MCSPI_CH0CONF_TCS             (BIT(25))
#define MCSPI_CH0CONF_FORCE           (BIT(20))
#define MCSPI_CHOCONF_TRM_MASK        (BIT(12)|BIT(13)) 
#define MCSPI_CHOCONF_TRM_SHIFT       12
#define MCSPI_CH0CONF_POL             (BIT(1))

#define MCSPI_CH0STAT_TXFFE        (BIT(3))
#define MCSPI_CH0STAT_TXFFF        (BIT(4))
#define MCSPI_CH0STAT_RXFFE        (BIT(5))
#define MCSPI_CH0STAT_RXFFF        (BIT(6))
#define MCSPI_CH0STAT_TXS          (BIT(1))
#define MCSPI_CH0STAT_RXS          (BIT(0))
#define MCSPI_CH0STAT_EOT          (BIT(2))
#define MCSPI_MODULCTRL_FDAA       (BIT(8))

#define MCSPI_XFERLEVEL_AFL_SHIFT    8

#define MCSPI_TRM_RXTX            (0u << MCSPI_CHOCONF_TRM_SHIFT)
#define MCSPI_TRM_RX              (1u << MCSPI_CHOCONF_TRM_SHIFT)
#define MCSPI_TRM_TX              (2u << MCSPI_CHOCONF_TRM_SHIFT)

#define SPI0_SCLK_PINMUX_REG_OFFSET  0x1c4
#define SPI0_D0_PINMUX_REG_OFFSET    0x1c8
#define SPI0_D1_PINMUX_REG_OFFSET    0x1cc
#define SPI0_CS0_PINMUX_REG_OFFSET   0x1bc

#define MCSPI_PULLUDEN_SHIFT         (16)
#define MCSPI_PULLTYPESEL_SHIFT      (17)
#define MCSPI_RXACTIVE_SHIFT         (18)

#define MCSPI_PULL_DISABLE           (1 << MCSPI_PULLUDEN_SHIFT)
#define MCSPI_PULL_ENABLE            (0 << MCSPI_PULLUDEN_SHIFT)

#define MCSPI_PULL_UP        (1 << MCSPI_PULLTYPESEL_SHIFT | MCSPI_PULL_ENABLE)
#define MCSPI_PULL_DOWN      (0 << MCSPI_PULLTYPESEL_SHIFT | MCSPI_PULL_ENABLE)

#define MCSPI_INPUT_EN               (1 << MCSPI_RXACTIVE_SHIFT)
#define MCSPI_INPUT_DISABLE          (0 << MCSPI_RXACTIVE_SHIFT)

/* Only these macros are expected be used directly in device tree files */
#define MCSPI_PIN_OUTPUT             (MCSPI_INPUT_DISABLE | MCSPI_PULL_DISABLE)
#define MCSPI_PIN_OUTPUT_PULLUP      (MCSPI_INPUT_DISABLE | MCSPI_PULL_UP)
#define MCSPI_PIN_OUTPUT_PULLDOWN    (MCSPI_INPUT_DISABLE | MCSPI_PULL_DOWN)
#define MCSPI_PIN_INPUT              (MCSPI_INPUT_EN | MCSPI_PULL_DISABLE)
#define MCSPI_PIN_INPUT_PULLUP       (MCSPI_INPUT_EN | MCSPI_PULL_UP)
#define MCSPI_PIN_INPUT_PULLDOWN     (MCSPI_INPUT_EN | MCSPI_PULL_DOWN)

#define MCSPI_PIN_MUX_MODE              0
#define AM65X_IOPAD_VAL(val, muxMode)   ((val) | (muxMode))

#define MCSPI_CH0CTRL_EXTCLK_SHIFT      8
#define MCSPI_CH0CTRL_EN                (BIT(0))
#define IS_SPI_FULL_DUPLEX(mode)        (((mode) & SPI_FULL_DUPLEX) != 0)

/* Number of chip selects supported */

#define SPI_MAX_CS_NUM             2

/* Device default freq, 48M */

#define AM335X_SPI_CLK_DEFAULT         48000000

/* controllor read and write interface */

#define MC_SPI_BAR(p)       (((TI_AM335X_SPI_CTRL *) vxbDevSoftcGet(p))->regBase)
#define MC_SPI_HANDLE(p)    (((TI_AM335X_SPI_CTRL *) vxbDevSoftcGet(p))->regHandle)

#define MC_SPI_PINMUX_BAR(p)      (((TI_AM335X_SPI_CTRL *) vxbDevSoftcGet(p)) \
                                   ->pinMuxRegBase)
#define MC_SPI_PINMUX_HANDLE(p)   (((TI_AM335X_SPI_CTRL *) vxbDevSoftcGet(p)) \
                                   ->pinMuxRegHandle)

#undef CSR_READ_1
#define CSR_READ_1(pDev, addr)                                  \
    vxbRead8 (MC_SPI_HANDLE(pDev), (UINT8 *)((char *)MC_SPI_BAR(pDev) + addr))

#undef CSR_WRITE_1
#define CSR_WRITE_1(pDev, addr, data)               \
    vxbWrite8 (MC_SPI_HANDLE(pDev),                \
        (UINT8 *)((char *)MC_SPI_BAR(pDev) + addr), data)

#undef CSR_CLRBIT_1
#define CSR_CLRBIT_1(pDev, offset, val)             \
    CSR_WRITE_1(pDev, offset, CSR_READ_1(pDev, offset) & (val))

#undef CSR_SETBIT_1
#define CSR_SETBIT_1(pDev, offset, val)             \
    CSR_WRITE_1(pDev, offset, CSR_READ_1(pDev, offset) | (val))

#undef CSR_SETCLRBIT_1
#define CSR_SETCLRBIT_1(pDev, offset, val1, val2)          \
        CSR_WRITE_1(pDev, offset, (CSR_READ_1(pDev, offset) & val1) | val2)

#undef CSR_READ_4
#define CSR_READ_4(pDev, addr)                                  \
    vxbRead32 (MC_SPI_HANDLE(pDev),               \
        (UINT32 *)((char *)MC_SPI_BAR(pDev) + addr))

#undef CSR_WRITE_4
#define CSR_WRITE_4(pDev, addr, data)               \
    vxbWrite32 (MC_SPI_HANDLE(pDev),              \
        (UINT32 *)((char *)MC_SPI_BAR(pDev) + addr), data)

#define CSR_WRITE_PINMUX_4(pDev, addr, data)                            \
        vxbWrite32 (MC_SPI_PINMUX_HANDLE(pDev),                         \
            (UINT32 *)((char *)MC_SPI_PINMUX_BAR(pDev) + addr), data)

#undef CSR_SETBIT_4
#define CSR_SETBIT_4(pDev, offset, val)             \
    CSR_WRITE_4(pDev, offset, CSR_READ_4(pDev, offset) | (val))

#undef CSR_CLRBIT_4
#define CSR_CLRBIT_4(pDev, offset, val)             \
    CSR_WRITE_4(pDev, offset, CSR_READ_4(pDev, offset) & (val))

#undef CSR_SETCLRBIT_4
#define CSR_SETCLRBIT_4(pDev, offset, val1, val2)          \
        CSR_WRITE_4(pDev, offset, (CSR_READ_4(pDev, offset) & val1) | val2)

/* typedefs */

/* structure holding the instance specific details */

typedef struct ti_am335x_spi_drv_ctrl
    {
    VXB_DEV_ID      pDev;
    void *          regBase;
    void *          regHandle;
    void *          pinMuxRegBase;
    void *          pinMuxRegHandle;
    UINT32          clkFrequency;
    UINT8 *         txBuf;
    UINT32          txLen;
    UINT32          txLeft;
    UINT8 *         rxBuf;
    UINT32          rxLen;
    UINT32          rxLeft;
    UINT32          curWorkingFrq;
    SEM_ID          semSync;
    VXB_RESOURCE *  intRes;
    } TI_AM335X_SPI_CTRL;

#ifdef __cplusplus
}
#endif

#endif /* __INCvxbTiK3Am335xSpih */
