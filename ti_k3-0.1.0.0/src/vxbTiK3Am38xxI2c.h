/* vxbTiK3Am38xxI2c.h - AM38xx I2C hardware defintions */

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
19feb19,whe  created (VXWPG-114)
*/

#ifndef __INCvxbTiK3Am38xxI2ch
#define __INCvxbTiK3Am38xxI2ch

#ifdef __cplusplus
extern "C" {
#endif

/* defines */

#define AM38XX_I2C_NAME         "am38xxI2c"
#define AM38XX_I2C_FCLK_FREQ    (48000000)
#define AM38XX_I2C_ICLK_FREQ    (24000000)
#define AM38XX_I2C_FCK_NAME     "fck"

/* register definitions for I2C */

#define I2C_REVNB_LO        0x00    /* Module Revision Register (LOW BYTES) */
#define I2C_REVNB_HI        0x04    /* Module Revision Register (HIGH BYTES) */
#define I2C_SYSC            0x10    /* System configuration register */
#define I2C_EOI             0x20    /* I2C End of Interrupt Register */
#define I2C_IRQSTATUS_RAW   0x24    /* I2C Status Raw Register */
#define I2C_IRQSTATUS       0x28    /* I2C Status Register */
#define I2C_IRQENABLE_SET   0x2c    /* I2C Interrupt Enable Set Register */
#define I2C_IRQENABLE_CLR   0x30    /* I2C Interrupt Enable Clear Register */
#define I2C_WE              0x34    /* I2C Wakeup Enable Register */
#define I2C_DMARXENABLE_SET 0x38    /* Receive DMA Enable Set Register */
#define I2C_DMATXENABLE_SET 0x3c    /* Transmit DMA Enable Set Register */
#define I2C_DMARXENABLE_CLR 0x40    /* Receive DMA Enable Clear Register */
#define I2C_DMATXENABLE_CLR 0x44    /* Transmit DMA Enable Clear Register */
#define I2C_DMARXWAKE_EN    0x48    /* Receive DMA Wakeup Register */
#define I2C_DMATXWAKE_EN    0x4c    /* Transmit DMA Wakeup Register */
#define I2C_SYSS            0x90    /* System Status Register */
#define I2C_BUF             0x94    /* Buffer Configuration Register */
#define I2C_CNT             0x98    /* Data Counter Register */
#define I2C_DATA            0x9c    /* Data Access Register */
#define I2C_CON             0xa4    /* I2C Configuration Register */
#define I2C_OA              0xa8    /* I2C Own Address Register */
#define I2C_SA              0xac    /* I2C Slave Address Register */
#define I2C_PSC             0xb0    /* I2C Clock Prescaler Register */
#define I2C_SCLL            0xb4    /* I2C SCL Low Time Register */
#define I2C_SCLH            0xb8    /* I2C SCL High Time Register */
#define I2C_SYSTEST         0xbc    /* System Test Register */
#define I2C_BUFSTAT         0xc0    /* I2C Buffer Status Register */
#define I2C_OA1             0xc4    /* I2C Own Address 1 Register */
#define I2C_OA2             0xc8    /* I2C Own Address 2 Register */
#define I2C_OA3             0xcc    /* I2C Own Address 3 Register */
#define I2C_ACTOA           0xd0    /* Active Own Address Register */
#define I2C_SBLOCK          0xd4    /* I2C Clock Blocking Enable Registe */

#define I2C_IRQENABLE_XDR   (0x1 << 14)
#define I2C_IRQENABLE_RDR   (0x1 << 13)
#define I2C_IRQENABLE_ROVR  (0x1 << 11)
#define I2C_IRQENABLE_XUDF  (0x1 << 10)
#define I2C_IRQENABLE_AAS   (0x1 << 9)
#define I2C_IRQENABLE_BF    (0x1 << 8)
#define I2C_IRQENABLE_AERR  (0x1 << 7)
#define I2C_IRQENABLE_STC   (0x1 << 6)
#define I2C_IRQENABLE_GC    (0x1 << 5)
#define I2C_IRQENABLE_XRDY  (0x1 << 4)
#define I2C_IRQENABLE_RRDY  (0x1 << 3)
#define I2C_IRQENABLE_ARDY  (0x1 << 2)
#define I2C_IRQENABLE_NACK  (0x1 << 1)
#define I2C_IRQENABLE_AL    (0x1 << 0)

#define I2C_IRQSTATUS_XDR   (0x1 << 14)
#define I2C_IRQSTATUS_RDR   (0x1 << 13)
#define I2C_IRQSTATUS_BB    (0x1 << 12)
#define I2C_IRQSTATUS_ROVR  (0x1 << 11)
#define I2C_IRQSTATUS_XUDF  (0x1 << 10)
#define I2C_IRQSTATUS_AAS   (0x1 << 9)
#define I2C_IRQSTATUS_BF    (0x1 << 8)
#define I2C_IRQSTATUS_AERR  (0x1 << 7)
#define I2C_IRQSTATUS_STC   (0x1 << 6)
#define I2C_IRQSTATUS_GC    (0x1 << 5)
#define I2C_IRQSTATUS_XRDY  (0x1 << 4)
#define I2C_IRQSTATUS_RRDY  (0x1 << 3)
#define I2C_IRQSTATUS_ARDY  (0x1 << 2)
#define I2C_IRQSTATUS_NACK  (0x1 << 1)
#define I2C_IRQSTATUS_AL    (0x1 << 0)

#define I2C_IRQSTATUS_ALL       (0x7FFF)
#define I2C_IRQSTATUS_DEFAULT   (I2C_IRQSTATUS_ROVR     | \
                                 I2C_IRQSTATUS_XUDF     | \
                                 I2C_IRQSTATUS_AERR     | \
                                 I2C_IRQSTATUS_STC      | \
                                 I2C_IRQSTATUS_NACK     | \
                                 I2C_IRQSTATUS_AL)

#define I2C_CON_EN          (0x1 << 15)
#define I2C_CON_STB         (0x1 << 11)
#define I2C_CON_MST         (0x1 << 10)
#define I2C_CON_TRX         (0x1 << 9 )
#define I2C_CON_XSA         (0x1 << 8 )
#define I2C_CON_STP         (0x1 << 1 )
#define I2C_CON_STT         (0x1 << 0 )

#define I2C_BUF_RXFIFO_CLR      (0x1 << 14)
#define I2C_BUF_TXFIFO_CLR      (0x1 << 6)
#define I2C_BUF_RXTRSH_SHIFT    (8)

#define I2C_BUFSTAT_FIFODEPTH_MASK      (0xC000)
#define I2C_BUFSTAT_FIFODEPTH_SHIFT     (14)

#define I2C_BUFSTAT_RXSTAT_MASK         (0x3F00)
#define I2C_BUFSTAT_RXSTAT_SHIFT        (8)

#define I2C_BUFSTAT_TXSTAT_MASK         (0x3F)
#define I2C_BUFSTAT_TXSTAT_SHIFT        (0)

#define I2C_BUFSTAT_FIFODEPTH_8         (0x0)
#define I2C_BUFSTAT_FIFODEPTH_16        (0x1)
#define I2C_BUFSTAT_FIFODEPTH_32        (0x2)
#define I2C_BUFSTAT_FIFODEPTH_64        (0x3)

#define I2C_SYSC_SRST       (1 << 1)
#define I2C_SYSS_RDONE      (1 << 0)

#define I2C_CNT_MAX         65535
#define I2C_TIMEOUT         20000

/* assume the slowest transfer speed is 1/5 of busFreq */

#define SEM_TIMEOUT         5
#define MAX_I2C_NUM         4

/* I2C RTC pinmux */

#define I2C0_SCL_PINMUX_REG_OFFSET 0x02e8
#define I2C0_SDA_PINMUX_REG_OFFSET 0x02ec

#define I2C_PULLUDEN_SHIFT         (16)
#define I2C_PULLTYPESEL_SHIFT      (17)
#define I2C_RXACTIVE_SHIFT         (18)

#define I2C_PULL_DISABLE           (1 << I2C_PULLUDEN_SHIFT)
#define I2C_PULL_ENABLE            (0 << I2C_PULLUDEN_SHIFT)

#define I2C_PULL_UP        (1 << I2C_PULLTYPESEL_SHIFT | I2C_PULL_ENABLE)
#define I2C_PULL_DOWN      (0 << I2C_PULLTYPESEL_SHIFT | I2C_PULL_ENABLE)

#define I2C_INPUT_EN               (1 << I2C_RXACTIVE_SHIFT)
#define I2C_INPUT_DISABLE          (0 << I2C_RXACTIVE_SHIFT)

/* Only these macros are expected be used directly in device tree files */

#define I2C_PIN_OUTPUT             (I2C_INPUT_DISABLE | I2C_PULL_DISABLE)
#define I2C_PIN_OUTPUT_PULLUP      (I2C_INPUT_DISABLE | I2C_PULL_UP)
#define I2C_PIN_OUTPUT_PULLDOWN    (I2C_INPUT_DISABLE | I2C_PULL_DOWN)
#define I2C_PIN_INPUT              (I2C_INPUT_EN | I2C_PULL_DISABLE)
#define I2C_PIN_INPUT_PULLUP       (I2C_INPUT_EN | I2C_PULL_UP)
#define I2C_PIN_INPUT_PULLDOWN     (I2C_INPUT_EN | I2C_PULL_DOWN)

#define I2C_PIN_MUX_MODE              0
#define AM65X_IOPAD_VAL(val, muxMode)   ((val) | (muxMode))

#define MC_I2C_PINMUX_BAR(p)      (((I2C_DRV_CTRL *) vxbDevSoftcGet(p)) \
                                   ->pinMuxRegBase)
#define MC_I2C_PINMUX_HANDLE(p)   (((I2C_DRV_CTRL *) vxbDevSoftcGet(p)) \
                                   ->pinMuxRegHandle)

#define CSR_WRITE_PINMUX_4(pDev, addr, data)                            \
        vxbWrite32 (MC_I2C_PINMUX_HANDLE(pDev),                         \
            (UINT32 *)((char *)MC_I2C_PINMUX_BAR(pDev) + addr), data)

/* I2C controller read and write interface */

#define AM38XX_I2C_BAR(p)       (((I2C_DRV_CTRL *)vxbDevSoftcGet(p))->bar)
#define AM38XX_I2C_HANDLE(p)    ((I2C_DRV_CTRL *)vxbDevSoftcGet(p))->i2cHandle

#define CSR_READ_4(pDev, addr)                          \
    vxbRead32 (AM38XX_I2C_HANDLE(pDev),                 \
        (UINT32 *)((char *)AM38XX_I2C_BAR(pDev) + addr))

#define CSR_WRITE_4(pDev, addr, data)                   \
    vxbWrite32 (AM38XX_I2C_HANDLE(pDev),                \
        (UINT32 *)((char *)AM38XX_I2C_BAR(pDev) + addr), data)

/* I2C rw modes */

#define I2C_RW_MODE_INT         (0)
#define I2C_RW_MODE_POLL        (1)
#define I2C_RW_MODE_INVALIDE    (2)
#define I2C_RWMODE              (I2C_RW_MODE_INT)

/* structure holding the instance specific details */

typedef struct i2c_drv_ctrl {
    VXB_DEV_ID      i2cDev;
    void *          i2cHandle;
    ULONG           bar;
    SEM_ID          i2cDevSem;
    SEM_ID          i2cDataSem;
    UINT32          fClkFreq;
    UINT32          iClkFreq;
    UINT32          busSpeed;
    UINT32          defBusSpeed;
    UINT32          rwMode;
    BOOL            i2cStatus;
    BOOL            intEnabled;
    UINT8 *         dataBuf;
    UINT32          dataLength;
    UINT32          fifoLen;
    UINT32          rxThreshold;
    UINT32          txThreshold;
    void *          pinMuxRegBase;
    void *          pinMuxRegHandle;
    VXB_RESOURCE *  intRes;
} I2C_DRV_CTRL;

#ifdef __cplusplus
}
#endif

#endif /* __INCvxbTiK3Am38xxI2ch */
