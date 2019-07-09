/* vxbTiK3Am3Sio.h - TI AM3xxx SIO Header file */

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
20may19,ghs  enable new TTY device name rule (F10912)
19feb19,whe  created (VXWPG-114)
*/

#ifndef __INCvxbTiK3Am3Sioh
#define __INCvxbTiK3Am3Sioh

#ifdef __cplusplus
extern "C" {
#endif

/*
 *********************************************************************
 *      Copyright (c) 1990,1991 Intel Corporation
 *
 * Intel hereby grants you permission to copy, modify, and
 * distribute this software and its documentation.  Intel grants
 * this permission provided that the above copyright notice
 * appears in all copies and that both the copyright notice and
 * this permission notice appear in supporting documentation.  In
 * addition, Intel grants this permission provided that you
 * prominently mark as not part of the original any modifications
 * made to this software or documentation, and that the name of
 * Intel Corporation not be used in advertising or publicity
 * pertaining to distribution of the software or the documentation
 * without specific, written prior permission.
 *
 * Intel Corporation does not warrant, guarantee or make any
 * representations regarding the use of, or the results of the use
 * of, the software and documentation in terms of correctness,
 * accuracy, reliability, currentness, or otherwise; and you rely
 * on the software, documentation and results solely at your own risk.
 *
 *\NOMANUAL
 **********************************************************************
*/

/*
 ******************************************************************************
 *
 * REGISTER DESCRIPTION OF NATIONAL 16552 DUART
 *
 * $Id: ns16552.h,v 2.1 1993/06/07 15:07:59 wise active $
 *
 *\NOMANUAL
 *******************************************************************************
*/

#ifndef _ASMLANGUAGE

#include <ttyLib.h>
#include <sioLib.h>
#include <spinLockLib.h>
#include <hwif/util/vxbIsrDeferLib.h>

/* Register offsets from base address */

#define RBR     0x00    /* receiver buffer register */
#define THR     0x00    /* transmit holding register */
#define DLL     0x00    /* divisor latch */
#define IER     0x04    /* interrupt enable register */
#define DLM     0x04    /* divisor latch(MS) */
#define IIR     0x08    /* interrupt identification register */
#define FCR     0x08    /* FIFO control register */
#define EFR     0x08    /* enhanced feature register */
#define LCR     0x0c    /* line control register */
#define MCR     0x10    /* modem control register */
#define LSR     0x14    /* line status register */
#define MSR     0x18    /* modem status register */
#define TCR     0x18    /* modem status register */
#define TLR     0x1C    /* trigger level register */
#define MDR1    0x20    /* mode define register 1 */
#define MDR2    0x24    /* mode define register 2 */
#define SCR     0x40    /* supplementary control register */
#define SSR     0x44    /* supplementary status register */

#define BAUD_LO(baud)  ((XTAL/(16*baud)) & 0xff)
#define BAUD_HI(baud)  (((XTAL/(16*baud)) & 0xff00) >> 8)

/* Line Control Register */

#define CHAR_LEN_5  0x00    /* 5bits data size */
#define CHAR_LEN_6  0x01    /* 6bits data size */
#define CHAR_LEN_7  0x02    /* 7bits data size */
#define CHAR_LEN_8  0x03    /* 8bits data size */
#define LCR_STB     0x04    /* 2 stop bits */
#define ONE_STOP    0x00    /* one stop bit */
#define LCR_PEN     0x08    /* parity enable */
#define PARITY_NONE 0x00    /* parity disable */
#define LCR_EPS     0x10    /* even parity select */
#define LCR_SP      0x20    /* stick parity select */
#define LCR_SBRK    0x40    /* break control bit */
#define LCR_DLAB    0x80    /* divisor latch access enable */
#define DLAB        LCR_DLAB

/* Line Status Register */

#define LSR_DR      0x01    /* data ready */
#define RxCHAR_AVAIL    LSR_DR  /* data ready */
#define LSR_OE      0x02    /* overrun error */
#define LSR_PE      0x04    /* parity error */
#define LSR_FE      0x08    /* framing error */
#define LSR_BI      0x10    /* break interrupt */
#define LSR_THRE    0x20    /* transmit holding register empty */
#define LSR_TEMT    0x40    /* transmitter empty */
#define LSR_FERR    0x80    /* in fifo mode, set when PE,FE or BI error */

/* Interrupt Identification Register */

#define IIR_IP      0x01
#define IIR_ID      0x0e
#define IIR_RLS     0x06    /* received line status */
#define Rx_INT      IIR_RLS /* received line status */
#define IIR_RDA     0x04    /* received data available */
#define RxFIFO_INT  IIR_RDA /* received data available */
#define IIR_THRE    0x02    /* transmit holding register empty */
#define TxFIFO_INT  IIR_THRE
#define IIR_MSTAT   0x00    /* modem status */
#define IIR_TIMEOUT 0x0c    /* char receive timeout */

/* Interrupt Enable Register */

#define IER_ERDAI   0x01    /* received data avail. & timeout int */
#define RxFIFO_BIT  IER_ERDAI
#define IER_ETHREI  0x02    /* transmitter holding register empty int */
#define TxFIFO_BIT  IER_ETHREI
#define IER_ELSI    0x04    /* receiver line status int enable */
#define Rx_BIT      IER_ELSI
#define IER_EMSI    0x08    /* modem status int enable */

/* Modem Control Register */

#define MCR_DTR     0x01    /* dtr output */
#define DTR         MCR_DTR
#define MCR_RTS     0x02    /* rts output */
#define MCR_OUT1    0x04    /* output #1 */
#define MCR_OUT2    0x08    /* output #2 */
#define MCR_LOOP    0x10    /* loopback enable */
#define MCR_TCRTLR  0x40    /* access to TCR, TLR retisters */

/* Modem Status Register */

#define MSR_DCTS    0x01    /* cts change */
#define MSR_DDSR    0x02    /* dsr change */
#define MSR_TERI    0x04    /* ring indicator change */
#define MSR_DDCD    0x08    /* data carrier indicator change */
#define MSR_CTS     0x10    /* complement of cts */
#define MSR_DSR     0x20    /* complement of dsr */
#define MSR_RI      0x40    /* complement of ring signal */
#define MSR_DCD     0x80    /* complement of dcd */

/* FIFO Control Register */

#define FCR_EN      0x01    /* enable xmit and rcvr */
#define FIFO_ENABLE FCR_EN
#define FCR_RXCLR   0x02    /* clears rcvr fifo */
#define RxCLEAR     FCR_RXCLR
#define FCR_TXCLR   0x04    /* clears xmit fifo */
#define TxCLEAR     FCR_TXCLR
#define FCR_DMA     0x08    /* dma */
#define FCR_RXTRIG_L    0x40    /* rcvr fifo trigger lvl low */
#define FCR_RXTRIG_H    0x80    /* rcvr fifo trigger lvl high */

/* FIFO depth */

#define AM3_SIO_FIFO_DEPTH_UNKNOWN    0
#define AM3_SIO_FIFO_DEPTH_NO         1
#define AM3_SIO_FIFO_DEPTH_DEFAULT    16

/* Enhanced Feature Register */

#define EFR_ENHANCEDEN      0x10    /* enable functions write */
#define EFR_AUTO_RTS_EN     0x40    /* Enable RTS */
#define EFR_AUTO_CTS_EN     0x80    /* Enable CTS */

/* Modem Define Register */

#define MDR_DIS             0x07    /* disable uart */

/* supplementary status register */

#define SSR_TXFIFOFULL      0x01

#define DISABLE_DEFER                 0x00000001

typedef  struct tiAm3SioChan     /* AM3_SIOVXB_CHAN * */
    {
    /* always goes first */

    SIO_DRV_FUNCS *     pDrvFuncs;      /* driver functions */

    /* callbacks */

    STATUS              (*getTxChar) (); /* pointer to xmitr function */
    STATUS              (*putRcvChar) (); /* pointer to rcvr function */
    void *              getTxArg;
    void *              putRcvArg;
    char                tyName [20];

    UINT8 *             regs;       /* AM3_SIO registers */
    UINT8               level;      /* 8259a interrupt level for this device */
    UINT8               ier;        /* copy of ier */
    UINT8               lcr;        /* copy of lcr, not used by ns16552 driver */
    UINT8               mcr;        /* copy of modem control register */
    UINT8               ierInit;    /* initial IER register value */
    UINT16              fifoSize;   /* FIFO depth */

    UINT16              channelMode;/* such as INT, POLL modes */
    UINT32              baudRate;
    UINT32              xtal;       /* UART clock frequency     */
    UINT32              options;    /* hardware setup options */

    ULONG               bar;        /* base address */
    VXB_DEV_ID          pDev;       /* bus subsystem: device ID */
    void *              handle;
    VXB_RESOURCE *      adrRes;
    VXB_RESOURCE *      intRes;
    UINT8               rxTrgLvl;
    UINT8               txTrgLvl;

    int                 channelNo;      /* channel number */
    int                 ttyIndex;       /* tty device index */
    spinlockIsr_t       spinlockIsr;    /* ISR-callable spinlock */
    BOOL                txDefer;        /* TX interrupt is currently deferred */
    ISR_DEFER_QUEUE_ID  queueId;
    ISR_DEFER_JOB       isrDefRd;
    ISR_DEFER_JOB       isrDefWr;
    UINT32              flag;           /* specific flag for different condition */

    /*
     * Each device may have multiple channels.  To handle this,
     * we keep a pointer to a pChan in pDev->pDrvCtrl, and maintain
     * a linked list of channels per controller.
     */
    struct tiAm3SioChan * pNext; /* next channel for this device */
    } AM3_SIO_CHAN;

#endif  /* _ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* __INCvxbTiK3Am3Sioh */
