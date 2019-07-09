/* vxbTiK3Am3Sio.c - TI AM3xxx SIO Driver */

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

/*
DESCRIPTION

This is the VxBus driver for the TI AM3xxx UART.
In general, this device includes two universal asynchronous
receiver/transmitters, a baud rate generator, and a complete
modem control capability.

Only asynchronous serial operation is supported by this driver.
The default serial settings are 8 data bits, 1 stop bit, no parity,
and software flow control.

Each SIO device should be bound to a device tree node which requires
below properties:

\cs
compatible:     Specify the programming model for the device.
                It should be set to "ti,am3-uart" or "ti,am5-uart", and is used
                by vxbus GEN2 for device driver selection.

reg:            Specify the address of the device's resources within
                the address space defined by its parent bus.

clocks:         List of clock source phandle and clock parameter pairs,
                one pair present one clock it uses.

clock-names:    List of clock source names corresponding to the clock
                source phandles.

def-baudrate:   Optional, specify the default working baud rate of the
                SIO device. If not specified, 115200 is used.

interrupts:     Specify interrupt vector of the interrupts that are generated
                by this device.

interrupt-parent: This property is available to define an interrupt
                parent. if it is missing from a device, it's interrupt parent
                is assumed to be its device tree parent.

pinmux-0:       Optional, specifies the PinMux phandle for the controller.

tx-trg-lvl:     Optional, specify the default transmit trigger level of the SIO
                device, the value must be between 1 and 15 (from 4 to 60
                characters with a granularity of 4 characters). If not
                specified, 15 is used.

rx-trg-lvl:     Optional, specify the default receiver trigger level of the SIO
                device, the value must be between 1 and 15 (from 4 to 60
                characters with a granularity of 4 characters). If not
                specified, 15 is used.
\ce

Below is an example which demonstrated the usage of the SIO device:

\cs
        serial1: serial@48022000
            {
            compatible = "ti,am3-uart";
            pinmux-0 = <&uart1_pads>;
            reg = <0x48022000 0x2000>;
            interrupts = <73>;
            interrupt-parent = <&intc>;
            clocks = <&clk 117>;
            clock-names = "fck";
            };
\ce

INCLUDE FILES: vxBus.h sioLib.h string.h vxbFdtLib.h
*/

/* includes */

#include <vxWorks.h>
#include <vsbConfig.h>
#include <intLib.h>
#include <errnoLib.h>
#include <errno.h>
#include <sioLib.h>
#include <ioLib.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ttyLib.h>

#ifdef    _WRS_CONFIG_SMP
#include <private/spinLockLibP.h>
#endif    /* _WRS_CONFIG_SMP */

#include <hwif/vxBus.h>
#include <hwif/buslib/vxbFdtLib.h>
#include <subsys/clk/vxbClkLib.h>
#include <subsys/int/vxbIntLib.h>
#include <subsys/pinmux/vxbPinMuxLib.h>
#include <hwif/drv/sio/vxbSioUtil.h>
#include "vxbTiK3Am3Sio.h"

#define AM3_SIO_INT_COUNT 1

#undef AM3_SIO_DEBUG_ON
#ifdef AM3_SIO_DEBUG_ON
#include <logLib.h>
LOCAL int tiAm3SiovxbDebugLevel = 1000;

#define AM3_SIO_DBG_MSG(level,fmt,a,b,c,d,e,f) \
        if ( tiAm3SiovxbDebugLevel >= level )  \
            if (_func_logMsg != NULL)          \
                _func_logMsg(fmt,a,b,c,d,e,f)

#else /* AM3_SIO_DEBUG_ON */

#define AM3_SIO_DBG_MSG(level,fmt,a,b,c,d,e,f)

#endif /* AM3_SIO_DEBUG_ON */

/* local defines       */

#ifdef    _WRS_CONFIG_SMP
LOCAL BOOL am3SioSpinlockFuncReady = FALSE;

#define VXB_AM3_SIO_SPIN_LOCK_ISR_TAKE(pChan)                \
    if (am3SioSpinlockFuncReady)                             \
        SPIN_LOCK_ISR_TAKE(&pChan->spinlockIsr)
#define VXB_AM3_SIO_SPIN_LOCK_ISR_GIVE(pChan)                \
    if (am3SioSpinlockFuncReady)                             \
        SPIN_LOCK_ISR_GIVE(&pChan->spinlockIsr)
#define VXB_AM3_SIO_SPIN_LOCK_ISR_HELD(pChan)                \
    (am3SioSpinlockFuncReady ? spinLockIsrHeld(&pChan->spinlockIsr) : FALSE)
#define VXB_AM3_SIO_ISR_SET(pChan)                           \
    SPIN_LOCK_ISR_TAKE(&pChan->spinlockIsr)
#define VXB_AM3_SIO_ISR_CLEAR(pChan)                         \
    SPIN_LOCK_ISR_GIVE(&pChan->spinlockIsr)
#define VXB_AM3_SIO_SPIN_LOCK_READY                          \
    am3SioSpinlockFuncReady = TRUE
#else
#define VXB_AM3_SIO_SPIN_LOCK_ISR_TAKE(pChan)                \
    SPIN_LOCK_ISR_TAKE(&pChan->spinlockIsr)
#define VXB_AM3_SIO_SPIN_LOCK_ISR_GIVE(pChan)                \
    SPIN_LOCK_ISR_GIVE(&pChan->spinlockIsr)
#define VXB_AM3_SIO_SPIN_LOCK_ISR_HELD(pChan)    FALSE
#define VXB_AM3_SIO_ISR_SET(pChan)
#define VXB_AM3_SIO_ISR_CLEAR(pChan)
#define VXB_AM3_SIO_SPIN_LOCK_READY
#endif    /* _WRS_CONFIG_SMP */

/* min/max baud rate */

#define AM3_SIO_MIN_RATE     300
#define AM3_SIO_MAX_RATE     3688400

#define AM5_SIO_MIN_RATE     1200
#define AM5_SIO_MAX_RATE     12000000

#define AM3_OP_MOD_16        0
#define AM3_OP_MOD_13        3

#define AM3_CLK_48_MHZ       48000000
#define AM3_BAUD_230_4_KBPS  230400
#define AM3_BAUD_3_MBPS      3000000
#define AM3_BAUD_921_6_KBPS  921600
#define AM3_BAUD_12_MBPS     12000000


#define AM3_UART_DEFAULT_TX_TRIGGER_LEVEL  0xF
#define AM3_UART_DEFAULT_RX_TRIGGER_LEVEL  0xF

#define REG_GET(reg, pchan, val)                  \
    val = vxbRead8 (pchan->handle,                \
                    (UINT8 *)pchan->bar + reg );

#define REG_SET(reg, pchan, val)                  \
    vxbWrite8 (pchan->handle,                     \
               (UINT8 *)pchan->bar + reg, val);

/* static forward declarations */

#ifndef    _VXBUS_BASIC_SIOPOLL
LOCAL   int     tiAm3SiovxbCallbackInstall (SIO_CHAN *, int, STATUS (*)(), void *);
LOCAL   STATUS  tiAm3SiovxbModeSet (AM3_SIO_CHAN *, UINT);
LOCAL   STATUS  tiAm3SiovxbIoctl (AM3_SIO_CHAN *, int, void *);
LOCAL   int     tiAm3SiovxbTxStartup (AM3_SIO_CHAN *);
LOCAL   int     tiAm3SiovxbPollInput (AM3_SIO_CHAN *, char *);
LOCAL   STATUS  tiAm3SiovxbOpen (AM3_SIO_CHAN * pChan );
LOCAL   STATUS  tiAm3SiovxbHup (AM3_SIO_CHAN * pChan );
LOCAL   void    tiAm3SiovxbIntRd2 (void * pData);
LOCAL   void    tiAm3SiovxbIntWr2 (void * pData);
void   tiAm3SiovxbInt (VXB_DEV_ID pDev);
#endif    /* _VXBUS_BASIC_SIOPOLL */

LOCAL   void    tiAm3SiovxbDevInit (AM3_SIO_CHAN * pChan);
LOCAL   STATUS  tiAm3SiovxbDummyCallback ();
LOCAL   void    tiAm3SiovxbInitChannel (AM3_SIO_CHAN *);
LOCAL   STATUS  tiAm3SiovxbBaudSet (AM3_SIO_CHAN *, UINT);
LOCAL   int     tiAm3SiovxbPollOutput (AM3_SIO_CHAN *, char);
LOCAL   STATUS  tiAm3SiovxbOptsSet (AM3_SIO_CHAN *, UINT);
LOCAL   STATUS  tiAm3SioProbe (VXB_DEV_ID pDev);
LOCAL   STATUS  tiAm3SioAttach (VXB_DEV_ID pDev);
LOCAL   STATUS  tiAm3SioShutdown (VXB_DEV_ID pDev);
LOCAL   STATUS  tiAm3SioDetach (VXB_DEV_ID pDev);
LOCAL   STATUS  tiAm3SioCreate (AM3_SIO_CHAN * pChan);
/* driver functions */

static SIO_DRV_FUNCS tiAm3SiovxbSioDrvFuncs =
    {
#ifdef    _VXBUS_BASIC_SIOPOLL
    (int (*)())NULL,
    (int (*)())NULL,
    (int (*)())NULL,
    (int (*)())NULL,
#else    /* _VXBUS_BASIC_SIOPOLL */
    (int (*)())tiAm3SiovxbIoctl,
    (int (*)())tiAm3SiovxbTxStartup,
    (int (*)())tiAm3SiovxbCallbackInstall,
    (int (*)())tiAm3SiovxbPollInput,
#endif    /* _VXBUS_BASIC_SIOPOLL */
    (int (*)(SIO_CHAN *,char))tiAm3SiovxbPollOutput
    };

typedef struct tiAm3SioCfg
    {
    UINT16      fifoSize;   /* FIFO depth */
    UINT32      baud;       /* default baud */
    }AM3_SIO_CFG;

LOCAL VXB_DRV_METHOD tiAm3SioMethodList[] =
    {
    /* DEVICE API */
    { VXB_DEVMETHOD_CALL(vxbDevProbe), tiAm3SioProbe },
    { VXB_DEVMETHOD_CALL(vxbDevAttach), tiAm3SioAttach },
    { VXB_DEVMETHOD_CALL(vxbDevShutdown), tiAm3SioShutdown },
    { VXB_DEVMETHOD_CALL(vxbDevDetach), tiAm3SioDetach },
    { 0, NULL }
    };

/* tiAm3Sio openfirmware driver */

VXB_DRV tiK3Am3SioDrv =
    {
    { NULL } ,
    "Ti Am3xxx SIO",                /* Name */
    "Ti AM3xxx serial FDT driver",  /* Description */
    VXB_BUSID_FDT,                  /* Class */
    0,                              /* Flags */
    0,                              /* Reference count */
    tiAm3SioMethodList              /* Method table */
    };

LOCAL const AM3_SIO_CFG tiAm3SioCfg =
    {
    AM3_SIO_FIFO_DEPTH_DEFAULT,     /* fifoSize */
    115200                          /* default baud */
    };

LOCAL const AM3_SIO_CFG tiAm5SioCfg =
    {
    64,                             /* fifoSize */
    115200                          /* default baud */
    };

LOCAL const VXB_FDT_DEV_MATCH_ENTRY tiAm3SioMatch[] =
    {
        {
        "ti,am3-uart",
        (void *)&tiAm3SioCfg
        },
        {
        "ti,am5-uart",
        (void *)&tiAm5SioCfg
        },
        {} /* Empty terminated list */
    };

VXB_DRV_DEF(tiK3Am3SioDrv)

/******************************************************************************
*
* tiAm3SioProbe - probe for device presence at specific address
*
* Check for tiAm3Siovxb (or compatible) device at the specified base
* address.  We assume one is present at that address, but
* we need to verify.
*
* RETURNS: OK if probe passes and assumed a valid tiAm3Siovxb
* (or compatible) device.  ERROR otherwise.
*
*/

LOCAL STATUS tiAm3SioProbe
    (
    VXB_DEV_ID pDev /* Device information */
    )
    {
    VXB_FDT_DEV_MATCH_ENTRY *pMatch;
    AM3_SIO_CFG * pCfg;
    STATUS score;

    score = vxbFdtDevMatch (pDev, tiAm3SioMatch, &pMatch);

    if (score == ERROR)
        return ERROR;

    pCfg = (AM3_SIO_CFG *)pMatch->data;

    vxbDevDrvDataSet(pDev,(void *)pCfg);

    return score;
    }

/******************************************************************************
*
* tiAm3SioAttach - attach for device presence at specific address
*
* This is the tiAm3Sio initialization routine.
*
* RETURNS: OK or ERROR when initialize failed
*
* ERRNO
*/

LOCAL STATUS tiAm3SioAttach
    (
    struct vxbDev * pDev /* Device information */
    )
    {
    VXB_FDT_DEV *      pFdtDev;
    UINT32             xtal;
    VXB_CLK *          pClk;
    VXB_RESOURCE_ADR * pResAdr = NULL;
    VXB_RESOURCE *     pRes;
    AM3_SIO_CHAN *     pChan;
    AM3_SIO_CFG *      tiAm3SioCfg;
    UINT32 *           prop;

    tiAm3SioCfg = (AM3_SIO_CFG *)vxbDevDrvDataGet(pDev);

    if (tiAm3SioCfg == NULL)
        return ERROR;

    pFdtDev = (VXB_FDT_DEV *)(vxbFdtDevGet(pDev));

    if (pFdtDev == NULL)
        return ERROR;

    pClk = vxbClkGet (pDev, NULL);

    if (pClk == NULL)
        return ERROR;

    (void) vxbClkEnableAll (pDev);
    (void) vxbPinMuxEnable (pDev);

    xtal = (UINT32)vxbClkRateGet (pClk);

    if (xtal == 0)
        return ERROR;

    pChan = (AM3_SIO_CHAN *)vxbMemAlloc(sizeof(*pChan));

    if ( pChan == NULL )
        return ERROR;

    pChan->ttyIndex = -1;

    vxbDevSoftcSet(pDev, (void *)pChan);

    pChan->xtal = xtal;
    if ((prop = (UINT32 *)vxFdtPropGet (pFdtDev->offset,
                                        "def-baudrate", NULL)) != NULL)
        {
        pChan->baudRate = vxFdt32ToCpu (prop[0]);
        }
    else
        {
        pChan->baudRate = tiAm3SioCfg->baud;
        }

    if ((prop = (UINT32 *)vxFdtPropGet (pFdtDev->offset,
                                        "tx-trg-lvl", NULL)) != NULL)
        {
        pChan->txTrgLvl = (UINT8)vxFdt32ToCpu (prop[0]);
        }
    else
        {
        pChan->txTrgLvl = AM3_UART_DEFAULT_TX_TRIGGER_LEVEL;
        }

    if ((prop = (UINT32 *)vxFdtPropGet (pFdtDev->offset,
                                        "rx-trg-lvl", NULL)) != NULL)
        {
        pChan->rxTrgLvl = (UINT8)vxFdt32ToCpu (prop[0]);
        }
    else
        {
        pChan->rxTrgLvl = AM3_UART_DEFAULT_RX_TRIGGER_LEVEL;
        }

    prop = (UINT32 *)vxFdtPropGet (pFdtDev->offset, "fifoSize", NULL);
    if (prop == NULL)
        pChan->fifoSize = tiAm3SioCfg->fifoSize;
    else
        pChan->fifoSize = (UINT16)vxFdt32ToCpu (prop[0]);

    pChan->pDev = pDev;

    pRes = vxbResourceAlloc(pDev, VXB_RES_MEMORY, 0);

    if(pRes == NULL)
        {
        vxbMemFree (pChan);
        return ERROR;
        }

    pResAdr = (VXB_RESOURCE_ADR *)pRes->pRes;

    if (pResAdr == NULL)
        {
        vxbMemFree (pChan);
        return ERROR;
        }

    pChan->adrRes = pRes;
    pChan->handle = pResAdr->pHandle;
    pChan->bar = (ULONG)pResAdr->virtual;

    pRes = vxbResourceAlloc(pDev, VXB_RES_IRQ, 0);

    if (pRes == NULL)
        {
        (void) vxbResourceFree(pDev, pChan->adrRes);
        vxbMemFree (pChan);
        return ERROR;
        }

    pChan->intRes = pRes;

    if (vxbIntConnect(pDev, pChan->intRes, tiAm3SiovxbInt, pDev) != OK)
        goto DeleteChan;

    if (vxbIntEnable(pDev,  pChan->intRes) != OK)
        goto DeleteChan;

    if (tiAm3SioCreate(pChan) == OK)
        return OK;

    /*
     * Note, the TTY dev is not created until the last thing in
     * tiAm3SioCreate(), and if the TTY device creation failed,
     * the TTY device need not be deleted.
     */

DeleteChan:

    (void) vxbResourceFree(pDev, pChan->adrRes);
    (void) vxbResourceFree(pDev, pChan->intRes);
    vxbMemFree (pChan);

    return ERROR;
    }

/******************************************************************************
*
* tiAm3SioCreate - create tiAm3Sio-compatible channel device
*
* This is the tiAm3Siovxb initialization routine.
*
* RETURNS: OK or ERROR when initialize failed
*
* ERRNO: N/A
*/

LOCAL STATUS tiAm3SioCreate
    (
    AM3_SIO_CHAN *  pChan
    )
    {
    STATUS stat;

    /* get the channel number */

    pChan->channelNo = vxbSioNextGet();

    /* fetch local copies of IER, LCR, and MCR */

    REG_GET(IER, pChan, pChan->ier);

    REG_GET(LCR, pChan, pChan->lcr);
    REG_GET(MCR, pChan, pChan->mcr);

    /* add ier initial bits in case the bits are cleared at reset */

    pChan->ier |= pChan->ierInit;

#ifndef    _VXBUS_BASIC_SIOPOLL
    pChan->isrDefRd.func = tiAm3SiovxbIntRd2;
    pChan->isrDefRd.pData = pChan;

    pChan->isrDefWr.func = tiAm3SiovxbIntWr2;
    pChan->isrDefWr.pData = pChan;
#endif    /* _VXBUS_BASIC_SIOPOLL */

    /* Initialize spinlock. */

    SPIN_LOCK_ISR_INIT (&pChan->spinlockIsr, 0);

    tiAm3SiovxbDevInit(pChan);

    VXB_AM3_SIO_SPIN_LOCK_READY;

#ifdef _WRS_CONFIG_CORE__IO_V2

    stat = usrTtyDevCreate (TTY_DEV_ONBOARD, &pChan->ttyIndex,
                            pChan->tyName, (SIO_CHAN *)pChan,
                            VXB_SIO_RX_BUF_SIZE, VXB_SIO_TX_BUF_SIZE,
                            pChan->channelNo);
#else

    pChan->ttyIndex = pChan->channelNo;
    (void) snprintf (pChan->tyName, sizeof (pChan->tyName),
                     "/tyCo/%d", pChan->channelNo);

    stat = ttyDevCreate (pChan->tyName, (SIO_CHAN *)pChan,
                         VXB_SIO_RX_BUF_SIZE, VXB_SIO_TX_BUF_SIZE);

#endif /* _WRS_CONFIG_CORE__IO_V2 */

    return stat;
    }

/******************************************************************************
*
* tiAm3SioShutdown - shutdown tiAm3Siovxb device
*
* This routine shoutdowns the tiAm3Sio device
*
* RETURNS: OK or ERROR when shutdown failed
*
* ERRNO
*/

LOCAL STATUS tiAm3SioShutdown
    (
    struct vxbDev * pDev
    )
    {
    AM3_SIO_CHAN * pChan;

    pChan = (AM3_SIO_CHAN *)vxbDevSoftcGet(pDev);

    while ( pChan != NULL )
        {
        (void) vxbIntDisable(pDev,  pChan->intRes);
        REG_SET(MDR1, pChan, MDR_DIS);
        pChan = pChan->pNext;
        }

    return OK;
    }

/******************************************************************************
*
* tiAm3SioDetach - detach tiAm3Siovxb device
*
* This routine detach the tiAm3Sio driver from the device
*
* RETURNS: OK or ERROR when detach failed
*
* ERRNO
*/

LOCAL STATUS tiAm3SioDetach
    (
    struct vxbDev * pDev
    )
    {
    AM3_SIO_CHAN * pChan, * pTmpChan;

    pChan = (AM3_SIO_CHAN *)vxbDevSoftcGet(pDev);

    while ( pChan != NULL )
        {
        if (pChan->adrRes != NULL)
            (void) vxbResourceFree(pDev, pChan->adrRes);

        if (pChan->intRes != NULL)
            {
            (void) vxbIntDisconnect(pDev, pChan->intRes);
            (void) vxbResourceFree(pDev, pChan->intRes);
            }

        (void)ttyDevDelete(pChan->tyName);

        pTmpChan = pChan;

        pChan = pChan->pNext;

        vxbMemFree((char *)pTmpChan);
        }

    return OK;
    }

/******************************************************************************
*
* tiAm3SiovxbDummyCallback - dummy callback routine.
*
* This routine is a dummy callback routine.
*
* RETURNS: ERROR, always
*
* ERRNO
*/

LOCAL STATUS tiAm3SiovxbDummyCallback (void)
    {
    return(ERROR);
    }

/******************************************************************************
*
* tiAm3SiovxbDevInit - initialize an AM3_SIO channel
*
* This routine initializes some SIO_CHAN function pointers and then resets
* the chip in a quiescent state.  Before this routine is called, the BSP
* must already have initialized all the device addresses, etc. in the
* AM3_SIO_CHAN structure.
*
* RETURNS: N/A
*
* ERRNO
*/

LOCAL void tiAm3SiovxbDevInit
    (
    AM3_SIO_CHAN * pChan    /* pointer to channel */
    )
    {
    /* set the non BSP-specific constants */

    pChan->getTxChar    = tiAm3SiovxbDummyCallback;
    pChan->putRcvChar   = tiAm3SiovxbDummyCallback;
    pChan->channelMode  = 0;    /* undefined */
    pChan->options      = (CLOCAL | CREAD | CS8);
    pChan->mcr          = MCR_OUT2;

    /* reset the chip */

    tiAm3SiovxbInitChannel (pChan);

    /* initialize the driver function pointers in the SIO_CHAN's */

    pChan->pDrvFuncs    = &tiAm3SiovxbSioDrvFuncs;
    }

/*******************************************************************************
*
* tiAm3SiovxbInitChannel - initialize UART
*
* Initialize the number of data bits, parity and set the selected
* baud rate. Set the modem control signals if the option is selected.
*
* RETURNS: N/A
*
* ERRNO
*/

LOCAL void tiAm3SiovxbInitChannel
    (
    AM3_SIO_CHAN * pChan    /* pointer to channel */
    )
    {
    UINT8 value;

    /* enable FIFO */

    REG_GET(LCR, pChan, pChan->lcr); /* 1a. */
    REG_SET(LCR, pChan, 0xBF); /* 1b. */
    REG_GET(EFR, pChan, value);
    REG_SET(EFR, pChan, value | EFR_ENHANCEDEN); /* 2. */
    REG_SET(LCR, pChan, 0x80); /* 3. */
    REG_GET(MCR, pChan, value);
    REG_SET(MCR, pChan, value | MCR_TCRTLR); /* 4. */
    value = (FCR_DMA | RxCLEAR | TxCLEAR | FIFO_ENABLE);
    REG_SET(FCR, pChan, value); /* 5 */
    REG_SET(LCR, pChan, 0xBF); /* 6. */
    REG_SET(TLR, pChan,
            (UINT8)(pChan->txTrgLvl | (pChan->rxTrgLvl << 4))); /* 7. */
    REG_SET(LCR, pChan, 0x80); /* 10. */
    REG_SET(LCR, pChan, pChan->lcr); /* 12. */

    /* set the requested baud rate */

    (void) tiAm3SiovxbBaudSet(pChan, pChan->baudRate);

    /* set the options */

    (void) tiAm3SiovxbOptsSet(pChan, pChan->options);
    }


/*******************************************************************************
*
* tiAm3SiovxbOptsSet - set the serial options
*
* Set the channel operating mode to that specified.  All sioLib options
* are supported: CLOCAL, HUPCL, CREAD, CSIZE, PARENB, and PARODD.
* When the HUPCL option is enabled, a connection is closed on the last
* close() call and opened on each open() call.
*
* NOTE:
*
* This routine disables the transmitter.  The calling routine
* may have to re-enable it.
*
* RETURNS: OK, or ERROR
*
* ERRNO
*/

LOCAL STATUS tiAm3SiovxbOptsSet
    (
    AM3_SIO_CHAN * pChan,   /* pointer to channel */
    UINT options            /* new hardware options */
    )
    {
    UINT8       value;
    UINT8       rxTrigHlt = 0;

    if (pChan == NULL || options & 0xffffff00)
    return ERROR;

    VXB_AM3_SIO_SPIN_LOCK_ISR_TAKE(pChan);

    /* clear RTS and DTR bits */

    pChan->mcr = (UINT8) (pChan->mcr & ~(MCR_RTS | MCR_DTR));

    switch (options & CSIZE)
        {
        case CS5:
            pChan->lcr = CHAR_LEN_5; break;
        case CS6:
            pChan->lcr = CHAR_LEN_6; break;
        case CS7:
            pChan->lcr = CHAR_LEN_7; break;
        case CS8:
        default:
            pChan->lcr = CHAR_LEN_8; break;
        }

    if (options & STOPB)
        pChan->lcr |= LCR_STB;
    else
        pChan->lcr |= ONE_STOP;

    switch (options & (PARENB | PARODD))
        {
        case PARENB|PARODD:
            pChan->lcr |= LCR_PEN; break;
        case PARENB:
            pChan->lcr |= (LCR_PEN | LCR_EPS); break;
        default:
        case 0:
            pChan->lcr |= PARITY_NONE; break;
        }

    REG_SET(IER, pChan, pChan->ierInit);

    /* check FIFO support if fifoSize is unknown  */

    if (pChan->fifoSize == AM3_SIO_FIFO_DEPTH_UNKNOWN)
        {
        /* test if FIFO is supported on the device */

        REG_GET(IIR, pChan, value);

        if ((value & (FCR_RXTRIG_L|FCR_RXTRIG_H)) ==
                            (FCR_RXTRIG_L|FCR_RXTRIG_H))
            {
            /*
             * FIFO is supported.
             * The max TX FIFO supported on this driver is 16 bytes, unless
             * BSP configures the depth through "fifoLen" parameter explicitly.
             */

            pChan->fifoSize = AM3_SIO_FIFO_DEPTH_DEFAULT;
            }
        else
            {
            /* No FIFO support */

            pChan->fifoSize = AM3_SIO_FIFO_DEPTH_NO;
            }
        }

    if (!(options & CLOCAL))
        {
        /* !clocal enables hardware flow control(DTR/DSR) */

        pChan->mcr = (UINT8) (pChan->mcr | (MCR_DTR | MCR_RTS));
        pChan->ier = (UINT8) (pChan->ier & (~TxFIFO_BIT));
        pChan->ier |= IER_EMSI;    /* enable modem status interrupt */

        REG_SET(LCR, pChan, 0x80); /* 1. */
        REG_SET(MCR, pChan, pChan->mcr | MCR_TCRTLR); /* 2. */
        REG_SET(LCR, pChan, 0xBF); /* 3. */
        REG_GET(EFR, pChan, value);
        REG_SET(EFR, pChan, value | EFR_ENHANCEDEN); /* 4. */
        if (pChan->fifoSize >= 2)
            {
            rxTrigHlt = (UINT8)(pChan->fifoSize - 1);
            }
        else
            {
            rxTrigHlt = 1;
            }
        REG_SET(TCR, pChan,
                (UINT8)(((rxTrigHlt - 1) << 4) | rxTrigHlt)); /* 5. */
        REG_SET(EFR, pChan, value | EFR_ENHANCEDEN | EFR_AUTO_CTS_EN | \
                            EFR_AUTO_RTS_EN); /* 6. */
        REG_SET(EFR, pChan, value | EFR_AUTO_CTS_EN | \
                            EFR_AUTO_RTS_EN); /* 6. Restore */
        REG_SET(LCR, pChan, 0x80); /* 7. */
        }
    else
        {
        /* disable modem status interrupt */

        pChan->ier = (UINT8) (pChan->ier & ~IER_EMSI);

        REG_SET(LCR, pChan, 0x80); /* 1. */
        REG_SET(MCR, pChan, pChan->mcr | MCR_TCRTLR); /* 2. */
        REG_SET(LCR, pChan, 0xBF); /* 3. */
        REG_GET(EFR, pChan, value);
        REG_SET(EFR, pChan, value | EFR_ENHANCEDEN); /* 4. */
        if (pChan->fifoSize >= 2)
            {
            rxTrigHlt = (UINT8)(pChan->fifoSize - 1);
            }
        else
            {
            rxTrigHlt = 1;
            }
        REG_SET(TCR, pChan, (UINT8)((rxTrigHlt - 1) << 4) | rxTrigHlt); /* 5. */
        REG_SET(EFR, pChan, (value | EFR_ENHANCEDEN) & (~EFR_AUTO_CTS_EN) & \
                            (~EFR_AUTO_RTS_EN)); /* 6. */
        REG_SET(EFR, pChan, value & (~EFR_AUTO_CTS_EN) & \
                            (~EFR_AUTO_RTS_EN)); /* 6. Restore */
        REG_SET(LCR, pChan, 0x80); /* 7. */
        }

    REG_SET(MCR, pChan, pChan->mcr); /* 8. */
    REG_SET(LCR, pChan, pChan->lcr); /* 9. */

    /* now reset the channel mode registers */

    value = (RxCLEAR | TxCLEAR | FIFO_ENABLE | FCR_RXTRIG_L | FCR_RXTRIG_H);
    REG_SET(FCR, pChan, value);

    /* enable FIFO in DMA mode 1 */
    value = (FCR_DMA | RxCLEAR | TxCLEAR | FIFO_ENABLE);
    REG_SET(FCR, pChan, value);

    if (options & CREAD)
        pChan->ier |= RxFIFO_BIT;

    if (pChan->channelMode == SIO_MODE_INT)
        {
        REG_SET(IER, pChan, pChan->ier);
        }

    pChan->options = options;

    VXB_AM3_SIO_SPIN_LOCK_ISR_GIVE(pChan);

    return OK;
    }

#ifndef    _VXBUS_BASIC_SIOPOLL
/*******************************************************************************
*
* tiAm3SiovxbHup - hang up the modem control lines
*
* Resets the RTS and DTR signals and clears both the receiver and
* transmitter sections.
*
* RETURNS: OK
*
* ERRNO
*/

LOCAL STATUS tiAm3SiovxbHup
    (
    AM3_SIO_CHAN * pChan    /* pointer to channel */
    )
    {
    UINT8 value = 0;

    VXB_AM3_SIO_SPIN_LOCK_ISR_TAKE(pChan);

    pChan->mcr = (UINT8) (pChan->mcr & ~(MCR_RTS | MCR_DTR));
    REG_SET(MCR, pChan, pChan->mcr);
    value = (RxCLEAR | TxCLEAR);
    REG_SET(FCR, pChan, value);

    VXB_AM3_SIO_SPIN_LOCK_ISR_GIVE(pChan);

    return(OK);
    }


/*******************************************************************************
*
* tiAm3SiovxbOpen - Set the modem control lines
*
* Set the modem control lines(RTS, DTR) TRUE if not already set.
* It also clears the receiver, transmitter and enables the FIFO.
*
* RETURNS: OK
*
* ERRNO
*/

LOCAL STATUS tiAm3SiovxbOpen
    (
    AM3_SIO_CHAN * pChan    /* pointer to channel */
    )
    {
    UINT8   mask;
    UINT8   value = 0;

    REG_GET(MCR, pChan, mask);
    mask = (UINT8) (mask & (MCR_RTS | MCR_DTR));

    if (mask != (MCR_RTS | MCR_DTR))
        {
        /* RTS and DTR not set yet */

        VXB_AM3_SIO_SPIN_LOCK_ISR_TAKE(pChan);

        /* set RTS and DTR TRUE */

        pChan->mcr |= (MCR_DTR | MCR_RTS);
        REG_SET(MCR, pChan, pChan->mcr);

        /* clear Tx and receive and enable FIFO */
        value = (FCR_DMA | RxCLEAR | TxCLEAR | FIFO_ENABLE);
        REG_SET(FCR, pChan, value);

        VXB_AM3_SIO_SPIN_LOCK_ISR_GIVE(pChan);
        }

    return(OK);
    }
#endif    /* _VXBUS_BASIC_SIOPOLL */

/******************************************************************************
*
* tiAm3SiovxbBaudSet - change baud rate for channel
*
* This routine sets the baud rate for the UART. The interrupts are disabled
* during chip access.
*
* RETURNS: OK always.
*
* ERRNO
*/

LOCAL STATUS  tiAm3SiovxbBaudSet
    (
    AM3_SIO_CHAN * pChan,   /* pointer to channel */
    UINT           baud     /* requested baud rate */
    )
    {
    int     divisor;
    UINT8   value;
    UINT8   opMode;

    VXB_ASSERT(pChan->pDev != NULL, ERROR)

    if (pChan->xtal == AM3_CLK_48_MHZ)
        {

        /* baud rate 3Mbps only for ti_sitara_ctxa15 */

        if ((baud <= AM3_BAUD_230_4_KBPS) || (baud == AM3_BAUD_3_MBPS))
            {
            opMode = 16;
            }
        else
            {
            opMode = 13;
            }
        }

    /* 192Mhz clock, only for ti_sitara_ctxa15 */

    else
        {
        if ((baud <= AM3_BAUD_921_6_KBPS) || (baud == AM3_BAUD_3_MBPS) ||
            (baud == AM3_BAUD_12_MBPS))
            {
            opMode = 16;
            }
        else
            {
            opMode = 13;
            }
        }

    divisor = (int)(pChan->xtal / (opMode * baud));

    if (opMode == 16)
        {
        REG_SET(MDR1, pChan, AM3_OP_MOD_16);
        }
    else
        {
        REG_SET(MDR1, pChan, AM3_OP_MOD_13);
        }

    AM3_SIO_DBG_MSG(500,"tiAm3SiovxbBaudSet(0x%x, %d), divisor = %d / 0x%08x\n",
                    (_Vx_usr_arg_t)pChan,baud,divisor,divisor,5,6);

    /* disable interrupts during chip access */

    VXB_AM3_SIO_SPIN_LOCK_ISR_TAKE(pChan);

    /* Enable access to the divisor latches by setting DLAB in LCR. */

    value = (LCR_DLAB | pChan->lcr);
    REG_SET(LCR, pChan, value);

    /* Set divisor latches. */

    value = (UINT8)divisor;
    REG_SET(DLL, pChan, value);
    value = (UINT8) (divisor >> 8);
    REG_SET(DLM, pChan, value);

    /* Restore line control register */

    REG_SET(LCR, pChan, pChan->lcr);

    pChan->baudRate = baud;

    VXB_AM3_SIO_SPIN_LOCK_ISR_GIVE(pChan);

    return(OK);
    }

#ifndef    _VXBUS_BASIC_SIOPOLL
/*******************************************************************************
*
* tiAm3SiovxbModeSet - change channel mode setting
*
* This driver supports both polled and interrupt modes and is capable of
* switching between modes dynamically.
*
* If interrupt mode is desired this routine enables the channels receiver and
* transmitter interrupts. If the modem control option is TRUE, the Tx interrupt
* is disabled if the CTS signal is FALSE. It is enabled otherwise.
*
* If polled mode is desired the device interrupts are disabled.
*
* RETURNS: OK, or ERROR
*/

LOCAL STATUS tiAm3SiovxbModeSet
    (
    AM3_SIO_CHAN * pChan,   /* pointer to channel */
    UINT    newMode         /* mode requested */
    )
    {
    UINT8   mask;
    BOOL    spinLockOwn;

    if ((newMode != SIO_MODE_POLL) && (newMode != SIO_MODE_INT))
        return(ERROR);

    /* test if spinlock is already owned by the currect CPU core */

    spinLockOwn = VXB_AM3_SIO_SPIN_LOCK_ISR_HELD(pChan);

    if (newMode == SIO_MODE_INT)
        {
        /* set the pointer of the  queue for deferred work */

        if ((pChan->queueId == NULL) && ((pChan->flag & DISABLE_DEFER) == 0))
            pChan->queueId = isrDeferQueueGet (pChan->pDev, 0, 0, 0);

        if (!spinLockOwn)
            {
            VXB_AM3_SIO_SPIN_LOCK_ISR_TAKE(pChan);
            }

        /* Enable appropriate interrupts */

        if (pChan->options & CLOCAL)
            {
            UINT8 ierLocal = (pChan->ier | RxFIFO_BIT | TxFIFO_BIT);
            REG_SET(IER, pChan, ierLocal);
            }
        else
            {
            REG_GET(MSR, pChan, mask);
            mask = (UINT8) (mask & MSR_CTS);

            /* if the CTS is asserted enable Tx interrupt */

            if (mask & MSR_CTS)
            pChan->ier |= TxFIFO_BIT;    /* enable Tx interrupt */
            else
            {
            /* disable Tx interrupt */

            pChan->ier = (UINT8) (pChan->ier & (~TxFIFO_BIT));
            }

            REG_SET(IER, pChan, pChan->ier);
            }
        }
    else
        {
        if (!spinLockOwn)
            {
            VXB_AM3_SIO_SPIN_LOCK_ISR_TAKE(pChan);
            }

        /* disable all tiAm3Siovxb interrupts */
        REG_SET(IER, pChan, pChan->ierInit);
        }

    pChan->channelMode = (UINT16) newMode;

    if (!spinLockOwn)
        {
        VXB_AM3_SIO_SPIN_LOCK_ISR_GIVE(pChan);
        }

    return(OK);
    }

/*******************************************************************************
*
* tiAm3SiovxbIoctl - special device control
*
* Includes commands to get/set baud rate, mode (INT,POLL), hardware options
* (parity, number of data bits), and modem control (RTS/CTS and DTR/DSR).
* The ioctl command SIO_HUP is sent by ttyDrv when the last close() function
* call is made. Likewise SIO_OPEN is sent when the first open() function call
* is made.
*
* RETURNS: OK on success
*
* ERRNO: EIO on device error, ENOSYS on unsupported request
*/

LOCAL STATUS tiAm3SiovxbIoctl
    (
    AM3_SIO_CHAN *  pChan,  /* pointer to channel */
    int             request,    /* request code */
    void *          pArg        /* some argument */
    )
    {
    FAST STATUS  status;
    long arg = (long)pArg;

    status = OK;

    switch (request)
        {
        case SIO_BAUD_SET:
            if (pChan->xtal == AM3_CLK_48_MHZ)
                {
                if (arg < AM3_SIO_MIN_RATE || arg > AM3_SIO_MAX_RATE)
                    status = EIO;       /* baud rate out of range */
                else
                    status = tiAm3SiovxbBaudSet (pChan, (UINT)arg);
                }
            else
                {
                if (arg < AM5_SIO_MIN_RATE || arg > AM5_SIO_MAX_RATE)
                    status = EIO;       /* baud rate out of range */
                else
                    status = tiAm3SiovxbBaudSet (pChan, (UINT)arg);
                }
            break;

        case SIO_BAUD_GET:
            *(int *) pArg = (int) pChan->baudRate;
            break;

        case SIO_MODE_SET:
            status = (tiAm3SiovxbModeSet (pChan, (UINT)arg) == OK) ? OK : EIO;
            break;

        case SIO_MODE_GET:
            *(int *) pArg = (int) pChan->channelMode;
            break;

        case SIO_AVAIL_MODES_GET:
            *(int *)pArg = SIO_MODE_INT | SIO_MODE_POLL;
            break;

        case SIO_HW_OPTS_SET:
            status = (tiAm3SiovxbOptsSet (pChan, (UINT)arg) == OK) ? OK : EIO;
            break;

        case SIO_HW_OPTS_GET:
            *(int *) pArg = (int) pChan->options;
            break;

        case SIO_HUP:
            /* check if hupcl option is enabled */

            if (pChan->options & HUPCL)
            status = tiAm3SiovxbHup (pChan);
            break;

        case SIO_OPEN:
            /* check if hupcl option is enabled */

            if (pChan->options & HUPCL)
            status = tiAm3SiovxbOpen (pChan);
            break;

        default:
            status = ENOSYS;
        }

    return(status);
    }

/*******************************************************************************
*
* tiAm3SiovxbIntWr2 - deferred handle a transmitter interrupt
*
* This routine handles write interrupts from the UART. It reads a character
* and puts it in the transmit holding register of the device for transfer.
*
* If there are no more characters to transmit, transmission is disabled by
* clearing the transmit interrupt enable bit in the IER(int enable register).
*
* RETURNS: N/A
*
* ERRNO
*/

LOCAL void tiAm3SiovxbIntWr2
    (
    void * pData        /* pointer to channel */
    )
    {
    AM3_SIO_CHAN * pChan = (AM3_SIO_CHAN *) pData;
    UINT8    status;
    char    outChar;

    VXB_AM3_SIO_SPIN_LOCK_ISR_TAKE(pChan);

    /*
     * Clear the txDefer.
     * Another deferred job can be posted before completing the deferred job
     * since the txDefer is cleared prior to start the getTxChar.
     * But the job queue is already cleared before entering into
     * tiAm3SiovxbIntWr2() and the maximum required job queue number for
     * transmit is one.
     */

    pChan->txDefer = FALSE;

    VXB_AM3_SIO_SPIN_LOCK_ISR_GIVE(pChan);

    /* Transmit characters upto the FIFO depth */

    REG_GET(SSR, pChan, status);

    /*
     * Re-take the spinlock to mutual exclude the TX Reg
     * access between the kprintf() in SMP mode.
     */

    VXB_AM3_SIO_ISR_SET(pChan);
    while ((status & SSR_TXFIFOFULL) != SSR_TXFIFOFULL)
        {
        if ((*pChan->getTxChar) (pChan->getTxArg, &outChar) != ERROR)
            {

            /* write char to Transmit Holding Reg */

            REG_SET(THR, pChan, outChar);
            REG_GET(SSR, pChan, status);
            }
        else
            {
            VXB_AM3_SIO_ISR_CLEAR(pChan);
            return;
            }
        }
    VXB_AM3_SIO_ISR_CLEAR(pChan);

    VXB_AM3_SIO_SPIN_LOCK_ISR_TAKE(pChan);
    if (pChan->options & CLOCAL)
        {
        pChan->ier |= (TxFIFO_BIT);    /* indicates to enable Tx Int */
        REG_SET(IER, pChan, pChan->ier);
        }
    else
        {
        REG_GET(MSR, pChan, status);

        /* only enable TX Int when CTS is active */

        if (status & MSR_CTS)
            {
            pChan->ier |= (TxFIFO_BIT);
            REG_SET(IER, pChan, pChan->ier);
            }
        }
    VXB_AM3_SIO_SPIN_LOCK_ISR_GIVE(pChan);
    }

/*******************************************************************************
*
* tiAm3SiovxbIntRd2 - deferred handle a receiver interrupt
*
* This routine handles read interrupts from the UART.
*
* RETURNS: N/A
*
* ERRNO
*/

LOCAL void tiAm3SiovxbIntRd2
    (
    void * pData        /* pointer to channel */
    )
    {
    AM3_SIO_CHAN * pChan = (AM3_SIO_CHAN *) pData;
    char   inchar;
    char * pChar = &inchar;

    while (TRUE)
        {
        VXB_AM3_SIO_SPIN_LOCK_ISR_TAKE(pChan);

        if ((tiAm3SiovxbPollInput (pChan, pChar)) == OK)
            {
            VXB_AM3_SIO_SPIN_LOCK_ISR_GIVE(pChan);

            (*pChan->putRcvChar) (pChan->putRcvArg, inchar);
            }
        else
            break;
        }

    /*
     * Another deferred job can be posted before completing the deferred job
     * if further Rx interrupt is raised before completing tiAm3SiovxbIntRd2().
     * But the job queue is already cleared before entering into
     * this routine and the maximum required job queue number for reception
     * is one.
     */

    pChan->ier |= RxFIFO_BIT;        /* indicates to enable Rx Int */
    REG_SET(IER, pChan, pChan->ier);

    VXB_AM3_SIO_SPIN_LOCK_ISR_GIVE(pChan);
    }

/********************************************************************************
*
* tiAm3SiovxbInt - interrupt level processing
*
* This routine handles four sources of interrupts from the UART. They are
* prioritized in the following order by the Interrupt Identification Register:
* Receiver Line Status, Received Data Ready, Transmit Holding Register Empty
* and Modem Status.
*
* When a modem status interrupt occurs, the transmit interrupt is enabled if
* the CTS signal is TRUE.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void tiAm3SiovxbInt
    (
    VXB_DEV_ID pDev
    )
    {
    int count = AM3_SIO_INT_COUNT;
    FAST AM3_SIO_CHAN * pChan = (AM3_SIO_CHAN *)(vxbDevSoftcGet(pDev));
    FAST volatile char     intStatus;
    UINT8 iirValue, lsrValue;

    while (( pChan != NULL ) && count > 0)
        {
        VXB_AM3_SIO_ISR_SET(pChan);

        /* read the Interrupt Status Register (Int. Ident.) */

        REG_GET(IIR, pChan, iirValue);
        intStatus = (char)iirValue;
        intStatus = (UINT8) (intStatus & 0x0f);

        /*
         * This UART chip always produces level active interrupts,
         * and the IIR only indicates the highest priority interrupt.
         * In the case that receive and transmit interrupts happened at
         * the same time, we must clear both interrupt pending to keep the
         * edge-triggered interrupt (output from interrupt controller) from
         * locking up. One way of doing it is to disable all interrupts
         * at the beginning of the ISR and reenable them at the end.
         */

        REG_SET(IER, pChan, pChan->ierInit);

        switch (intStatus)
            {
            case IIR_RLS:
                /* overrun,parity error and break interrupt */

                REG_GET(LSR, pChan, lsrValue); /* read LSR to reset interrupt */

                /* coverity[assigned_value] */
                intStatus = (char)lsrValue;
                break;

            case IIR_RDA:           /* received data available */
            case IIR_TIMEOUT:
                {
                /*
                 * receiver FIFO interrupt. In some cases, IIR_RDA is
                 * not indicated in IIR register when there is more
                 * than one character in FIFO.
                 */

                /* indicate to disable Rx Int */

                pChan->ier = (UINT8) (pChan->ier & ~(RxFIFO_BIT));
                REG_SET(IER, pChan, pChan->ier);

                /*
                 * Another spinlock is taken in isrDeferJobAdd().
                 * So release spinlock before entering into isrDeferJobAdd().
                 */

                VXB_AM3_SIO_ISR_CLEAR(pChan);

                /* defer the job */

                if (pChan->queueId == NULL)
                    tiAm3SiovxbIntRd2(pChan);
                else
                    isrDeferJobAdd (pChan->queueId, &pChan->isrDefRd);

                goto nextChan;
                }

            case IIR_THRE:  /* transmitter holding register ready */

            /*
             * If Tx deferred job is still in queue, no need to add
             * another one and save the number of the job queue to prevent
             * from the queue overflow.
             */

            /* indicate to disable Tx Int */

            pChan->ier = (UINT8) (pChan->ier & ~(TxFIFO_BIT));

            if (!pChan->txDefer)
                {
                /* defer the job */

                pChan->txDefer = TRUE;

                REG_SET(IER, pChan, pChan->ier);

                /*
                 * Another spinlock is taken in isrDeferJobAdd ().
                 * So release spinlock before entering into
                 * isrDeferJobAdd().
                 */

                VXB_AM3_SIO_ISR_CLEAR(pChan);

                if (pChan->queueId == NULL)
                    tiAm3SiovxbIntWr2(pChan);
                else
                    isrDeferJobAdd (pChan->queueId, &pChan->isrDefWr);

                goto nextChan;
                }

            break;

            case IIR_MSTAT: /* modem status register */
                {
                UINT8    msr;

                REG_GET(MSR, pChan, msr);

#define SPR_74889_FIX
#ifdef  SPR_74889_FIX

                /*
                 * If the CTS line changed, enable or
                 * disable Tx interrupt accordingly.
                 */
                if (msr & MSR_DCTS)
                    {
                    if (msr & MSR_CTS)
                         /* CTS was turned on */
                        pChan->ier |= TxFIFO_BIT;
                    else
                        /* CTS was turned off */
                        pChan->ier = (UINT8) (pChan->ier & (~TxFIFO_BIT));
                    }
#else   /* SPR_74889_FIX */
                /* if CTS is asserted by modem, enable Tx interrupt */

                if ((msr & MSR_CTS) && (msr & MSR_DCTS))
                    pChan->ier |= TxFIFO_BIT;
                else
                    pChan->ier &= (~TxFIFO_BIT);
#endif  /* SPR_74889_FIX */
                }
            break;

            default:
            break;
            }

        REG_SET(IER, pChan, pChan->ier); /* enable interrupts accordingly */

        VXB_AM3_SIO_ISR_CLEAR(pChan);

nextChan:

        if (--count == 0)
            pChan = pChan->pNext;
        }
    }

/*******************************************************************************
*
* tiAm3SiovxbTxStartup - transmitter startup routine
*
* Call interrupt level character output routine and enable interrupt if it is
* in interrupt mode with no hardware flow control.
* If the option for hardware flow control is enabled and CTS is set TRUE,
* then the Tx interrupt is enabled.
*
* RETURNS: OK
*
* ERRNO: ENOSYS if in polled mode
*/

LOCAL int tiAm3SiovxbTxStartup
    (
    AM3_SIO_CHAN * pChan    /* pointer to channel */
    )
    {
    UINT8 mask;
    BOOL  spinLockOwn;
    char  outChar;

    AM3_SIO_DBG_MSG(500,"tiAm3SiovxbTxStartup(0x%x)\n",
            (_Vx_usr_arg_t)pChan,2,3,4,5,6);

    /* test if spinlock is already owned by the currect CPU core */

    spinLockOwn = VXB_AM3_SIO_SPIN_LOCK_ISR_HELD(pChan);

    if (pChan->channelMode == SIO_MODE_INT)
        {
        if (!spinLockOwn)
            {
            VXB_AM3_SIO_SPIN_LOCK_ISR_TAKE(pChan);
            }

        if (pChan->options & CLOCAL)
            {
            /* No modem control */

            /*
             * If there is a job queue already posted for transmit, not
             * necessary to enable Tx interrupt.
             */

            if (!pChan->txDefer)
                {
                if (!spinLockOwn)
                    {
                    VXB_AM3_SIO_SPIN_LOCK_ISR_GIVE(pChan);
                    }

                if ((*pChan->getTxChar) (pChan->getTxArg, &outChar) != ERROR)
                    {
                  if (!spinLockOwn)
                        {
                        VXB_AM3_SIO_SPIN_LOCK_ISR_TAKE(pChan);
                        }

                     /* write char to Transmit Holding Reg */

                     REG_SET(THR, pChan, outChar);
                    }
                else
                    {
                    return (ENOSYS);
                    }
                pChan->ier |= TxFIFO_BIT;
                }
            }
        else
            {
            REG_GET(MSR, pChan, mask);
            mask = (UINT8) (mask & MSR_CTS);

            /* if the CTS is asserted enable Tx interrupt */

            if (mask & MSR_CTS)
                {
                /*
                 * If there is a job queue already posted for transmit, not
                 * necessary to enable Tx interrupt.
                 */

                if (!pChan->txDefer)
                    pChan->ier |= TxFIFO_BIT;    /* enable Tx interrupt */
                }
            else
                {
                /* disable Tx interrupt */

                pChan->ier = (UINT8) (pChan->ier & (~TxFIFO_BIT));
                }
            }

        REG_SET(IER, pChan, pChan->ier);

        if (!spinLockOwn)
            {
            VXB_AM3_SIO_SPIN_LOCK_ISR_GIVE(pChan);
            }

        AM3_SIO_DBG_MSG(500,"tiAm3SiovxbTxStartup(0x%x): started\n",
                (_Vx_usr_arg_t)pChan,2,3,4,5,6);
        return(OK);
        }
    else
        {
        AM3_SIO_DBG_MSG(500,"tiAm3SiovxbTxStartup(0x%x): ENOSYS\n",
                (_Vx_usr_arg_t)pChan,2,3,4,5,6);
        return(ENOSYS);
        }
    }
#endif    /* _VXBUS_BASIC_SIOPOLL */

/******************************************************************************
*
* tiAm3SiovxbPollOutput - output a character in polled mode
*
* This routine transmits a character in polled mode.
*
* RETURNS: OK if a character sent
*
* ERRNO: EAGAIN if the output buffer is full
*/

LOCAL int tiAm3SiovxbPollOutput
    (
    AM3_SIO_CHAN *  pChan,  /* pointer to channel */
    char            outChar /* char to send */
    )
    {
    UINT8 pollStatus;
    UINT8 msr;

    AM3_SIO_DBG_MSG(500,"tiAm3SiovxbPollOutput(0x%x, '%c')\n",
                (_Vx_usr_arg_t)pChan,outChar,3,4,5,6);

    REG_GET(LSR, pChan, pollStatus);
    REG_GET(MSR, pChan, msr);

    /* is the transmitter ready to accept a character? */

    while ((pollStatus & LSR_THRE) == 0x00)
        {
        AM3_SIO_DBG_MSG(500,"tiAm3SiovxbPollOutput(): not ready\n",
                1,2,3,4,5,6);
        REG_GET(LSR, pChan, pollStatus);
        }

    /* modem flow control ? */

    if ((!(pChan->options & CLOCAL)) && !(msr & MSR_CTS))
        {
        AM3_SIO_DBG_MSG(500,"tiAm3SiovxbPollOutput(): flow control\n",
                1,2,3,4,5,6);
        return(EAGAIN);
        }

    REG_SET(THR, pChan, outChar);       /* transmit character */

    AM3_SIO_DBG_MSG(500,"tiAm3SiovxbPollOutput(0x%x, '%c'): sent\n",
                (_Vx_usr_arg_t)pChan,outChar,3,4,5,6);

    return(OK);
    }

#ifndef    _VXBUS_BASIC_SIOPOLL
/******************************************************************************
*
* tiAm3SiovxbPollInput - poll for input
*
* This routine polls the device for input.
*
* RETURNS: OK if a character arrived
*
* ERRNO: EAGAIN if the input buffer if empty
*/

LOCAL int tiAm3SiovxbPollInput
    (
    AM3_SIO_CHAN *  pChan,  /* pointer to channel */
    char *          pChar   /* pointer to char */
    )
    {
    UINT8 pollStatus;
    UINT8 *inChar = (UINT8 *)pChar;

    AM3_SIO_DBG_MSG(500,"tiAm3SiovxbPollInput(0x%x, 0x%08x)\n",
            (_Vx_usr_arg_t)pChan,(_Vx_usr_arg_t)pChar,3,4,5,6);

    REG_GET(LSR, pChan, pollStatus);

    if ((pollStatus & LSR_DR) == 0x00)
        {
        AM3_SIO_DBG_MSG(500,"tiAm3SiovxbPollInput(): LSR_DR\n",
                1,2,3,4,5,6);
        return(EAGAIN);
        }

    /* got a character */

    REG_GET(RBR, pChan, *inChar);

    AM3_SIO_DBG_MSG(500,"tiAm3SiovxbPollInput(0x%x, 0x%08x): got '%c'\n",
            (_Vx_usr_arg_t)pChan,(_Vx_usr_arg_t)pChar,*inChar,4,5,6);

    return(OK);
    }

/******************************************************************************
*
* tiAm3SiovxbCallbackInstall - install ISR callbacks
*
* This routine installs the callback functions to get/put chars for the
* driver.
*
* RETURNS: OK on success
*
* ERRNO: ENOSYS on unsupported callback type
*/

LOCAL int tiAm3SiovxbCallbackInstall
    (
    SIO_CHAN *  pSioChan,       /* pointer to device to control */
    int         callbackType,   /* callback type(Tx or receive) */
    STATUS      (*callback)(),  /* pointer to callback function */
    void *      callbackArg     /* callback function argument */
    )
    {
    AM3_SIO_CHAN * pChan = (AM3_SIO_CHAN *)pSioChan;

    switch (callbackType)
        {
        case SIO_CALLBACK_GET_TX_CHAR:
            pChan->getTxChar    = callback;
            pChan->getTxArg     = callbackArg;
            return(OK);
        case SIO_CALLBACK_PUT_RCV_CHAR:
            pChan->putRcvChar   = callback;
            pChan->putRcvArg    = callbackArg;
            return(OK);
        default:
            return(ENOSYS);
        }
    }
#endif    /* _VXBUS_BASIC_SIOPOLL */

