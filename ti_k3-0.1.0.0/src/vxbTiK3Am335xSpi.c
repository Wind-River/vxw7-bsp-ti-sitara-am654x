/* vxbTiK3Am335xSpi.c - TiK3 AM335x SPI Controller Driver */

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

/*
DESCRIPTION

The TI AM335x SPI is a high-speed serial input/output port that allows
a serial bit stream of programmed length (4 to 32 bits) to be shifted into and
out of the device at a programmed bit-transfer rate. The SPI is normally used
for communication between the device and external peripherals. Typical
applications include interface to external I/O or peripheral expansion via
devices such as shift registers, display drivers, SPI EPROMS, SPI Flash and
analog-to-digital converters.

This module implements a driver for the SPI controller present on many
TI ARM processors. The controller is capable of acting as a master
mode. This driver supports working on polling, interrupt mode,
meanwhile, the driver can dynamically switch clock frequency, duplex mode and
chipSelect according to the device requirement. The driver unconditionally
supports the FIFO.

To add the driver to the vxWorks image, add the following component to the
kernel configuration:

\cs
vxprj component add DRV_AM35XX_SPI_TI_K3
\ce

SEE ALSO: vxBus, spiBus
INCLUDE FILES: vxbSpiLib.h vxbTiK3Am335xSpi.h
\tb "AM335x ARM Cortex-A8 Microprocessors (MPUs) Technical Reference Manual"
*/

/* includes */

#include <vxWorks.h>
#include <vsbConfig.h>
#include <semLib.h>
#include <hwif/vxBus.h>
#include <hwif/buslib/vxbFdtSpiLib.h>
#include <subsys/int/vxbIntLib.h>
#include <subsys/pinmux/vxbPinMuxLib.h>
#include <subsys/timer/vxbTimerLib.h>
#include <subsys/clk/vxbClkLib.h>
#include "vxbTiK3Am335xSpi.h"

/* defines */

/* debug macro */

#undef  SPI_DBG_ON
#ifdef  SPI_DBG_ON

/* turning local symbols into global symbols */

#define SPI_DBG_OFF            0x00000000
#define SPI_DBG_ISR            0x00000001
#define SPI_DBG_RW             0x00000002
#define SPI_DBG_ERR            0x00000004
#define SPI_DBG_RTN            0x00000008
#define SPI_DBG_ALL            0xffffffff

LOCAL UINT32 spiDbgMask = SPI_DBG_ERR;

#define SPI_DBG(mask, ...)                                              \
    do                                                                  \
        {                                                               \
        if ((spiDbgMask & mask) || (mask == SPI_DBG_ALL))               \
            {                                                           \
            if (_func_kprintf != NULL)                                  \
                {                                                       \
                (* _func_kprintf)(__VA_ARGS__);                         \
                }                                                       \
            }                                                           \
        }                                                               \
    while ((FALSE))
#else
#undef SPI_DBG
#define SPI_DBG(...)
#endif  /* SPI_DBG_ON */

/* forward declarations */

LOCAL STATUS    am335xSpiProbe (VXB_DEV_ID);
LOCAL STATUS    am335xSpiAttach (VXB_DEV_ID);
LOCAL STATUS    am335xSpiInit (VXB_DEV_ID pDev);
LOCAL void      am335xSpiPinMux(struct vxbDev * pDev);
LOCAL void      am335xSpiIsr (VXB_DEV_ID pDev);
LOCAL STATUS    am335xSpiChanCfg (VXB_DEV_ID pDev, SPI_HARDWARE * pSpiDev);
LOCAL STATUS    am335xSpiWaitBitSet (VXB_DEV_ID pDev, UINT32 regOffset, UINT32
                                     regBit);
LOCAL void      am335xSpiChanShutdown (VXB_DEV_ID pDev, UINT32 chan);
LOCAL STATUS    am335xSpiTrans (VXB_DEV_ID pDev, SPI_HARDWARE * devInfo,
                                SPI_TRANSFER * pPkg);
LOCAL STATUS    am335xSpiTransfer (VXB_DEV_ID pDev, SPI_HARDWARE * devInfo,
                                   SPI_TRANSFER * pPkg);

/* locals */

LOCAL VXB_DRV_METHOD am335xSpiMethods[] = {
    { VXB_DEVMETHOD_CALL (vxbDevProbe),  (FUNCPTR)am335xSpiProbe},
    { VXB_DEVMETHOD_CALL (vxbDevAttach), (FUNCPTR)am335xSpiAttach},
    { VXB_DEVMETHOD_CALL (vxbSpiXfer),   (FUNCPTR)am335xSpiTransfer},
    { VXB_DEVMETHOD_CALL (vxbFdtDevGet), (FUNCPTR)vxbSpiFdtDevGet},
    { VXB_DEVMETHOD_CALL (vxbResourceFree),  (FUNCPTR)vxbSpiResFree},
    { VXB_DEVMETHOD_CALL (vxbResourceAlloc), (FUNCPTR)vxbSpiResAlloc},
    VXB_DEVMETHOD_END
};

VXB_DRV vxbOfTiK3Am335xSpiDrv =
    {
    { NULL } ,
    TI_AM335X_SPI_NAME,                 /* Name */
    "TI AM335x SPI controller",         /* Description */
    VXB_BUSID_FDT,                      /* Class */
    0,                                  /* Flags */
    0,                                  /* Reference count */
    (VXB_DRV_METHOD*)&am335xSpiMethods  /* Method table */
    };

VXB_DRV_DEF (vxbOfTiK3Am335xSpiDrv)

LOCAL const VXB_FDT_DEV_MATCH_ENTRY am335xSpiMatch[] =
    {
        {
        "ti,am335x-spi", /* compatible */
        (void *)NULL
        },
        {} /* Empty terminated list */
    };

/******************************************************************************
*
* am335xSpiProbe - probe for device presence at specific address
*
* Check for SPI contoller (or compatible) device at the specified
* base address.  We assume one is present at that address, but we need to verify.
*
* RETURNS: OK if probe passes and assumed a valid am335x spi controller
* (or compatible) device.  ERROR otherwise.
* ERRNO: N/A
*/

LOCAL STATUS am335xSpiProbe
    (
    struct vxbDev * pDev
    )
    {
    return vxbFdtDevMatch (pDev, am335xSpiMatch, NULL);
    }

/*****************************************************************************
*
* am335xSpiAttach - attach am335x SPI controller
*
* This function initializes the device.
*
* RETURNS: OK or ERROR when failed.
*
* ERRNO: N/A
*/

LOCAL STATUS am335xSpiAttach
    (
    VXB_DEV_ID  pDev
    )
    {
    TI_AM335X_SPI_CTRL *    pDrvCtrl;
    VXB_RESOURCE_ADR *      pResAdr = NULL;
    VXB_RESOURCE *          pRes    = NULL;
    STATUS                  ret     = ERROR;

    if (pDev == NULL)
        {
        return ERROR;
        }

    /* create controller driver context structure for core */

    pDrvCtrl = (TI_AM335X_SPI_CTRL *) vxbMemAlloc (sizeof (TI_AM335X_SPI_CTRL));
    if (pDrvCtrl == NULL)
        {
        return ERROR;
        }

    /* save pDrvCtrl in VXB_DEVICE structure */

    vxbDevSoftcSet (pDev, pDrvCtrl);

    /* save instance ID */

    pDrvCtrl->pDev = pDev;

    pRes = vxbResourceAlloc (pDev, VXB_RES_MEMORY, 0);
    if (pRes == NULL)
        {
        goto attach_error;
        }

    pResAdr = (VXB_RESOURCE_ADR *)pRes->pRes;
    if (pResAdr == NULL)
        {
        goto attach_error;
        }

    pDrvCtrl->regHandle = pResAdr->pHandle;
    pDrvCtrl->regBase = (void *)pResAdr->virtual;

    pRes = vxbResourceAlloc (pDev, VXB_RES_MEMORY, 1);
    if (pRes != NULL)
        {
        pResAdr = (VXB_RESOURCE_ADR *)pRes->pRes;
        if (pResAdr == NULL)
            {
            goto attach_error;
            }

        pDrvCtrl->pinMuxRegHandle = pResAdr->pHandle;
        pDrvCtrl->pinMuxRegBase = (void *)pResAdr->virtual;
        }

    pDrvCtrl->intRes = vxbResourceAlloc (pDev, VXB_RES_IRQ, 0);

    pDrvCtrl->clkFrequency = AM335X_SPI_CLK_DEFAULT;

    /* SPI PinMux init */

    am335xSpiPinMux (pDev);

    /* SPI controller init */

    if (am335xSpiInit (pDev) != OK)
        {
        SPI_DBG (SPI_DBG_ERR, "SPI initialize failed\n");
        goto attach_error;
        }

    /* connect and enable interrupt for non-poll mode */

    if (pDrvCtrl->intRes != NULL)
        {
        /* The semSync semaphore is used to synchronize the SPI transfer. */

        pDrvCtrl->semSync = semBCreate (SEM_Q_PRIORITY, SEM_EMPTY);
        if (pDrvCtrl->semSync == SEM_ID_NULL)
            {
            SPI_DBG (SPI_DBG_ERR, "semBCreate failed for semSync\n");
            goto attach_error;
            }

        /* enable and connect ISR */

        ret = vxbIntConnect (pDev, pDrvCtrl->intRes, (VOIDFUNCPTR)am335xSpiIsr,
                             pDev);
        if (ret != OK)
            {
            goto attach_error;
            }

        ret = vxbIntEnable (pDev, pDrvCtrl->intRes);
        if (ret != OK)
            {
            (void)vxbIntDisconnect (pDev, pDrvCtrl->intRes);
            goto attach_error;
            }
        }

    if (vxbSpiCtrlRegister (pDev) != OK)
        {
        if (pDrvCtrl->intRes != NULL)
            {
            (void)vxbIntDisable (pDev, pDrvCtrl->intRes);
            (void)vxbIntDisconnect (pDev, pDrvCtrl->intRes);
            }
        goto attach_error;
        }

    return OK;

attach_error:
    if (pDrvCtrl->semSync != SEM_ID_NULL)
        {
        (void)semDelete (pDrvCtrl->semSync);
        }

    if (pDrvCtrl->intRes != NULL)
        {
        (void)vxbResourceFree (pDev, pDrvCtrl->intRes);
        }

    if (pRes != NULL)
        {
        (void)vxbResourceFree (pDev, pRes);
        }

    (void)vxbClkDisableAll (pDev);
    (void)vxbPinMuxDisable (pDev);
    vxbMemFree (pDrvCtrl);
    return ERROR;
    }

/*******************************************************************************
*
* am335xSpiInit -  SPI controller initialization
*
* This routine performs the SPI controller initialization.
*
* RETURNS: OK or ERROR when failed.
*
* ERRNO: N/A
*/

LOCAL STATUS am335xSpiInit
    (
    VXB_DEV_ID  pDev
    )
    {
    UINT32  value;
    UINT32  us = 0;

    /* software reset */

    CSR_SETBIT_4 (pDev, MCSPI_SYSCONFIG, SPI_RESET);

    /* wait the reset is done */

    do
        {
        vxbUsDelay (1);
        value = CSR_READ_4 (pDev, MCSPI_SYSSTATUS);

        if (us++ > SPI_TIMEOUT_US)
            {
            return ERROR;
            }
        }while ((value & SPI_RESET_DONE) == 0);

    /* disable all SPI interrupts */

    CSR_WRITE_4 (pDev, MCSPI_IRQENABLE, 0);

    /* clear previous pending IRQ status */

    value = CSR_READ_4 (pDev, MCSPI_IRQSTATUS);
    CSR_WRITE_4 (pDev, MCSPI_IRQSTATUS, value);

    /* Configure the SPI for master/single channel mode */

    CSR_CLRBIT_4 (pDev, MCSPI_MODULCTRL, ~MCSPI_MODULCTRL_MS);
    CSR_SETBIT_4 (pDev, MCSPI_MODULCTRL, MCSPI_MODULCTRL_SINGLE);

    return OK;
    }

/*******************************************************************************
*
* am335xSpiPinMux - init SPI PinMux
*
* This routine handles PinMux of SPI.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am335xSpiPinMux
    (
    struct vxbDev * pDev
    )
    {
    CSR_WRITE_PINMUX_4(pDev, SPI0_SCLK_PINMUX_REG_OFFSET, 
                       AM65X_IOPAD_VAL (MCSPI_PIN_INPUT, MCSPI_PIN_MUX_MODE));  /* SPI0_CLK */
    CSR_WRITE_PINMUX_4(pDev, SPI0_D0_PINMUX_REG_OFFSET, 
                       AM65X_IOPAD_VAL (MCSPI_PIN_INPUT, MCSPI_PIN_MUX_MODE));  /* SPI0_D0 */
    CSR_WRITE_PINMUX_4(pDev, SPI0_D1_PINMUX_REG_OFFSET, 
                       AM65X_IOPAD_VAL (MCSPI_PIN_INPUT, MCSPI_PIN_MUX_MODE));  /* SPI0_D1 */
    CSR_WRITE_PINMUX_4(pDev, SPI0_CS0_PINMUX_REG_OFFSET, 
                       AM65X_IOPAD_VAL (MCSPI_PIN_OUTPUT, MCSPI_PIN_MUX_MODE));  /* SPI0_CS0 */
    }

/*******************************************************************************
*
* am335xSpiIsr - interrupt service routine
*
* This routine handles interrupts of SPI.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am335xSpiIsr
    (
    VXB_DEV_ID  pDev
    )
    {
    TI_AM335X_SPI_CTRL *    pDrvCtrl;

    if (pDev == NULL)
        {
        return;
        }

    pDrvCtrl = (TI_AM335X_SPI_CTRL *) vxbDevSoftcGet (pDev);
    if (pDrvCtrl == NULL)
        {
        return;
        }

    /* disable all SPI interrupts */

    CSR_WRITE_4 (pDev, MCSPI_IRQENABLE, 0);

    SPI_DBG (SPI_DBG_ISR, "am335xSpiIsr: intSts 0x%x\n",
             CSR_READ_4 (pDev, MCSPI_IRQSTATUS));

    (void)semGive (pDrvCtrl->semSync);
    }

/*******************************************************************************
*
* am335xSpiChanCfg - configure the SPI channel
*
* This routine configures the SPI channel register (MCSPI_CH(i)CONF) which
* controls SPI master operation with its parameters.
*
* RETURNS: OK if device was successfully configured, otherwise ERROR
*
* ERRNO: N/A
*/

 LOCAL STATUS am335xSpiChanCfg
    (
    VXB_DEV_ID      pDev,       /* controller pDev */
    SPI_HARDWARE *  devInfo     /* device info */
    )
    {
    UINT32                  spiConf;
    UINT32                  spiCtl;
    UINT32                  wordLen;
    UINT32                  divideRatio;
    UINT32                  div = 0;
    UINT32                  channel;
    TI_AM335X_SPI_CTRL *    pDrvCtrl;

    pDrvCtrl = (TI_AM335X_SPI_CTRL *)vxbDevSoftcGet (pDev);

    channel = devInfo->chipSelect;
    if (channel >= SPI_MAX_CS_NUM)
        {
        SPI_DBG (SPI_DBG_ERR, "am335xSpiChanCfg: invalid SPI cs:%d\n",
                 channel);
        return ERROR;
        }

    wordLen = devInfo->bitWidth;

    /* only support data length from 4 to 32 bits */

    if (wordLen > SPI_WL_MAX || wordLen < SPI_WL_MIN)
        {
        SPI_DBG (SPI_DBG_ERR, "am335xSpiChanCfg: invalid SPI word lenght\n");
        return ERROR;
        }

    wordLen--;
    spiConf = wordLen << MCSPI_CH0CONF_WL_SHIFT;

    /* calculate clock divisor */

    if (devInfo->devFreq == 0)
        {
        SPI_DBG (SPI_DBG_ERR, "am335xSpiChanCfg: device frequency is 0\n");
        return ERROR;
        }

    divideRatio = pDrvCtrl->clkFrequency / devInfo->devFreq;
    if (pDrvCtrl->clkFrequency % devInfo->devFreq != 0)
        {
        divideRatio++;
        }

    if (divideRatio > MAX_DIVIDERATIO_2)
        {
        SPI_DBG (SPI_DBG_ERR, "am335xSpiChanCfg: unsupported divisor\n");
        return ERROR;
        }
    else if ((divideRatio >= MAX_DIVIDERATIO_1) ||
             ((divideRatio & (divideRatio - 1)) == 0) /* It is power of 2 */
            )
        {

        /* clock granularity of power of 2 */

        spiConf &= ~(1u << MCSPI_CH0CONF_CLKG_SHIFT);
        divideRatio = 1;
        while (pDrvCtrl->clkFrequency / (1 << div) > devInfo->devFreq)
            {
            div++;
            divideRatio <<= 1;
            }
        spiConf |= (div << MCSPI_CH0CONF_CLKD_SHIFT);
        }
    else
        {

        /* 1 clock cycle granularity */

        spiConf |= (1 << MCSPI_CH0CONF_CLKG_SHIFT);

        spiConf |= (((divideRatio - 1) & MCSPI_CH0CONF_CLKD_MASK) <<
                                                     MCSPI_CH0CONF_CLKD_SHIFT);
        spiCtl = (((divideRatio - 1) >> MCSPI_CH0CONF_CLKD_SHIFT) <<
                                                   MCSPI_CH0CTRL_EXTCLK_SHIFT);
        CSR_WRITE_4 (pDev, MCSPI_CHCTRL (channel), spiCtl);
        }

    /* save the current working frequency */

    pDrvCtrl->curWorkingFrq = pDrvCtrl->clkFrequency / divideRatio;

    /* SPIEN polarity */

    if ((devInfo->mode & SPI_CSPOL_HIGH) != 0)
        {
        spiConf &= ~(1u << MCSPI_CH0CONF_EPOL_SHIFT);  /* high active */
        }
    else
        {
        spiConf |= (1 << MCSPI_CH0CONF_EPOL_SHIFT);   /* low active */
        }

    /* SPI clock polarity */

    if ((devInfo->mode & SPI_CKPOL) != 0)
        {
        spiConf |= MCSPI_CH0CONF_POL;    /* low active */
        }
    else
        {
        spiConf &= ~MCSPI_CH0CONF_POL;   /* high active */
        }

    /* SPI clock phase */

    if ((devInfo->mode & SPI_CKPHA) != 0)
        {
        spiConf |= MCSPI_CH0CONF_PHA;    /* latched on even numbered edges */
        }
    else
        {
        spiConf &= ~MCSPI_CH0CONF_PHA;   /* latched on odd numbered edges */
        }

    /* SPIDAT[1] for reception/SPIDAT[0] for transmission */

    spiConf |= MCSPI_CH0CONF_DPE1;

    /* 1.5 clock cycles */

    spiConf |= MCSPI_CH0CONF_TCS;

    /* write the configuration to register */

    CSR_WRITE_4 (pDev, MCSPI_CHCONF (channel), spiConf);

    SPI_DBG (SPI_DBG_RTN,
             "am335xSpiChanCfg: CHCONF:0x%x spiFreq:%d devFreq:%d curFreq:%d\n",
             spiConf, pDrvCtrl->clkFrequency, devInfo->devFreq,
             pDrvCtrl->curWorkingFrq);

    return OK;
    }

/*******************************************************************************
*
* am335xSpiWaitBitSet - wait a register's specific bit is set
*
* The routine waits a register's specific bit is set
*
* RETURNS: OK or ERROR if timeout
*
* ERRNO: N/A
*/

LOCAL STATUS am335xSpiWaitBitSet
    (
    VXB_DEV_ID  pDev,       /* controller pDev */
    UINT32      regOffset,  /* Register  */
    UINT32      regBit      /* which bit */
    )
    {
    STATUS                  sts = ERROR;
    int                     timeout;
    TI_AM335X_SPI_CTRL *    pDrvCtrl;

    pDrvCtrl = (TI_AM335X_SPI_CTRL *) vxbDevSoftcGet (pDev);

    timeout = (8 * SPI_FIFO_LEN * 1000000) / pDrvCtrl->curWorkingFrq;
    timeout++;

    do
        {
        if ((CSR_READ_4 (pDev, regOffset) & regBit) != 0)
            {
            sts = OK;
            break;
            }

        vxbUsDelay (1);
        }while (timeout--);

    return sts;
    }

/*******************************************************************************
*
* am335xSpiChanShutdown - shutdown one SPI channel
*
* The routine performs shutdown operation of a specific SPI channel
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

 LOCAL void am335xSpiChanShutdown
    (
    VXB_DEV_ID  pDev,   /* controller pDev */
    UINT32      chan    /* SPI channel */
    )
    {
    UINT32  spiConf;

    /* Disable SPI */

    CSR_CLRBIT_4 (pDev, MCSPI_CHCTRL(chan), ~MCSPI_CH0CTRL_EN);

    spiConf = CSR_READ_4 (pDev, MCSPI_CHCONF(chan));

    /* disable Tx/Rx FIFO */

    spiConf &= ~(MCSPI_CH0CONF_FFER | MCSPI_CH0CONF_FFEW);

    /* Transmit-Receive mode */

    spiConf &= ~MCSPI_CHOCONF_TRM_MASK;

    /* disable Tx/Rx DMA */

    spiConf &= ~(MCSPI_CH0CONF_DMAR | MCSPI_CH0CONF_DMAW);

    /* deactive cs */

    spiConf &= ~MCSPI_CH0CONF_FORCE;

    CSR_WRITE_4 (pDev, MCSPI_CHCONF(chan), spiConf);

    /* disable all SPI interrupts */

    CSR_WRITE_4 (pDev, MCSPI_IRQENABLE, 0);

    /* clear previous pending IRQ status */

    CSR_WRITE_4 (pDev, MCSPI_IRQSTATUS,
                 CSR_READ_4 (pDev, MCSPI_IRQENABLE));
    }

/*******************************************************************************
*
* am335xSpiTrans - start SPI transfer
*
* The routine starts a transmission
*
* RETURNS: OK or ERROR.
*
* ERRNO: N/A
*/

LOCAL STATUS am335xSpiTrans
    (
    VXB_DEV_ID      pDev,        /* controller pDev */
    SPI_HARDWARE *  devInfo,     /* device info */
    SPI_TRANSFER *  pPkg         /* transfer data info */
    )
    {
    TI_AM335X_SPI_CTRL *    pDrvCtrl;
    UINT32                  channel;
    UINT32                  spiConf;
    BOOL                    isFullDuplex;
    UINT32                  len;
    UINT32                  intSts;
    UINT32                  alignSize;
    UINT32                  data;
    UINT32                  i;
    UINT32                  timeoutTick;
    STATUS                  sts = OK;
    UINT32                  skipRxCnt;

    pDrvCtrl = (TI_AM335X_SPI_CTRL *) vxbDevSoftcGet (pDev);

    channel  = devInfo->chipSelect;

    CSR_WRITE_4 (pDev, MCSPI_IRQENABLE, 0);

     if (devInfo->bitWidth <= 8)       /* 4 to 8 bits */
        {
        alignSize = sizeof(char);
        }
    else if (devInfo->bitWidth <= 16) /* 9 to 16 bits */
        {
        alignSize = sizeof(UINT16);
        }
    else    /* 17 to 32 bits */
        {
        alignSize = sizeof(UINT32);
        }

    spiConf = CSR_READ_4 (pDev, MCSPI_CHCONF(channel));

    spiConf |= (MCSPI_CH0CONF_FFEW | MCSPI_CH0CONF_FFER);
    spiConf &= ~(MCSPI_CH0CONF_DMAR | MCSPI_CH0CONF_DMAW);

    /* Transmit-Receive mode */

    spiConf &= ~(MCSPI_TRM_TX | MCSPI_TRM_RX);

    isFullDuplex = IS_SPI_FULL_DUPLEX(devInfo->mode);
    skipRxCnt = isFullDuplex ? 0 : pPkg->txLen;
    len = pDrvCtrl->txLeft > pDrvCtrl->rxLeft ? pDrvCtrl->txLeft :
          pDrvCtrl->rxLeft;
    if (len > SPI_FIFO_LEN)
        {
        CSR_WRITE_4 (pDev, MCSPI_XFERLEVEL,
                     (SPI_FIFO_LEN - 1) << MCSPI_XFERLEVEL_AFL_SHIFT);
        }
    else
        {
        CSR_WRITE_4 (pDev, MCSPI_XFERLEVEL,
                     (len - 1) << MCSPI_XFERLEVEL_AFL_SHIFT);
        }

    spiConf |= MCSPI_CH0CONF_FORCE;        /* forece cs */

    CSR_WRITE_4 (pDev, MCSPI_CHCONF(channel), spiConf);

    /* Enable SPI */

    CSR_SETBIT_4 (pDev, MCSPI_CHCTRL(channel), MCSPI_CH0CTRL_EN);

    /* Clear the pending interrupts */

    CSR_WRITE_4 (pDev, MCSPI_IRQSTATUS, CSR_READ_4 (pDev, MCSPI_IRQSTATUS));

    /* enable Rx interrupt only. Tx in Rx interrupt */

    if ((pDrvCtrl->intRes != NULL) &&
        ((devInfo->mode & SPI_POLLING) == 0))
        {
        CSR_SETBIT_4 (pDev, MCSPI_IRQENABLE, MCSPI_IRQENABLE_RX (channel));
        }

    for (i = 0; i < SPI_FIFO_LEN && i < len; i += alignSize)
        {
        if (pDrvCtrl->txLeft > 0)
            {
            if (alignSize == sizeof (char))
                {
                data = pDrvCtrl->txBuf[pDrvCtrl->txLen - pDrvCtrl->txLeft];
                }
            else if (alignSize == sizeof (UINT16))
                {
                data = ((UINT16 *)pDrvCtrl->txBuf)[pDrvCtrl->txLen -
                                                   pDrvCtrl->txLeft];
                }
            else
                {
                data = ((UINT32 *)pDrvCtrl->txBuf)[pDrvCtrl->txLen -
                                                   pDrvCtrl->txLeft];
                }

            CSR_WRITE_4 (pDev, MCSPI_TX (channel), data);

            SPI_DBG (SPI_DBG_RW, "Tx: 0x%x\n", data);

            pDrvCtrl->txLeft -= alignSize;
            }
        else
            {
            CSR_WRITE_4 (pDev, MCSPI_TX (channel), 0);  /* write dummy */
            }
        }

    do
        {
        if ((pDrvCtrl->intRes != NULL) &&
            ((devInfo->mode & SPI_POLLING) == 0))
            {
            timeoutTick = ((UINT32) vxbSysClkRateGet () * 8u * SPI_FIFO_LEN) /
                          pDrvCtrl->curWorkingFrq;
            timeoutTick++;

            (void)semTake (pDrvCtrl->semSync, timeoutTick);
            }

        /* Read SPI Interrupt Status Register */

        intSts = CSR_READ_4 (pDev, MCSPI_IRQSTATUS);

        if ((pDrvCtrl->intRes != NULL) &&
            ((devInfo->mode & SPI_POLLING) == 0))
            {
            SPI_DBG (SPI_DBG_RW, "am335xSpiIsr: intSts 0x%x\n", intSts);

            if (intSts == 0)  /* fake interrupt or timeout */
                {
                SPI_DBG (SPI_DBG_ERR, "am335xSpiIsr: fake interrupt or "
                         "timeout\n");

                sts = ERROR;
                goto intDone;
                }
            }

        CSR_WRITE_4 (pDev, MCSPI_IRQSTATUS, intSts);

        /* Handle Rx */

        if (IS_RX_INT (channel, intSts))
            {

            /* Rx */

            for (i = 0; i < SPI_FIFO_LEN && i < len; i += alignSize)
                {
                if (skipRxCnt > 0)
                    {
                    (void)CSR_READ_4 (pDev, MCSPI_RX (channel));
                    skipRxCnt -= alignSize;
                    }
                else if (pDrvCtrl->rxLeft > 0)
                    {
                    data =  CSR_READ_4 (pDev, MCSPI_RX (channel));
                    if (alignSize == sizeof(char))
                        {
                        pDrvCtrl->rxBuf[pDrvCtrl->rxLen - pDrvCtrl->rxLeft]
                                  = (UINT8)data;
                        }
                    else if (alignSize == sizeof(UINT16))
                        {
                        ((UINT16 *)pDrvCtrl->rxBuf)[pDrvCtrl->rxLen - pDrvCtrl->rxLeft]
                                  = (UINT16)data;
                        }
                    else
                        {
                        ((UINT32 *)pDrvCtrl->rxBuf)[pDrvCtrl->rxLen - pDrvCtrl->rxLeft]
                                  = data;
                        }

                    SPI_DBG (SPI_DBG_RW, "Rx: 0x%x\n", data);
                    pDrvCtrl->rxLeft -= alignSize;
                    }
                else
                    {
                    (void) CSR_READ_4 (pDev, MCSPI_RX (channel));  /* read dummy */
                    }
                }

            len = pDrvCtrl->txLeft > pDrvCtrl->rxLeft ? pDrvCtrl->txLeft :
                  pDrvCtrl->rxLeft;

            if (len > SPI_FIFO_LEN)
                {
                CSR_WRITE_4 (pDev, MCSPI_XFERLEVEL,
                             (SPI_FIFO_LEN - 1) << MCSPI_XFERLEVEL_AFL_SHIFT);
                }
            else
                {
                CSR_WRITE_4 (pDev, MCSPI_XFERLEVEL,
                             (len - 1) << MCSPI_XFERLEVEL_AFL_SHIFT);
                }

            /* Tx */

            for (i = 0; i < SPI_FIFO_LEN && i < len; i += alignSize)
                {
                if (pDrvCtrl->txLeft > 0)
                    {
                    if (alignSize == sizeof (char))
                        {
                        data = pDrvCtrl->txBuf[pDrvCtrl->txLen -
                                               pDrvCtrl->txLeft];
                        }
                    else if (alignSize == sizeof (UINT16))
                        {
                        data = ((UINT16 *)pDrvCtrl->txBuf)[pDrvCtrl->txLen -
                                                           pDrvCtrl->txLeft];
                        }
                    else
                        {
                        data = ((UINT32 *)pDrvCtrl->txBuf)[pDrvCtrl->txLen -
                                                           pDrvCtrl->txLeft];
                        }

                    CSR_WRITE_4 (pDev, MCSPI_TX (channel), data);

                    SPI_DBG (SPI_DBG_RW, "Tx: 0x%x\n", data);

                    pDrvCtrl->txLeft -= alignSize;;
                    }
                else
                    {
                    CSR_WRITE_4 (pDev, MCSPI_TX (channel), 0);  /* write dummy */
                    }
                }

            if ((skipRxCnt == 0) && (pDrvCtrl->txLeft == 0) &&
                (pDrvCtrl->rxLeft == 0))
                {
                sts = am335xSpiWaitBitSet (pDev, MCSPI_CHSTAT (channel),
                                           MCSPI_CH0STAT_TXFFE);
                break;
                }

            if ((pDrvCtrl->intRes != NULL) &&
                ((devInfo->mode & SPI_POLLING) == 0))
                {
                /* enable Rx interrupt only. Tx in Rx interrupt */

                CSR_SETBIT_4 (pDev, MCSPI_IRQENABLE,
                              MCSPI_IRQENABLE_RX (channel));
                }
            }
        }while (TRUE);

intDone:

    am335xSpiChanShutdown (pDev, channel);

    return sts;
    }

/*******************************************************************************
*
* am335xSpiTransfer - SPI transfer routine
*
* This routine is used to perform one transmission. It is the interface which
* can be called by SPI device driver to send and receive data via the SPI
* controller.
*
* RETURNS: OK or ERROR
*
* ERRNO: N/A
*/

LOCAL STATUS am335xSpiTransfer
    (
    VXB_DEV_ID      pDev,           /* controller pDev */
    SPI_HARDWARE *  devInfo,        /* device info */
    SPI_TRANSFER *  pPkg            /* transfer data info */
    )
    {
    TI_AM335X_SPI_CTRL *    pDrvCtrl;

    pDrvCtrl = (TI_AM335X_SPI_CTRL *)vxbDevSoftcGet (pDev);
    if (pDrvCtrl == NULL)
        {
        SPI_DBG (SPI_DBG_ERR, "am335xSpiTransfer pDrvCtrl is NULL \n");
        return ERROR;
        }

    SPI_DBG (SPI_DBG_RTN, "am335xSpiTransfer  txLen[%d] rxLen[%d] \n",
             pPkg->txLen, pPkg->rxLen);

    if (am335xSpiChanCfg (pDev, devInfo) != OK)
        {
        SPI_DBG (SPI_DBG_ERR, "am335xSpiChanCfg failed.\n");
        return ERROR;
        }

    pDrvCtrl->txBuf  = pPkg->txBuf;
    pDrvCtrl->txLen  = pPkg->txLen;
    pDrvCtrl->txLeft = pPkg->txLen;
    pDrvCtrl->rxBuf  = pPkg->rxBuf;
    pDrvCtrl->rxLen  = pPkg->rxLen;
    pDrvCtrl->rxLeft = pPkg->rxLen;

    return am335xSpiTrans (pDev, devInfo, pPkg);
    }
