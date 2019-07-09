/* vxbTiK3Omap3Timer.c - TI OMAP35xx processor timer library */

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

/*
DESCRIPTION
This is the vxBus compliant timer driver which implements the functionality
specific to OMAP35xx timer series. This driver currently only support AM335X and
 AM437X.

A timer-specific data structure (struct omap35xxTimerData) is maintained
within this driver.  This is given as 'pCookie' in omap35xxTimerAllocate() and
is used from then on by the Timer Abstraction Layer or application to
communicate with this driver.

The driver implements the vxBus driver-specific methods
routines like omap35xxTimerAttach(), ofOmap3TimerProbe.

A variable of type 'struct omap35xxTimerData' is allocated for
a timer device and stored in 'pInst->pDrvCtrl' in omap35xxTimerInstInit()

omap35xxTimerAttach() hooks an ISR to be used for the timer device.

Hardware Functional Description:

Each GP timer contains a free-running upward counter with autoreload
capability on overflow. The timer counter can be read and written
on-the-fly (while counting). Each GP timer includes compare logic to
allow an interrupt event on a programmable counter matching value. A
dedicated output signal can be pulsed or toggled on either an overflow
or a match event. This offers time-stamp trigger signaling or PWM
signal sources. A dedicated input signal can be used to trigger an
automatic timer counter capture or an interrupt event on a
programmable input signal transition type. A programmable clock
divider (prescaler) allows reduction of the timer input clock
frequency. All internal timer interrupt sources are merged into one
module interrupt line and one wake-up line. Each internal interrupt
source can be independently enabled/disabled with a dedicated bit of
the GPTi.TIER register for the interrupt features and a dedicated bit
of the GPTi.TWER register for the wakeup. In addition, GPTIMER1,
GPTIMER2, and GPTIMER10 have implemented a mechanism to generate an
accurate tick interrupt.

Note: many of the above features are not implemented in this driver.  This
driver provides functionality for timestamp and periodic interrupt
capabilities.

Note: Although some infrastructure is built into this driver to lay the
groundwork for supporting power management, the full capabilities of
power management supported by the hardware are not supported by this driver.

To add the driver to the vxWorks image, add the following component to the
kernel configuration.

\cs
vxprj component add DRV_TIMER_FDT_OMAP3
\ce

Each device should be bound to a device tree node which requires
below properties:

\cs
compatible:     Specify the programming model for the device.
                It should be set to "ti,dmtimer" and is used
                by vxbus GEN2 for device driver selection.

reg:            Specify the address of the device's resources within
                the address space defined by its parent bus.

clock-frequency: Specify the clock frequency of the timers's clock
                 source in HZ.

interrupts:     Specify interrupt vector of the interrupts that are generated
                by this device.

interrupt-parent: This property is available to define an interrupt
                parent. if it is missing from a device, it's interrupt parent
                is assumed to be its device tree parent.
\ce

Below is an example:

\cs
        dmtimer0: dmtimer@44E05000
            {
            compatible = "ti,dmtimer";
            #address-cells = <1>;
            #size-cells = <1>;
            reg = <0x44E05000 0x400>;
            interrupts = <98>;
            interrupt-parent = <&intc>;
            };
\ce
            
INCLUDE FILES: vxbTimerLib.h vxbOmap3Timer.h
*/

/* includes */

#include <vxWorks.h>
#include <string.h>
#include <intLib.h>
#include <vsbConfig.h>
#include <errnoLib.h>
#include <errno.h>
#include <sioLib.h>
#include <ioLib.h>
#include <stdio.h>
#include <logLib.h> /* for logMsg */
#include <windPwrLib.h>
#include <hwif/vxBus.h>
#include <subsys/int/vxbIntLib.h>
#include <subsys/timer/vxbTimerLib.h>
#include <subsys/clk/vxbClkLib.h>
#include <subsys/pinmux/vxbPinMuxLib.h>
#include "vxbTiK3Omap3Timer.h"
#include <hwif/buslib/vxbFdtLib.h>

/* debug macro */

#undef TI_TIMER_DEBUG
#ifdef TI_TIMER_DEBUG
#include <private/kwriteLibP.h>         /* _func_kprintf */

#define DBG_OFF             0x00000000
#define DBG_WARN            0x00000001
#define DBG_ERR             0x00000002
#define DBG_INFO            0x00000004
#define DBG_ALL             0xffffffff
LOCAL UINT32 dbgMask = DBG_ERR;

#undef DBG_MSG
#define DBG_MSG(mask,...)                          \
do                                                 \
{                                                  \
    if ((dbgMask & (mask)) || ((mask) == DBG_ALL)) \
    {                                              \
        if (_func_kprintf != NULL)                 \
        {                                          \
        (* _func_kprintf)("%s,%d, ",__FUNCTION__,__LINE__);\
        (* _func_kprintf)(__VA_ARGS__);            \
        }                                          \
    }                                              \
}while (0)
#else
#define DBG_MSG(...)
#endif  /* TI_TIMER_DEBUG */

/* externs */

IMPORT struct vxbTimerFunctionality * pClkTimer;

/* defines */

/* VxBus-compliant register access macros */

#define CSR_READ_4(pDev, addr)                                              \
    vxbRead32 ((pDev->vxbHandle),                                           \
               (UINT32 *)((char *)(pDev->baseAddress) + addr))

#define CSR_WRITE_4(pDev, addr, data)                                       \
    vxbWrite32 ((pDev->vxbHandle),                                          \
                (UINT32 *)((char *)(pDev->baseAddress) + addr), data)

#define OMAP35XX_TIMER_REG_SYNC_WRITE(pTimerData, reg, result, syncBit)     \
    do {                                                                    \
        if (pTimerData->isPosted == TRUE)                                   \
            {                                                               \
            UINT32 __val;                                                   \
            do                                                              \
                {                                                           \
                __val = CSR_READ_4 (pTimerData, TIMER_TWPS_OFFSET) & 0xff;  \
                } while (__val & syncBit);                                  \
            }                                                               \
        CSR_WRITE_4 (pTimerData, reg, result);                              \
    } while (0)

#define OMAP35XX_TIMER_REG_SYNC_READ(pTimerData, reg, result, syncBit)      \
    do {                                                                    \
        if (pTimerData->isPosted == TRUE)                                   \
            {                                                               \
            UINT32 __val;                                                   \
            do                                                              \
                {                                                           \
                __val = CSR_READ_4 (pTimerData, TIMER_TWPS_OFFSET) & 0xff;  \
                } while (__val & syncBit);                                  \
            }                                                               \
        result = CSR_READ_4 (pTimerData, reg);                              \
    } while (0)

#define OMAP35XX_TIMER_REG_READ(pTimerData, reg, result)                    \
    do {                                                                    \
        result = CSR_READ_4(pTimerData, reg);                               \
    } while (FALSE)

#define OMAP35XX_TIMER_REG_WRITE(pTimerData, reg, result)                   \
    do {                                                                    \
        CSR_WRITE_4(pTimerData, reg, result);                               \
    } while (FALSE)

#define REG_SYNC                                                            \
    do {                                                                    \
        OMAP35XX_TIMER_REG_READ (pTimerData, TIMER_TWPS_OFFSET, regVal);    \
    } while (regVal != 0x00000000)

#define OMAP35XX_MAX_COUNT_VAL      0xffffffff
#define OMAP35XX_MAX_MATCH_VAL      0xffffff
#define TIMER_NAME                  "omap35xxTimer"

/* function declarations */

LOCAL STATUS ofOmap3TimerProbe (VXB_DEV_ID);
LOCAL STATUS ofOmap3TimerAttach (VXB_DEV_ID);
LOCAL STATUS vxbOmap3TimerAttach (VXB_DEV_ID);
LOCAL STATUS vxbOmap3TimerModuleDetect (OMAP35XX_TIMER_DATA *);

/* Locals */

/*
 * Those registers have different offset in omap35xx and dm81xx,
 * but the function are the same.
 * Use array to reference.
 */

LOCAL UINT32 omap35xxRegs[] =
    {
    OMAP35XX_TIMER_TIOCP_CFG_OFFSET,
    OMAP35XX_TIMER_TCLR_OFFSET,
    OMAP35XX_TIMER_TCRR_OFFSET,
    OMAP35XX_TIMER_TLDR_OFFSET,
    OMAP35XX_TIMER_TTGR_OFFSET,
    OMAP35XX_TIMER_TWPS_OFFSET,
    OMAP35XX_TIMER_TMAR_OFFSET,
    OMAP35XX_TIMER_TCAR1_OFFSET,
    OMAP35XX_TIMER_TSICR_OFFSET,
    OMAP35XX_TIMER_TCAR2_OFFSET,
    };

LOCAL UINT32 dm81xxRegs[] =
    {
    DM81XX_TIMER_TIOCP_CFG_OFFSET,
    DM81XX_TIMER_TCLR_OFFSET,
    DM81XX_TIMER_TCRR_OFFSET,
    DM81XX_TIMER_TLDR_OFFSET,
    DM81XX_TIMER_TTGR_OFFSET,
    DM81XX_TIMER_TWPS_OFFSET,
    DM81XX_TIMER_TMAR_OFFSET,
    DM81XX_TIMER_TCAR1_OFFSET,
    DM81XX_TIMER_TSICR_OFFSET,
    DM81XX_TIMER_TCAR2_OFFSET,
    };

LOCAL VXB_DRV_METHOD ofOmap3TimerMethodList[] =
    {
    /* DEVICE API */
    { VXB_DEVMETHOD_CALL(vxbDevProbe),  ofOmap3TimerProbe },
    { VXB_DEVMETHOD_CALL(vxbDevAttach), ofOmap3TimerAttach },
    { 0, NULL }
    };

VXB_DRV vxbOfTiK3Omap3TimerDrv =
    {
    { NULL } ,
    "omap-dmtimer",             /* Name */
    "OMAP Dual-Mode Timers",    /* Description */
    VXB_BUSID_FDT,               /* Class */
    0,                          /* Flags */
    0,                          /* Reference count */
    ofOmap3TimerMethodList      /* Method table */
    };

/* hold for extra configuration */

LOCAL OMAP3_TIMER_CFG dmTimerCfg =
    {
    TIMER_VERSION_UNUSED
    };

/*
 * OMAP35XX doesn't have fixed TIDR, so set timer version as 
 * TIMER_VERSION_OMAP35XX to identify its timer module.
 */

LOCAL OMAP3_TIMER_CFG omap3TimerCfg =
    {
    TIMER_VERSION_OMAP35XX
    };

/* currently support am335x only */

LOCAL const VXB_FDT_DEV_MATCH_ENTRY dmtimerMatch [] =
    {
        {
        "ti,dmtimer",
        (void *) &dmTimerCfg
        },
        {
        "ti,omap35xx-dmtimer-1ms",
        (void *) &omap3TimerCfg
        },
        {}                      /* Empty terminated list */
    };

#define TIMER_TIOCP_CFG_OFFSET  pTimerData->pChipRegs[0]
#define TIMER_TCLR_OFFSET       pTimerData->pChipRegs[1]
#define TIMER_TCRR_OFFSET       pTimerData->pChipRegs[2]
#define TIMER_TLDR_OFFSET       pTimerData->pChipRegs[3]
#define TIMER_TTGR_OFFSET       pTimerData->pChipRegs[4]
#define TIMER_TWPS_OFFSET       pTimerData->pChipRegs[5]
#define TIMER_TMAR_OFFSET       pTimerData->pChipRegs[6]
#define TIMER_TCAR1_OFFSET      pTimerData->pChipRegs[7]
#define TIMER_TSICR_OFFSET      pTimerData->pChipRegs[8]
#define TIMER_TCAR2_OFFSET      pTimerData->pChipRegs[9]

/* globals */

VXB_DRV_DEF (vxbOfTiK3Omap3TimerDrv)

/*******************************************************************************
*
* vxbOmap35xxTimerClkInt - handle system clock interrupts
*
* This routine handles system clock interrupts.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void vxbOmap35xxTimerClkInt
    (
    VXB_DEV_ID pInst
    )
    {
    struct omap35xxTimerData *pTimerData;
    UINT32                    regVal;

    DBG_MSG (DBG_INFO, "vxbOmap35xxTimerClkInt\n");

    if ((pInst == NULL) || (vxbDevSoftcGet(pInst) == NULL))
        return;

    pTimerData = (struct omap35xxTimerData *)vxbDevSoftcGet(pInst);

    DBG_MSG (DBG_INFO, "base address = 0x%x\n",pTimerData->baseAddress);

    /*
     * Clear all types of pending interrupts by writing 1 to the status flag.
     * Bit 0 is match interrupt
     * Bit 1 is overflow interrupt
     * Bit 2 is capture interrupt
     */

    if (pTimerData->timerModule == TIMER_MODULE_DMTIMER)
        {
        OMAP35XX_TIMER_REG_WRITE (pTimerData, DM81XX_TIMER_IRQSTATUS_OFFSET,
                              0x00000007);

        OMAP35XX_TIMER_REG_READ (pTimerData, DM81XX_TIMER_IRQSTATUS_OFFSET, 
                              regVal);
        if (regVal != 0)
            {
            DBG_MSG (DBG_ERR, "Clear ISR error\n");
            }
        }
    else
        {
        OMAP35XX_TIMER_REG_WRITE (pTimerData, OMAP35XX_TIMER_TISR_OFFSET,
                              0x00000007);

        OMAP35XX_TIMER_REG_READ (pTimerData, OMAP35XX_TIMER_TISR_OFFSET,
                              regVal);
        if (regVal != 0)
            {
            DBG_MSG (DBG_ERR, "Clear ISR error\n");
            }
        }

    /* re-enable the timer if the autoReload is not set */

    if (!pTimerData->autoReload)
        {

        /* restart or start the timer*/

        OMAP35XX_TIMER_REG_SYNC_READ (pTimerData, TIMER_TCLR_OFFSET, regVal,
                        TIMER_W_PEND_TCLR);
        regVal |= 0x1;
        OMAP35XX_TIMER_REG_SYNC_WRITE (pTimerData, TIMER_TCLR_OFFSET, regVal,
                        TIMER_W_PEND_TCLR);
        }

    if (pTimerData->pIsrFunc != NULL)
        (*(pTimerData->pIsrFunc)) (pTimerData->arg);
    }

/*******************************************************************************
*
* omap35xxTimerAllocate - allocate resources for a timer
*
* This is the function called to allocate resources for a timer for usage by the
* Timer Abstraction Layer
*
* RETURNS: OK or ERROR if timer allocation fails
*
* ERRNO: N/A
*/

LOCAL STATUS omap35xxTimerAllocate
    (
    void *         pCookie,
    UINT32         flags
    )
    {
    struct omap35xxTimerData      * pTimerData;
    struct vxbTimerFunctionality  * pTimerFunc;

    VXB_ASSERT ( (pCookie != NULL), ERROR);

    pTimerFunc = (struct vxbTimerFunctionality *)pCookie;
    pTimerData = TIMERFUNC_TO_TIMERDATA(pTimerFunc);

    /* check whether 'pTimerData' is valid or not */

    if (pTimerData != NULL)
        {
        /* if the timer is already allocated, return ERROR */

        if (pTimerData->timerFunclty.allocated)
            return ERROR;

        /* set the auto-reload flag */

        if ((flags & VXB_TIMER_AUTO_RELOAD) != 0)
            pTimerData->autoReload = TRUE;

        /* set the allocated flag */

        pTimerData->timerFunclty.allocated = TRUE;
        }
    else
        return ERROR;

    return OK;
    }

/*******************************************************************************
*
* omap35xxTimerDisable - disable the timer
*
* This routine disables system clock interrupts.
*
* RETURNS: OK or ERROR if argument is NULL
*
* ERRNO: N/A
*/

LOCAL STATUS omap35xxTimerDisable
    (
    void*    pCookie
    )
    {
    struct omap35xxTimerData *   pTimerData;
    struct vxbTimerFunctionality * pTimerFunc;

    DBG_MSG (DBG_INFO, "omap35xxTimerDisable\n");

    /* check whether the parameters are valid */

    VXB_ASSERT ( (pCookie != NULL), ERROR);

    pTimerFunc = (struct vxbTimerFunctionality *)pCookie;
    pTimerData = TIMERFUNC_TO_TIMERDATA(pTimerFunc);

    if (pTimerData->timerEnabled)
        {
        /*
         * just disable the timer tick interrupt.
         * Note:
         *      don't disable timer counting,
         *      other functions(e.g sysUsDelay()) may use it.
         */

        if (vxbIntDisable (pTimerData->pInst, pTimerData->intRes) != OK)
            DBG_MSG (DBG_ERR, "int disable error!!!\n");

        pTimerData->timerEnabled = FALSE;
        }

    return OK;
    }

/*******************************************************************************
*
* omap35xxTimerRelease - release the timer resource
*
* This is the function called to release the resources allocated for a timer
* device
*
* RETURNS: OK or ERROR
*
* ERRNO: N/A
*/

LOCAL STATUS omap35xxTimerRelease
    (
    void * pCookie
    )
    {
    struct omap35xxTimerData     * pTimerData;
    struct vxbTimerFunctionality * pTimerFunc;

    DBG_MSG (DBG_INFO, "omap35xxTimerRelease\n");

    /* check whether the parameters are valid */

    VXB_ASSERT ( (pCookie != NULL), ERROR);

    pTimerFunc = (struct vxbTimerFunctionality *)pCookie;
    pTimerData = TIMERFUNC_TO_TIMERDATA(pTimerFunc);

    /* check whether the members are valid */

    if (!pTimerData->timerFunclty.allocated)
        return ERROR;

    if (omap35xxTimerDisable (pCookie) == ERROR)
        return ERROR;

    /* reset the autoReload flag */

    if (pTimerData->autoReload)
        pTimerData->autoReload = FALSE;

    pTimerData->pIsrFunc = NULL;
    pTimerData->arg = 0;

    /* reset the timer allocated flag */

    pTimerData->timerFunclty.allocated = FALSE;

    return OK;
    }

/*******************************************************************************
*
* omap35xxTimerRolloverGet - retrieve the maximum value of the counter
*
* This is the function called to retrieve the maximum value of the counter.
* The maximum value is returned in 'count' parameter.
*
* RETURNS: OK or ERROR if the parameter is invalid.
*
* ERRNO: N/A
*/

LOCAL STATUS omap35xxTimerRolloverGet
    (
    void*    pCookie,
    UINT32*  count
    )
    {
    struct omap35xxTimerData *   pTimerData;
    struct vxbTimerFunctionality * pTimerFunc;

    DBG_MSG (DBG_INFO, "omap35xxTimerRolloverGet\n");

    /* check whether the parameters are valid */

    VXB_ASSERT ( (pCookie != NULL), ERROR);

    pTimerFunc = (struct vxbTimerFunctionality *)pCookie;
    pTimerData = TIMERFUNC_TO_TIMERDATA(pTimerFunc);

    *count = pTimerData->rollover;

    return OK;
    }

/*******************************************************************************
*
* omap35xxTimerCountGet - retrieve the current timer count
*
* This function is used to retrieve the current timer count.
* The current value is returned in 'count' parameter.
*
* RETURNS: OK or ERROR if the parameter is invalid.
*
* ERRNO: N/A
*/

LOCAL STATUS omap35xxTimerCountGet
    (
    void*    pCookie,
    UINT32*  count
    )
    {
    struct omap35xxTimerData *   pTimerData;
    struct vxbTimerFunctionality * pTimerFunc;
    UINT32 timer_val, load_val;  /* timer counter and start/load values */

    DBG_MSG (DBG_INFO, "omap35xxTimerCountGet\n");

    /* check whether the parameters are valid */

    VXB_ASSERT ( (pCookie != NULL), ERROR);

    pTimerFunc = (struct vxbTimerFunctionality *)pCookie;
    pTimerData = TIMERFUNC_TO_TIMERDATA(pTimerFunc);


    OMAP35XX_TIMER_REG_SYNC_READ (pTimerData, TIMER_TCRR_OFFSET, timer_val,
                    TIMER_W_PEND_TCRR);
    OMAP35XX_TIMER_REG_SYNC_READ (pTimerData, TIMER_TLDR_OFFSET, load_val ,
                    TIMER_W_PEND_TLDR);

    /*
     * Since this is an upward counting timer, the count value is the
     * increase from the start/load value.
     */

    *count = timer_val - load_val;

    return OK;
    }

/*******************************************************************************
*
* omap35xxTimerISRSet - set a function to be called on the timer interrupt
*
* This function is called to set a function which can be called whenever
* the timer interrupt occurs.
*
* RETURNS: OK or ERROR if the parameter is invalid.
*
* ERRNO: N/A
*/

LOCAL STATUS omap35xxTimerISRSet
    (
    void*           pCookie,
    void            (*pFunc)(_Vx_usr_arg_t),
    _Vx_usr_arg_t    arg
    )
    {
    struct omap35xxTimerData *   pTimerData;
    struct vxbTimerFunctionality * pTimerFunc;

    DBG_MSG (DBG_INFO, "omap35xxTimerISRSet\n");

    /* check whether the parameters are valid */

    if (pCookie == NULL)
        return ERROR;

    pTimerFunc = (struct vxbTimerFunctionality *)pCookie;
    pTimerData = TIMERFUNC_TO_TIMERDATA(pTimerFunc);

    /*
     * Store the interrupt routine and argument information
     * 1) set the function to NULL so old ISR is not called while we are here
     * 2) next set the arg so if the new function is called while as we are
     *    setting up, it does not get the arg from the old ISR
     * 3) assign the function last
     */

    pTimerData->pIsrFunc = NULL;
    pTimerData->arg = arg;
    pTimerData->pIsrFunc = pFunc;

    return OK;
    }

/*******************************************************************************
*
* omap35xxTimerEnable - enable the timer
*
* This routine enables system clock interrupts.
*
* RETURNS: OK or ERROR if argument is NULL
*
* ERRNO: N/A
*/

LOCAL STATUS omap35xxTimerEnable
    (
    void*    pCookie,
    UINT32   maxTimerCount
    )
    {
    struct omap35xxTimerData *   pTimerData;
    struct vxbTimerFunctionality * pTimerFunc;
    UINT32                    regVal;

    DBG_MSG (DBG_INFO, "omap35xxTimerEnable(0x%x, %d)\n",
                            pCookie, maxTimerCount);

    if (pCookie == NULL)
        return ERROR;

    pTimerFunc = (struct vxbTimerFunctionality *)pCookie;
    pTimerData = TIMERFUNC_TO_TIMERDATA(pTimerFunc);

    pTimerData->rollover = maxTimerCount;

    DBG_MSG (DBG_INFO, "base address = 0x%x\n",pTimerData->baseAddress);

    /* Hit the reset bit to stop the timer & reset various regs */

    if (pTimerData->timerModule == TIMER_MODULE_DMTIMER)
        {
        OMAP35XX_TIMER_REG_WRITE (pTimerData, TIMER_TIOCP_CFG_OFFSET, 0x1);

        /* wait for reset to complete */

        do
            {
            OMAP35XX_TIMER_REG_READ (pTimerData, TIMER_TIOCP_CFG_OFFSET,
                                     regVal);
            }
        while (regVal & 0x1);
        }
    else
        {
        OMAP35XX_TIMER_REG_WRITE (pTimerData, TIMER_TIOCP_CFG_OFFSET, 0x2);

        /* wait for reset to complete */

        do
            {
            OMAP35XX_TIMER_REG_READ (pTimerData, OMAP35XX_TIMER_TISTAT_OFFSET,
                                     regVal);
            }
        while (!(regVal & 0x1));
        }

    /*
     * Set the rollover load register
     * The timer counts from rollover up to 0xffffffff so subtract.
     */

    /*
     * Caution: Do not put the overflow value (0xffffffff) in the GPTi.TLDR
     * register because it can lead to undesired results
     *
     */

    if (pTimerData->rollover == 0)
        pTimerData->rollover = 1;

    DBG_MSG (DBG_INFO, "TLDR=0x%x\n",
             OMAP35XX_MAX_COUNT_VAL - pTimerData->rollover);

    /* set up value to be loaded */

    OMAP35XX_TIMER_REG_SYNC_WRITE (pTimerData, TIMER_TLDR_OFFSET,
                                   OMAP35XX_MAX_COUNT_VAL - pTimerData->rollover,
                                   TIMER_W_PEND_TLDR);

    /* and load it */

    OMAP35XX_TIMER_REG_SYNC_WRITE (pTimerData, TIMER_TTGR_OFFSET, 0x1,
                                   TIMER_W_PEND_TTGR);

    /* enable overflow interrupts and wakeup (capture and match are disabled) */

    if (pTimerData->timerModule == TIMER_MODULE_DMTIMER)
        {
        OMAP35XX_TIMER_REG_WRITE (pTimerData,
                                  DM81XX_TIMER_IRQSTATUS_SET_OFFSET, 0x2);
        OMAP35XX_TIMER_REG_WRITE (pTimerData,
                                  DM81XX_TIMER_IRQWAKEEN_OFFSET, 0x2);
        }
    else
        {
        OMAP35XX_TIMER_REG_WRITE (pTimerData, OMAP35XX_TIMER_TIER_OFFSET, 0x2);
        OMAP35XX_TIMER_REG_WRITE (pTimerData, OMAP35XX_TIMER_TWER_OFFSET, 0x2);
        }

    if (pTimerData->autoReload)
        {
        /* enable autoreload and start the timer with overflow */

        OMAP35XX_TIMER_REG_SYNC_WRITE (pTimerData, TIMER_TCLR_OFFSET,
                                       0x403, TIMER_W_PEND_TCLR);
        }
    else
        {
        /* start the timer as a one-shot*/

        OMAP35XX_TIMER_REG_SYNC_WRITE (pTimerData, TIMER_TCLR_OFFSET,
                                       0x401, TIMER_W_PEND_TCLR);
        OMAP35XX_TIMER_REG_SYNC_WRITE (pTimerData, TIMER_TMAR_OFFSET,
                                       1, TIMER_W_PEND_TMAR);
        }

    DBG_MSG (DBG_INFO, "omap35xxTimerEnable calling vxbIntEnable\n");

    (void) vxbIntEnable (pTimerData->pInst, pTimerData->intRes);

    DBG_MSG (DBG_INFO, "... called omap35xxTimerEnable\n");

    pTimerData->timerEnabled = TRUE;

    return OK;
    }

/*******************************************************************************
*
* vxbOmap3TimerAttach -attach routine of timer device
*
* This is attach routine of timer device
*
* RETURNS: OK or ERROR if failed.
*
* ERRNO: N/A
*/

LOCAL STATUS vxbOmap3TimerAttach
    (
    VXB_DEV_ID              pDev
    )
    {
    struct omap35xxTimerData     * pTimerData;
    struct vxbTimerFunctionality * pTimerFunc;
    VXB_CLK                      * pClk;
    UINT32                         val;
    VXB_RESOURCE                 * pRes;
    VXB_RESOURCE                 * pMemRes;
    VXB_FDT_DEV                  * pFdtDev;
    char                         * pMode;
    int                          offset;
    static UINT32                timerNo = 0;
    OMAP3_TIMER_CFG *            pCfg = NULL;

    DBG_MSG (DBG_INFO, "omap35xxTimerInstInit\n");

    /* check for valid parameter */

    if (pDev == NULL)
        return ERROR;

    vxbDevSoftcSet (pDev, NULL);  /* just in case an error occurs */
    pFdtDev = (VXB_FDT_DEV *)(vxbFdtDevGet(pDev));

    if (pFdtDev == NULL)
        return ERROR;

    offset = pFdtDev->offset;
    
    (void) vxbPinMuxEnable (pDev);
    
    /* enable timer clock */
    
    (void) vxbClkEnableAll(pDev);

    /* allocate the memory for the structure */

    pTimerData = (struct omap35xxTimerData *)
                  vxbMemAlloc (sizeof (struct omap35xxTimerData));

    /* check if memory allocation is successful */

    if (pTimerData == NULL)
        return ERROR;

    pMemRes = vxbResourceAlloc (pDev, VXB_RES_MEMORY, 0);

    if ((pMemRes == NULL) || (pMemRes->pRes == NULL))
        {
        vxbMemFree (pTimerData);
        return ERROR;
        }

    pMode = (char *) vxFdtPropGet (offset, "dmmode", 0);

    pTimerData->baseAddress = 
        (void*)(((VXB_RESOURCE_ADR *) (pMemRes->pRes))->virtual);
    pTimerData->vxbHandle =   ((VXB_RESOURCE_ADR *) (pMemRes->pRes))->pHandle;

    /*
     * On some processor, TIDR doesn't have fixed value, like OMAP35XX.
     * If "ti,omap35xx-dmtimer-1ms" is set, it will force timer module as
     * TIMER_MODULE_DMTIMER_1MS without reading TIDR.
     */

    pCfg = (OMAP3_TIMER_CFG *) vxbDevDrvDataGet (pDev);
    if (pCfg == NULL)
        {
        vxbMemFree (pTimerData);
        (void) vxbResourceFree(pDev,pMemRes);
        return ERROR;
        }
     
    if (pCfg->version == TIMER_VERSION_OMAP35XX)    
        pTimerData->timerModule = TIMER_MODULE_DMTIMER_1MS;
    else
        pTimerData->timerModule = TIMER_MODULE_UNKNOWN;

    if (pMode == NULL)  /* means we are working as normal timer */
        {
        pRes = vxbResourceAlloc (pDev, VXB_RES_IRQ, 0);

        if (pRes == NULL)
            {
            vxbMemFree (pTimerData);
            (void) vxbResourceFree(pDev,pMemRes);
            return ERROR;
            }

        pTimerData->intRes = pRes;

        /*
         * We are about to initialize the timer functionality structure.
         * Grab a pointer to it.
         */

        pTimerFunc = &(pTimerData->timerFunclty);

        /* get the minimum clock period */
        pTimerFunc->minFrequency = 1;

        pTimerFunc->maxFrequency = 8000;
        
        pClk = vxbClkGet (pDev, "fck");
		
		if (pClk == NULL)
            pClk = vxbClkGet (pDev, NULL);

        if (pClk == NULL ||
            (pTimerFunc->clkFrequency = (UINT32)vxbClkRateGet (pClk)) 
             == CLOCK_RATE_INVALID)
            pTimerFunc->clkFrequency = 24000000;

        /* Store the feature provided by a general purpose timer */

        pTimerFunc->features =  VXB_TIMER_CAN_INTERRUPT |
                                VXB_TIMER_INTERMEDIATE_COUNT |
                                VXB_TIMER_SIZE_32 |
                                VXB_TIMER_AUTO_RELOAD;

        /* store the ticksPerSecond of the timer */

        pTimerFunc->ticksPerSecond = 60;

        /*
         * For manual testing of the timer, the name of the timer is
         * initialized
         */

        strncpy(pTimerFunc->timerName , TIMER_NAME, MAX_DRV_NAME_LEN);

        /* update the timer rollover period */

        pTimerFunc->rolloverPeriod = OMAP35XX_MAX_COUNT_VAL /
                                     pTimerFunc->clkFrequency;

        /* populate the function pointers */

        pTimerFunc->timerAllocate    = omap35xxTimerAllocate;
        pTimerFunc->timerRelease     = omap35xxTimerRelease;
        pTimerFunc->timerRolloverGet = omap35xxTimerRolloverGet;
        pTimerFunc->timerCountGet    = omap35xxTimerCountGet;
        pTimerFunc->timerDisable     = omap35xxTimerDisable;
        pTimerFunc->timerEnable      = omap35xxTimerEnable;
        pTimerFunc->timerISRSet      = omap35xxTimerISRSet;

        pTimerData->rollover = (pTimerFunc->clkFrequency) /
            (pTimerFunc->ticksPerSecond);

        /*
         * Caution: Do not put the overflow value (0xffffffff) in the GPTi.TLDR
         * register because it can lead to undesired results
         */

        if (pTimerData->rollover == 0)
            pTimerData->rollover = 1;

        /* detect timer module */
  
        if (vxbOmap3TimerModuleDetect (pTimerData) == ERROR)
            {
            vxbMemFree (pTimerData);
            (void) vxbResourceFree(pDev, pMemRes);
            return ERROR;
            }
      
        OMAP35XX_TIMER_REG_READ (pTimerData, TIMER_TSICR_OFFSET, val);

        if (val & TIMER_POSTED)
            pTimerData->isPosted = TRUE;
        else
            pTimerData->isPosted = FALSE;

        pTimerFunc->timerNo = timerNo++;
        }
    else if(strncmp(pMode, "capture", 10) == 0) /* we are in capture mode */
        {
        /*
         * Only a workaround for CPSW EMAC, never make this timer work as
         * true timer. Following code is making the Timer5/6 working as
         * capture mode.
         */

        UINT32 regVal;
        
        /* detect timer module */
        
        if (vxbOmap3TimerModuleDetect (pTimerData) == ERROR)
            {
            vxbMemFree (pTimerData);
            (void) vxbResourceFree(pDev, pMemRes);
            return ERROR;
            }
        
        /* since this error only exist in am335x, we use dm81xx reg */

        OMAP35XX_TIMER_REG_WRITE (pTimerData, DM81XX_TIMER_TIOCP_CFG_OFFSET,
                                  0x1);

        /* wait for reset to complete */

        do
            {
            OMAP35XX_TIMER_REG_READ (pTimerData, DM81XX_TIMER_TIOCP_CFG_OFFSET,
                                     regVal);
            }
        while (regVal & 0x1);

        OMAP35XX_TIMER_REG_WRITE (pTimerData, DM81XX_TIMER_IRQSTATUS_OFFSET,
                                  0x7);
        OMAP35XX_TIMER_REG_WRITE (pTimerData,
                                  DM81XX_TIMER_IRQSTATUS_CLEAR_OFFSET, 0x7);

        OMAP35XX_TIMER_REG_WRITE (pTimerData, DM81XX_TIMER_IRQSTATUS_SET_OFFSET,
                                  DM81XX_TCAR_EN_FLAG);

        OMAP35XX_TIMER_REG_SYNC_READ (pTimerData, TIMER_TCLR_OFFSET, regVal,
                                      TIMER_W_PEND_TCLR);
        regVal &= ~DM81XX_CAPT_MODE;
        regVal |= DM81XX_CAPT_LOW_TO_HIGH;
        OMAP35XX_TIMER_REG_SYNC_WRITE (pTimerData, TIMER_TCLR_OFFSET, regVal,
                                       TIMER_W_PEND_TCLR);

        vxbDevSoftcSet (pDev, (void *) pTimerData);
        return OK;
        }
    else
        {
        vxbMemFree (pTimerData);
        (void) vxbResourceFree(pDev, pMemRes);
        return ERROR;
        }

    vxbDevSoftcSet (pDev, (void *) pTimerData);

    pTimerData->pInst = pDev;

    (void) vxbIntConnect (pDev, pRes, vxbOmap35xxTimerClkInt, pDev);

    vxbTimerRegister(pTimerFunc);

    DBG_MSG (DBG_INFO, "... end omap35xxTimerAttach\n");

    return OK;
    }

/**************************************************************************
*
* omap35xxOneShotEnable - clear wakeup time
*
* This routine resets the system clock timer for an expiration that
* is a programmable time in the future.  This is used as part of power
* management.
* If nTicks == 0, wait forever (or as long as possible)
*
* RETURNS: past ticks
*
* ERRNO: N/A
*\NOMANUAL
*/

void omap35xxOneShotEnable
    (
    VXB_DEV_ID pInst,
    UINT32     nTicks
    )
    {
    UINT32 sleepTicks;
    UINT32 isrStatus;
    struct omap35xxTimerData * pTimerData;

    if (pInst == NULL)
        {
        return; /* ERROR */
        }

    /*
     * check if sys clk timer was initialized and enabled
     */

    pTimerData = (struct omap35xxTimerData *)vxbDevSoftcGet(pInst);

    if (!pTimerData || !pTimerData->timerEnabled )
        {
        return; /* ERROR */
        }

    /*
     * we want to sleep one tick less than what was requested, plus whatever
     * fraction of a tick remains on the system clock
     *
     * hardware limitation requires clamping this number
     */

    if ((nTicks == 0 ) || (nTicks >= OMAP35XX_MAX_MATCH_VAL))
        {
        sleepTicks = OMAP35XX_MAX_MATCH_VAL;
        }
    else
        {
        /*
         * advance the number of requested ticks
         */

        sleepTicks = nTicks - 1;    /* Caller will not pass nTicks == 1 */
        }

    /* write the match count register */

    OMAP35XX_TIMER_REG_SYNC_WRITE(pTimerData, OMAP35XX_TIMER_TOWR_OFFSET,
        sleepTicks, TIMER_W_PEND_TOWR);

    /*
     * Detect rollover failure
     * The count register may be in the process of rolling over.  An interrupt
     * pending at this point means that the timer will generate an interrupt
     * and needs to be serviced.  The number of ticks elapsed is only one
     * however, the rollover tick.
     */

    /* get the current ISR status */

    OMAP35XX_TIMER_REG_READ (pTimerData, OMAP35XX_TIMER_TISR_OFFSET,
                             isrStatus);

    /* If interrupt is pending */

    if ((isrStatus & 0x2) != 0)
        {

        /*
         * Clear the match count register.  The timer will service the
         * interrupt and advance system time by one-tick.  Since
         * the omap35xxOneShotDisable uses the match count to figure out
         * how long its been sleeping, we need to clear it out.  It hasn't
         * slept.
         */

        OMAP35XX_TIMER_REG_WRITE (pTimerData,
                                  OMAP35XX_TIMER_TOWR_OFFSET, 0);

        }
    }

/**************************************************************************
*
* omap35xxOneShotDisable - system clock clear
*
* This routine turns off the sleep timer.  This is used as part of power
* management.
*
* RETURNS: the number of ticks the system slept.
*
* ERRNO: N/A
*\NOMANUAL
*/

UINT32 omap35xxOneShotDisable
    (
    VXB_DEV_ID pInst
    )
    {
    UINT32 elapsedCount;
    UINT32 isrStatus;
    UINT32 ticksExpired;
    struct omap35xxTimerData *pTimerData;

    if (pInst == NULL)
        {
        return 0; /* ERROR */
        }

    /*
     * check if sys clk timer was initialized and enabled
     */

    pTimerData = (struct omap35xxTimerData *)vxbDevSoftcGet(pInst);

    if (!pTimerData || !pTimerData->timerEnabled )
        {
        return 0; /* ERROR */
        }

    /* get the current ISR status */

    OMAP35XX_TIMER_REG_READ (pTimerData, OMAP35XX_TIMER_TISR_OFFSET,
                             isrStatus);

    /*
     * Check the timer interrupt pending status
     */

    if (isrStatus & 0x2)
        {
        /*
         * Timer interrupt, the number of ticks expired is equal
         * to what we requested.
         */

        /* read the match count register */

        OMAP35XX_TIMER_REG_SYNC_READ(pTimerData, OMAP35XX_TIMER_TOWR_OFFSET,
            ticksExpired, TIMER_W_PEND_TOWR);

        /*
         * When the timer expires, the TOCR resets to zero so TOWR is invalid
         * The matchCount has the number of ticks that were skipped, so the
         * total number is matchCount+1.  We want to report one less however
         * because the timer ISR will call windTickAnnounce for one tick.
         */

        OMAP35XX_TIMER_REG_SYNC_READ (pTimerData, OMAP35XX_TIMER_TOCR_OFFSET,
                                      elapsedCount, TIMER_W_PEND_TOCR);
        }
    else
        {
        /*
         * Some of the ticks expired, return expired ticks
         */

        /*
         * read how much time remains on the current one shot timer. TOCR
         * increments till it is equal to TOWR, then reset to 0 and starts
         * incrementing again
         */

        OMAP35XX_TIMER_REG_SYNC_READ (pTimerData, OMAP35XX_TIMER_TOCR_OFFSET,
                                      ticksExpired, TIMER_W_PEND_TOCR);
        }

    /* while awake, interrupt with each clock tick */

    OMAP35XX_TIMER_REG_SYNC_WRITE (pTimerData, OMAP35XX_TIMER_TOCR_OFFSET, 0, 
                                   TIMER_W_PEND_TOCR);
    OMAP35XX_TIMER_REG_SYNC_WRITE (pTimerData, OMAP35XX_TIMER_TOWR_OFFSET, 0, 
                                   TIMER_W_PEND_TOWR);

    /*
     * Detect rollover failure
     * The count register was zero'd above and the match register
     * was set to zero.  If the count register subsequently incremented,
     * there should be an interrupt pending.  In the failure mode,
     * the timer rolled over but no match was detected (i.e. no interrupt).
     * If we don't correct for this, we won't get a timer interrupt until
     * it rolls over again.
     */

    OMAP35XX_TIMER_REG_SYNC_READ (pTimerData, OMAP35XX_TIMER_TOCR_OFFSET,
                                  elapsedCount, TIMER_W_PEND_TOCR);
    OMAP35XX_TIMER_REG_READ (pTimerData, OMAP35XX_TIMER_TISR_OFFSET,
                             isrStatus);

    /* If the counter rolled over but no interrupt is pending */

    if ((elapsedCount != 0) && ((isrStatus & 0x2) == 0))
        {
        ticksExpired++;     /* Missed tick, increment count */
        OMAP35XX_TIMER_REG_SYNC_WRITE (pTimerData,    /* clear overflow count */
                                       OMAP35XX_TIMER_TOCR_OFFSET, 0,
                                       TIMER_W_PEND_TOCR);

        }

    return (ticksExpired); /* return ticks expired */
    }

/******************************************************************************
*
* vxbOmap3TimerModuleDetect - detect omap3 timer module
*
* This is the omap3 timer module initialization routine. It will read the timer
* ID register first and compare it with known Ti timer ID to identify the timer
* module.
*
* RETURNS: OK or ERROR when parameter error or unknown timer module is detected.
*
* ERRNO
*/

LOCAL STATUS vxbOmap3TimerModuleDetect
    (
    OMAP35XX_TIMER_DATA *pTimerData
    )
    {
    UINT32      regVal;

    DBG_MSG (DBG_INFO, "vxbOmap3TimerModuleDetect\n");

    if (pTimerData == NULL)
        return ERROR;

    /* 
     * If timer module is already determined, skip timer module detection.
     *
     * Note:
     *     OMAP35XX timer id do not have fixed TIDR, in order to support them 
     *     in the future, driver needs set timer module as 
     *     TIMER_MODULE_DMTIMER_1MS before call vxbOmap3TimerModuleDetect ().
     */

    if (pTimerData->timerModule != TIMER_MODULE_UNKNOWN)
        {
        pTimerData->pChipRegs = omap35xxRegs;
        return OK;
        }
        
    OMAP35XX_TIMER_REG_READ (pTimerData, OMAP35XX_TIMER_ID_OFFSET, regVal);
    
    switch (regVal)
        {
        case AM437X_DMTIMER_ID:
        case AM335X_DMTIMER_ID:
        case AM389X_DMTIMER_ID:
        case AM57XX_DMTIMER_ID:
        case AM57XX_DMTIMER_1MS_ID:
        case AM65XX_DMTIMER_ID:
            pTimerData->pChipRegs = dm81xxRegs;
            pTimerData->timerModule = TIMER_MODULE_DMTIMER;
            break;
        case AM437X_DMTIMER_1MS_ID:
            pTimerData->pChipRegs = omap35xxRegs;
            pTimerData->timerModule = TIMER_MODULE_DMTIMER_1MS;
            break;
        default:
            DBG_MSG (DBG_INFO, "unknown timer module\n");
            pTimerData->timerModule = TIMER_MODULE_UNKNOWN;
            return ERROR;
        }

    return OK;
    }

/******************************************************************************
*
* ofOmap3TimerProbe - probe for device presence at specific address
*
* Check for Omap3 timer contoller (or compatible) device at the specified
* base address.  
*
* RETURNS: OK if probe passes. ERROR otherwise.
*
*/

LOCAL STATUS ofOmap3TimerProbe
    (
    struct vxbDev * pDev /* Device information */
    )
    {
    VXB_FDT_DEV_MATCH_ENTRY * pMatch;
    OMAP3_TIMER_CFG * pCfg;

    if (vxbFdtDevMatch (pDev, dmtimerMatch, &pMatch) == ERROR)
        return ERROR;

    pCfg = (OMAP3_TIMER_CFG *) pMatch->data;

    vxbDevDrvDataSet(pDev,(void *) pCfg);
    
    return OK;
    }

/******************************************************************************
*
* ofOmap3TimerAttach - attach omap3 timer device
*
* This is the omap3 timer initialization routine.
*
* RETURNS: OK or ERROR when initialize failed
*
* ERRNO
*/

LOCAL STATUS ofOmap3TimerAttach
    (
    VXB_DEV_ID pDev
    )
    {
    if (vxbOmap3TimerAttach (pDev) != OK)
        return ERROR;

    return OK;
    }    
