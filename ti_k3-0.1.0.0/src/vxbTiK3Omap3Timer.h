/* vxbTiK3Omap3Timer.h - vxbus OMAP3/AM3 timer interfaces header file */

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
This file provides the OMAP timers specific definitions.
*/

#ifndef __INCvxbTiK3Omap3Timerh
#define __INCvxbTiK3Omap3Timerh

#include <hwif/vxBus.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of timers */

#define OMAP35XX_MAX_TIMERS                 12

/*
 * These base addresses are used for purposes of identifying which timer is
 * being used
 */

#define OMAP35XX_BASE_ADDR_GPTIMER1         0x48318000
#define OMAP35XX_BASE_ADDR_GPTIMER2         0x49032000
#define OMAP35XX_BASE_ADDR_GPTIMER3         0x49034000
#define OMAP35XX_BASE_ADDR_GPTIMER4         0x49036000
#define OMAP35XX_BASE_ADDR_GPTIMER5         0x49038000
#define OMAP35XX_BASE_ADDR_GPTIMER6         0x4903A000
#define OMAP35XX_BASE_ADDR_GPTIMER7         0x4903C000
#define OMAP35XX_BASE_ADDR_GPTIMER8         0x4903E000
#define OMAP35XX_BASE_ADDR_GPTIMER9         0x49040000
#define OMAP35XX_BASE_ADDR_GPTIMER10        0x48086000
#define OMAP35XX_BASE_ADDR_GPTIMER11        0x48088000
#define OMAP35XX_BASE_ADDR_GPTIMER12        0x48304000
#define OMAP35XX_BASE_ADDR_CM_CLKOUT_CTRL   0x48004D70

/* GP Timer offsets */

#define OMAP35XX_TIMER_TIOCP_CFG_OFFSET     0x010
#define OMAP35XX_TIMER_TISTAT_OFFSET        0x014
#define OMAP35XX_TIMER_TISR_OFFSET          0x018
#define OMAP35XX_TIMER_TIER_OFFSET          0x01C
#define OMAP35XX_TIMER_TWER_OFFSET          0x020
#define OMAP35XX_TIMER_TCLR_OFFSET          0x024
#define OMAP35XX_TIMER_TCRR_OFFSET          0x028
#define OMAP35XX_TIMER_TLDR_OFFSET          0x02C
#define OMAP35XX_TIMER_TTGR_OFFSET          0x030
#define OMAP35XX_TIMER_TWPS_OFFSET          0x034
#define OMAP35XX_TIMER_TMAR_OFFSET          0x038
#define OMAP35XX_TIMER_TCAR1_OFFSET         0x03C
#define OMAP35XX_TIMER_TSICR_OFFSET         0x040
#define OMAP35XX_TIMER_TCAR2_OFFSET         0x044
#define OMAP35XX_TIMER_TPIR_OFFSET          0x048
#define OMAP35XX_TIMER_TNIR_OFFSET          0x04C
#define OMAP35XX_TIMER_TCVR_OFFSET          0x050
#define OMAP35XX_TIMER_TOCR_OFFSET          0x054
#define OMAP35XX_TIMER_TOWR_OFFSET          0x058

/* DM81xx GP Timer offsets */

#define DM81XX_TIMER_TIOCP_CFG_OFFSET       0x010
#define DM81XX_TIMER_IRQSTATUS_OFFSET       0x028
#define DM81XX_TIMER_IRQSTATUS_SET_OFFSET   0x02C
#define DM81XX_TIMER_IRQSTATUS_CLEAR_OFFSET 0x030
#define DM81XX_TIMER_IRQWAKEEN_OFFSET       0x034
#define DM81XX_TIMER_TCLR_OFFSET            0x038
#define DM81XX_TIMER_TCRR_OFFSET            0x03C
#define DM81XX_TIMER_TLDR_OFFSET            0x040
#define DM81XX_TIMER_TTGR_OFFSET            0x044
#define DM81XX_TIMER_TWPS_OFFSET            0x048
#define DM81XX_TIMER_TMAR_OFFSET            0x04C
#define DM81XX_TIMER_TCAR1_OFFSET           0x050
#define DM81XX_TIMER_TSICR_OFFSET           0x054
#define DM81XX_TIMER_TCAR2_OFFSET           0x058

#define DM81XX_TCAR_EN_FLAG                 (0x1u << 2)
#define DM81XX_CAPT_MODE                    (0x1u << 13)
#define DM81XX_CAPT_LOW_TO_HIGH             (0x1u << 8)
#define DM81XX_CAPT_HIGH_TO_LOW             (0x2u << 8)
#define DM81XX_CAPT_BOTH                    (0x3u << 8)

#define TIMER_W_PEND_TCLR                   0x1
#define TIMER_W_PEND_TCRR                   0x2
#define TIMER_W_PEND_TLDR                   0x4
#define TIMER_W_PEND_TTGR                   0x8
#define TIMER_W_PEND_TMAR                   0x10
#define TIMER_W_PEND_TPIR                   0x20
#define TIMER_W_PEND_TNIR                   0x40
#define TIMER_W_PEND_TCVR                   0x80
#define TIMER_W_PEND_TOCR                   0x100
#define TIMER_W_PEND_TOWR                   0x200

#define TIMER_POSTED                        0x4

#define OMAP35XX_TIMER_ID_OFFSET            (0x0)

/* AM437x Timer ID */

#define AM437X_DMTIMER_ID                   (0x4FFF0301)
#define AM437X_DMTIMER_1MS_ID               (0x15)

/* AM335x Timer ID */

#define AM335X_DMTIMER_ID                   (0x40001000)
#define AM335X_DMTIMER_1MS_ID               (0x15)

/* AM389x Timer ID */

#define AM389X_DMTIMER_ID                   (0x4FFF1301)

/* AM57XX Timer ID */

#define AM57XX_DMTIMER_ID                   (0x4FFF2b01)
#define AM57XX_DMTIMER_1MS_ID               (0x50002100)

/* AM65XX Timer ID */

#define AM65XX_DMTIMER_ID                   (0x50003100)

/* OMAP35XX doesn't have fixed TIDR */

#define TIMER_VERSION_UNUSED                (0x0)
#define TIMER_VERSION_OMAP35XX              (0x1)

#define TIMER_MODULE_UNKNOWN                (0xFFFFFFFF)
#define TIMER_MODULE_DMTIMER                (0x1)
#define TIMER_MODULE_DMTIMER_1MS            (0x2)

/* structure to store the timer information */

/* define the power domain of the timer */

enum omapTimerPowerDomain
    {
    OMAP_TIMER_PWR_WKUP,
    OMAP_TIMER_PWR_PER,
    OMAP_TIMER_PWR_CORE
    };

typedef struct omap35xxTimerData
    {
    VXB_DEV_ID                   pInst;
    void *                       baseAddress;      /* base address of timer */
    void *                       vxbHandle;        /* vxBus handle */
    struct vxbTimerFunctionality timerFunclty;     /* capabilities of driver */
    void                         (*pIsrFunc)(_Vx_usr_arg_t); /* BSP provides ISR Func */
    _Vx_usr_arg_t                arg;              /* ISR func argument */
    BOOL                         timerEnabled;     /* true if enabled */
    BOOL                         errorCorrect1ms;  /* GP1,2,10 1ms exactly */
    BOOL                         autoReload;       /* auto-reload timer */
    enum omapTimerPowerDomain    powerDomain;      /* Power management domain */
    UINT32                       rollover;         /* rollover value */
    UINT32                       positiveIncrement1ms; /* 1ms correction */
    UINT32                       negativeIncrement1ms; /* 1ms correction */
    UINT32                       timerID;          /* timer ID register value */
    UINT32                       timerModule;      /* timer module type */
    UINT32 *                     pChipRegs;
    BOOL                         isPosted;
    VXB_RESOURCE *               intRes;
    } OMAP35XX_TIMER_DATA;

#define TIMERFUNC_TO_TIMERDATA(pTimerFunc)   \
                    (struct omap35xxTimerData *)((ULONG) (pTimerFunc) - \
                     OFFSET (struct omap35xxTimerData, timerFunclty))

typedef struct Omap3TimerCfg
    {
    UINT32 version;
    } OMAP3_TIMER_CFG;

/* prototypes */

#ifndef _ASMLANGUAGE
IMPORT void omap35xxOneShotEnable (VXB_DEV_ID pInst, UINT32 nTicks);
IMPORT UINT32 omap35xxOneShotDisable (VXB_DEV_ID pInst);
#endif /* _ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* __INCvxbTiK3Omap3Timerh */

