/* vxbTiK3NavssUdma.c - TI AM65x MCU NAVSS UDMA Module driver */

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
20jun19,hkl  created (VXWPG-114)
*/

/*
DESCRIPTION
This module implements the dedicated MCU NAVSS UDMA driver to support MCU CPSW
MAC transmitting and receiving. This driver will create two specific DMA 
channels for TX & RX. The MCU CPSW will use them to send or recevie packets.

The MCU NAVSS UDMA driver will rely on MCU NVASS RingAcc driver (which is in
vxbTiK3NavssRingacc.c) and UDMA descriptor driver (which is in 
vxbTiK3NavssUdmaDesc.c) to bind ring queues and construct the packet descriptor.

The detail resource allocation of TX & RX DMA channels is:
TX DMA channel ID: 0, transmit ring ID: 0, transmit compeletion ring ID: 96;
RX DMA channel ID: 0, free descriptor ring ID: 48, receive ring ID: 97.

INCLUDE FILES: vxWorks.h stdio.h string.h vmLib.h cacheLib.h pmapLib.h
               vmLibCommon.h vxbTimerLib.h vxbAccessArchLib.h vxBus.h
               tiSciPredefs.h vxbTiK3NavssUdmaDesc.h vxbTiK3NavssRingacc.h
               vxbTiK3NavssUdma.h
*/

/* includes */

#include <vxWorks.h>
#include <stdio.h>
#include <string.h>
#include <vmLib.h>
#include <cacheLib.h>
#include <pmapLib.h>
#include <vmLibCommon.h>
#include <subsys/timer/vxbTimerLib.h>
#include <arch/arm/vxbAccessArchLib.h>
#include <hwif/vxBus.h>
#include <tiSciClientApi.h>
#include "vxbTiK3NavssUdmaDesc.h"
#include "vxbTiK3NavssRingacc.h"
#include "vxbTiK3NavssUdma.h"

/* defines */

/* debug macro */

#undef  DEBUG_MSG
#undef  K3_NAVSS_UDMA_DBG
#ifdef  K3_NAVSS_UDMA_DBG

/* turning local symbols into global symbols */

#ifdef  LOCAL
#undef  LOCAL
#define LOCAL
#endif

#include <private/kwriteLibP.h>         /* _func_kprintf */
#define K3_NAVSS_UDMA_DBG_OFF           0x00000000
#define K3_NAVSS_UDMA_DBG_ISR           0x00000001
#define K3_NAVSS_UDMA_DBG_ERR           0x00000002
#define K3_NAVSS_UDMA_DBG_INFO          0x00000004
#define K3_NAVSS_UDMA_DBG_ALL           0xffffffff

LOCAL UINT32 k3NavssUdmaDbgMask = K3_NAVSS_UDMA_DBG_ALL;

#define DEBUG_MSG(mask, ...)                                                  \
    do                                                                        \
        {                                                                     \
        if ((k3NavssUdmaDbgMask & (mask)) \
            || ((mask) == K3_NAVSS_UDMA_DBG_ALL))  \
            {                                                                 \
            if (_func_kprintf != NULL)                                        \
                {                                                             \
                (* _func_kprintf) (__VA_ARGS__);                              \
                }                                                             \
            }                                                                 \
        }                                                                     \
    while (FALSE)
#else
#define DEBUG_MSG(...)
#endif  /* K3_NAVSS_UDMA_DBG */

/* register access */

#ifdef ARMBE8
#    define SWAP32 vxbSwap32
#    define SWAP64 vxbSwap64
#else
#    define SWAP32
#    define SWAP64
#endif /* ARMBE8 */

#undef REG_READ_4
#define REG_READ_4(regBase, offset)                                     \
        SWAP32 (* ((volatile UINT32 *) ((UINT8 *) regBase + offset)))

#undef REG_WRITE_4
#define REG_WRITE_4(regBase, offset, value)                             \
        * ((volatile UINT32 *) ((UINT8 *) regBase + offset)) = SWAP32 (value)

#undef REG_SETBIT_4
#define REG_SETBIT_4(regBase, offset, data)                             \
        REG_WRITE_4 (regBase, offset,                                   \
                     SWAP32 ((UINT32) (data) | REG_READ_4 (regBase, offset)))

#undef REG_CLEARBIT_4
#define REG_CLEARBIT_4(regBase, offset, mask)                           \
        REG_WRITE_4 (regBase, offset,                                   \
                     SWAP32 ((UINT32) (~(mask)) & REG_READ_4 (regBase, offset)))

#define UDMA_STOP_CHAN_TIMEOUT          (1000)

/* typedefs */

typedef enum udmaPhyAddrType
    {
    UDMA_PHY_ADDR_TX_DESC_TYPE = 0,
    UDMA_PHY_ADDR_RX_DESC_TYPE,
    UDMA_PHY_ADDR_RX_BUF_TYPE
    } UDMA_PHY_ADDR_TYPE;

/* forward declarations */

LOCAL PHYS_ADDR udmaVirToPhy (VIRT_ADDR virtAdrs);
LOCAL VIRT_ADDR udmaPhyToVir (TI_K3_NAVSS_UDMA_CHAN * pChan,
                              UDMA_PHY_ADDR_TYPE phyAdrsType,
                              PHYS_ADDR phyAdrs);
LOCAL BOOL tiK3NavssUdmaChanIsRun (TI_K3_NAVSS_UDMA_CHAN * pChan);
LOCAL STATUS tiK3NavssUdmaStopDev2Mem (TI_K3_NAVSS_UDMA_CHAN * pChan);
LOCAL STATUS tiK3NavssUdmaStopMem2Dev (TI_K3_NAVSS_UDMA_CHAN * pChan);
LOCAL STATUS tiK3NavssUdmaStop (TI_K3_NAVSS_UDMA_CHAN * pChan);
LOCAL STATUS tiK3NavssUdmaHardStop (TI_K3_NAVSS_UDMA_CHAN * pChan);
LOCAL void tiK3NavssUdmaRstRings (TI_K3_NAVSS_UDMA_CHAN * pChan);
LOCAL void tiK3NavssUdmaRstCounters (TI_K3_NAVSS_UDMA_CHAN * pChan);
LOCAL STATUS tiK3NavssUdmaStart (TI_K3_NAVSS_UDMA_CHAN * pChan);
LOCAL STATUS tiK3NavssUdmaRxchanSciCfg (TI_K3_NAVSS_UDMA_CHAN * pRxChan);
LOCAL STATUS tiK3NavssUdmaTxChanSciCfg (TI_K3_NAVSS_UDMA_CHAN * pTxChan);
LOCAL STATUS tiK3NavssUdmaRxChanCfg (TI_K3_NAVSS_UDMA_CHAN * pRxChan);
LOCAL STATUS tiK3NavssUdmaTxChanCfg (TI_K3_NAVSS_UDMA_CHAN * pTxChan);
LOCAL STATUS tiK3NavssUdmaEnable (TI_K3_NAVSS_UDMA_CHAN * pChan);

/* locals */

LOCAL TI_K3_NAVSS_UDMA_DRVCTRL tiK3NavssUdmaDrv = {0};

/*******************************************************************************
*
* udmaPhyToVir - translate the virtual address to physical address
*
* This routine translates the virtual address to physical address.
*
* RETURNS: The physical address.
*
* ERRNO: N/A
*/

LOCAL PHYS_ADDR udmaVirToPhy
    (
    VIRT_ADDR virtAdrs
    )
    {
#ifdef _WRS_CONFIG_UNSUPPORTS_MMU
    return (PHYS_ADDR) virtAdrs;
#else
    PHYS_ADDR physAddr;
    (void) vmTranslate (NULL, virtAdrs, &physAddr);
    return physAddr;
#endif
    }

/*******************************************************************************
*
* udmaPhyToVir - translate the physical address to virtual address
*
* This routine translates the physical address to virtual address.
*
* RETURNS: The virtual address, or NULL if translate failed.
*
* ERRNO: N/A
*/

LOCAL VIRT_ADDR udmaPhyToVir
    (
    TI_K3_NAVSS_UDMA_CHAN * pChan,
    UDMA_PHY_ADDR_TYPE      phyAdrsType,
    PHYS_ADDR               phyAdrs
    )
    {
#ifdef _WRS_CONFIG_UNSUPPORTS_MMU
    return (VIRT_ADDR) phyAdrs;
#else
    switch (phyAdrsType)
        {
        case UDMA_PHY_ADDR_TX_DESC_TYPE:
            if (phyAdrs < pChan->txDescPhyBase)
                {
                return NULL;
                }

            return (VIRT_ADDR) pChan->pTxDesc + 
                   (phyAdrs - pChan->txDescPhyBase);

        case UDMA_PHY_ADDR_RX_DESC_TYPE:
            if (phyAdrs < pChan->rxDescPhyBase)
                {
                return NULL;
                }

            return (VIRT_ADDR) pChan->pRxDesc + 
                   (phyAdrs - pChan->rxDescPhyBase);

        case UDMA_PHY_ADDR_RX_BUF_TYPE:
            if (phyAdrs < pChan->rxBufPhyBase)
                {
                return NULL;
                }

            return (VIRT_ADDR) pChan->pRxBuf +
                   (phyAdrs - pChan->rxBufPhyBase);

        default:
            DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR, 
                       "udmaPhyToVir error type:%d!\n", phyAdrsType);
            return NULL;
        }
#endif
    }

/*******************************************************************************
*
* tiK3NavssUdmaChanIsRun - check the DMA channel running state
*
* This routine checks whether the specifc DMA channel is running.
*
* RETURNS: TRUE, or FALSE if it is not running.
*
* ERRNO: N/A
*/

LOCAL BOOL tiK3NavssUdmaChanIsRun
    (
    TI_K3_NAVSS_UDMA_CHAN * pChan
    )
    {
    UINT32  trtCtl = 0;
    UINT32  rrtCtl = 0;

    switch (pChan->dir) 
        {
        case UDMA_DEV_TO_MEM:
            rrtCtl = REG_READ_4 (pChan->pRchan->rchanRegBase,
                                 TI_K3_NAVSS_UDMA_RCHAN_RT_CTL_OFS);
            if (rrtCtl & TI_K3_NAVSS_UDMA_CHAN_RT_CTL_EN)
                {
                return TRUE;
                }
            break;

        case UDMA_MEM_TO_DEV:
            trtCtl = REG_READ_4 (pChan->pTchan->tchanRegBase,
                                 TI_K3_NAVSS_UDMA_TCHAN_RT_CTL_OFS);
            if (trtCtl & TI_K3_NAVSS_UDMA_CHAN_RT_CTL_EN)
                {
                return TRUE;
                }
            break;

        default:
            break;
        }

    DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
               "ERROR! %s() Line:%d\n", __func__, __LINE__);
    return FALSE;
    }

/*******************************************************************************
*
* tiK3NavssUdmaStopDev2Mem - stop a DMA RX channel
*
* This routine stops the specific DMA RX chennel by requeting channel teardown.
*
* RETURNS: OK, or ERROR if stop failed.
*
* ERRNO: N/A
*/

LOCAL STATUS tiK3NavssUdmaStopDev2Mem
    (
    TI_K3_NAVSS_UDMA_CHAN * pChan
    )
    {
    UINT32  i = 0;
    UINT32  regVal;

    REG_WRITE_4 (pChan->pRchan->rchanRegBase, 
                 TI_K3_NAVSS_UDMA_RCHAN_RT_PEER_RT_EN_OFS,
                 (TI_K3_NAVSS_UDMA_PEER_RT_EN_ENABLE | 
                 TI_K3_NAVSS_UDMA_PEER_RT_EN_TEARDOWN));

    while (i < UDMA_STOP_CHAN_TIMEOUT) 
        {
        regVal = REG_READ_4 (pChan->pRchan->rchanRegBase, 
                             TI_K3_NAVSS_UDMA_RCHAN_RT_CTL_OFS);
        if ((regVal & TI_K3_NAVSS_UDMA_CHAN_RT_CTL_EN) == 0) 
            {
            regVal = REG_READ_4 (pChan->pRchan->rchanRegBase, 
                                 TI_K3_NAVSS_UDMA_RCHAN_RT_PEER_RT_EN_OFS);
            if (regVal & TI_K3_NAVSS_UDMA_PEER_RT_EN_ENABLE)
                {
                DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR, 
                           "%s() Line:%d peer stop TIMEOUT!\n", __func__, __LINE__);
                return ERROR;
                }
            return OK;
            }

        i++;
        vxbUsDelay (1);
        }

    DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR, "%s() TIMEOUT!\n", __func__);
    return ERROR;
    }

/*******************************************************************************
*
* tiK3NavssUdmaStopMem2Dev - stop a DMA TX channel
*
* This routine stops the specific DMA TX chennel by requeting channel teardown.
*
* RETURNS: OK, or ERROR if stop failed.
*
* ERRNO: N/A
*/

LOCAL STATUS tiK3NavssUdmaStopMem2Dev
    (
    TI_K3_NAVSS_UDMA_CHAN * pChan
    )
    {
    UINT32  i = 0;
    UINT32  regVal;

    REG_WRITE_4 (pChan->pTchan->tchanRegBase, TI_K3_NAVSS_UDMA_TCHAN_RT_CTL_OFS,
                 TI_K3_NAVSS_UDMA_CHAN_RT_CTL_EN |
                 TI_K3_NAVSS_UDMA_CHAN_RT_CTL_TDOWN);

    while (i < UDMA_STOP_CHAN_TIMEOUT) 
        {
        regVal = REG_READ_4 (pChan->pTchan->tchanRegBase, 
                             TI_K3_NAVSS_UDMA_TCHAN_RT_CTL_OFS);
        if ((regVal & TI_K3_NAVSS_UDMA_CHAN_RT_CTL_EN) == 0)
            {
            regVal = REG_READ_4 (pChan->pTchan->tchanRegBase, 
                                 TI_K3_NAVSS_UDMA_TCHAN_RT_PEER_RT_EN_OFS);
            if (regVal & TI_K3_NAVSS_UDMA_PEER_RT_EN_ENABLE)
                {
                DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR, 
                           "%s() Line:%d peer not stopped TIMEOUT !\n",
                           __func__, __LINE__);
                return ERROR;
                }

            return OK;
            }

        i++;
        vxbUsDelay (1);
        }

    DEBUG_MSG(K3_NAVSS_UDMA_DBG_ERR, "%s() Line:%d TIMEOUT!\n", __func__, __LINE__);
    return ERROR;
    }

/*******************************************************************************
*
* tiK3NavssUdmaStop - stop a DMA channel
*
* This routine stops the specific DMA chennel by requeting channel teardown.
*
* RETURNS: OK, or ERROR if stop failed.
*
* ERRNO: N/A
*/

LOCAL STATUS tiK3NavssUdmaStop
    (
    TI_K3_NAVSS_UDMA_CHAN * pChan
    )
    {
    switch (pChan->dir)
        {
        case UDMA_DEV_TO_MEM:
            return tiK3NavssUdmaStopDev2Mem (pChan);

        case UDMA_MEM_TO_DEV:
            return tiK3NavssUdmaStopMem2Dev (pChan);

        default:
            DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                       "ERROR! %s() Line:%d\n", __func__, __LINE__);
            return ERROR;
        }

    return OK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaHardStop - stop a DMA channel
*
* This routine disables the specific DMA chennel.
*
* RETURNS: OK, or ERROR if stop failed.
*
* ERRNO: N/A
*/

LOCAL STATUS tiK3NavssUdmaHardStop
    (
    TI_K3_NAVSS_UDMA_CHAN * pChan
    )
    {
    switch (pChan->dir) 
        {
        case UDMA_DEV_TO_MEM:
            REG_WRITE_4 (pChan->pRchan->rchanRegBase, 
                         TI_K3_NAVSS_UDMA_RCHAN_RT_PEER_RT_EN_OFS, 0);
            REG_WRITE_4 (pChan->pRchan->rchanRegBase,
                         TI_K3_NAVSS_UDMA_RCHAN_RT_CTL_OFS, 0);
            break;

        case UDMA_MEM_TO_DEV:
            REG_WRITE_4 (pChan->pTchan->tchanRegBase, 
                         TI_K3_NAVSS_UDMA_TCHAN_RT_CTL_OFS, 0);
            REG_WRITE_4 (pChan->pTchan->tchanRegBase, 
                         TI_K3_NAVSS_UDMA_TCHAN_RT_PEER_RT_EN_OFS, 0);
            break;

        default:
            DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                       "ERROR! %s() Line:%d\n", __func__, __LINE__);
            return ERROR;
        }

    return OK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaRstCounters - reset a DMA channel
*
* This routine reset the specific DMA chennel.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void tiK3NavssUdmaRstRings
    (
    TI_K3_NAVSS_UDMA_CHAN * pChan
    )
    {
    TI_K3_NAVSS_RINGACC_RING *  pRing0 = NULL;
    TI_K3_NAVSS_RINGACC_RING *  pRing1 = NULL;

    switch (pChan->dir)
        {
        case UDMA_DEV_TO_MEM:
            pRing0 = pChan->pRchan->pFdRing;
            pRing1 = pChan->pRchan->pRRing;
            break;

        case UDMA_MEM_TO_DEV:
            pRing0 = pChan->pTchan->pTRing;
            pRing1 = pChan->pTchan->pTcRing;
            break;

        default:
            DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                       "ERROR! %s() Line:%d\n", __func__, __LINE__);
            return;
        }

    if (pRing0 != NULL)
        {
        (void) tiK3NavssRingaccRingRstDma (pRing0, 0);
        }

    if (pRing1 != NULL)
        {
        (void) tiK3NavssRingaccRingRst (pRing1);
        }
    }

/*******************************************************************************
*
* tiK3NavssUdmaRstCounters - reset all counters of the DMA channel
*
* This routine resets all counters of the specific DMA channel.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void tiK3NavssUdmaRstCounters
    (
    TI_K3_NAVSS_UDMA_CHAN * pChan
    )
    {
    UINT32  regVal;

    if (pChan->pTchan != NULL)
        {
        regVal = REG_READ_4 (pChan->pTchan->tchanRegBase, 
                             TI_K3_NAVSS_UDMA_TCHAN_RT_BCNT_OFS);
        REG_WRITE_4 (pChan->pTchan->tchanRegBase, 
                     TI_K3_NAVSS_UDMA_TCHAN_RT_BCNT_OFS, regVal);

        regVal = REG_READ_4 (pChan->pTchan->tchanRegBase, 
                             TI_K3_NAVSS_UDMA_TCHAN_RT_SBCNT_OFS);
        REG_WRITE_4 (pChan->pTchan->tchanRegBase,
                     TI_K3_NAVSS_UDMA_TCHAN_RT_SBCNT_OFS, regVal);

        regVal = REG_READ_4 (pChan->pTchan->tchanRegBase, 
                             TI_K3_NAVSS_UDMA_TCHAN_RT_PCNT_OFS);
        REG_WRITE_4 (pChan->pTchan->tchanRegBase, 
                     TI_K3_NAVSS_UDMA_TCHAN_RT_PCNT_OFS, regVal);

        regVal = REG_READ_4 (pChan->pTchan->tchanRegBase, 
                             TI_K3_NAVSS_UDMA_TCHAN_RT_PEER_BCNT_OFS);
        REG_WRITE_4 (pChan->pTchan->tchanRegBase, 
                     TI_K3_NAVSS_UDMA_TCHAN_RT_PEER_BCNT_OFS, regVal);
        }

    if (pChan->pRchan != NULL)
        {
        regVal = REG_READ_4 (pChan->pRchan->rchanRegBase, 
                             TI_K3_NAVSS_UDMA_RCHAN_RT_BCNT_OFS);
        REG_WRITE_4 (pChan->pRchan->rchanRegBase, 
                     TI_K3_NAVSS_UDMA_RCHAN_RT_BCNT_OFS, regVal);

        regVal = REG_READ_4 (pChan->pRchan->rchanRegBase, 
                             TI_K3_NAVSS_UDMA_RCHAN_RT_SBCNT_OFS);
        REG_WRITE_4 (pChan->pRchan->rchanRegBase, 
                     TI_K3_NAVSS_UDMA_RCHAN_RT_SBCNT_OFS, regVal);

        regVal = REG_READ_4 (pChan->pRchan->rchanRegBase, 
                             TI_K3_NAVSS_UDMA_RCHAN_RT_PCNT_OFS);
        REG_WRITE_4 (pChan->pRchan->rchanRegBase, 
                     TI_K3_NAVSS_UDMA_RCHAN_RT_PCNT_OFS, regVal);

        regVal = REG_READ_4 (pChan->pRchan->rchanRegBase, 
                             TI_K3_NAVSS_UDMA_RCHAN_RT_PEER_BCNT_OFS);
        REG_WRITE_4 (pChan->pRchan->rchanRegBase, 
                     TI_K3_NAVSS_UDMA_RCHAN_RT_PEER_BCNT_OFS, regVal);
        }
    }

/*******************************************************************************
*
* tiK3NavssUdmaRxchanSciCfg - start the DMA channel
*
* This routine lets the specific DMA channel start working.
*
* RETURNS: OK, or ERROR if start failed.
*
* ERRNO: N/A
*/

LOCAL STATUS tiK3NavssUdmaStart
    (
    TI_K3_NAVSS_UDMA_CHAN * pChan
    )
    {
    if (tiK3NavssUdmaChanIsRun (pChan))
        {
        return OK;
        }

    /* clear the teardown bit */

    (void) tiK3NavssUdmaHardStop (pChan);

    /* reset the rings */

    (void) tiK3NavssUdmaRstRings (pChan);

    /* reset all counters */
    
    (void) tiK3NavssUdmaRstCounters (pChan);

    switch (pChan->dir) 
        {
        case UDMA_DEV_TO_MEM:
            REG_WRITE_4 (pChan->pRchan->rchanRegBase, 
                         TI_K3_NAVSS_UDMA_RCHAN_RT_CTL_OFS,
                         TI_K3_NAVSS_UDMA_CHAN_RT_CTL_EN);

            REG_WRITE_4 (pChan->pRchan->rchanRegBase, 
                         TI_K3_NAVSS_UDMA_RCHAN_RT_PEER_RT_EN_OFS,
                         TI_K3_NAVSS_UDMA_PEER_RT_EN_ENABLE);
            break;

        case UDMA_MEM_TO_DEV:
            REG_WRITE_4 (pChan->pTchan->tchanRegBase, 
                         TI_K3_NAVSS_UDMA_TCHAN_RT_PEER_RT_EN_OFS,
                         TI_K3_NAVSS_UDMA_PEER_RT_EN_ENABLE);

            REG_WRITE_4 (pChan->pTchan->tchanRegBase, 
                         TI_K3_NAVSS_UDMA_TCHAN_RT_CTL_OFS,
                         TI_K3_NAVSS_UDMA_CHAN_RT_CTL_EN);
            break;

        default:
            DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                       "ERROR! %s() Line:%d\n", __func__, __LINE__);
            return ERROR;
        }

    return OK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaRxchanSciCfg - set the configuration of DMA RX channel to the HW
*
* This routine configures the specific DMA RX channel on the HW.
*
* RETURNS: OK, or ERROR if configure failed.
*
* ERRNO: N/A
*/

LOCAL STATUS tiK3NavssUdmaRxchanSciCfg
    (
    TI_K3_NAVSS_UDMA_CHAN * pRxChan
    )
    {
    UINT32  fdRingId;
    UINT32  rRingId;
    struct  ti_sci_msg_rm_udmap_rx_ch_cfg req = {0};
    struct  ti_sci_msg_rm_udmap_flow_cfg flow_req = {0};
    UINT32  mode;

    if (pRxChan == NULL)
        {
        return ERROR;
        }

    fdRingId = pRxChan->pRchan->pFdRing->id;
    rRingId = pRxChan->pRchan->pRRing->id;

    if (pRxChan->pktMode == UDMA_PKT_MODE)
        {
        mode = TISCI_RM_UDMAP_CHAN_TYPE_PKT_PBRR;
        }
    else
        {
        mode = TISCI_RM_UDMAP_CHAN_TYPE_3RDP_PBRR;
        }

    req.valid_params = TISCI_MSG_VALUE_RM_UDMAP_CH_FETCH_SIZE_VALID 
                       | TISCI_MSG_VALUE_RM_UDMAP_CH_CQ_QNUM_VALID 
                       | TISCI_MSG_VALUE_RM_UDMAP_CH_CHAN_TYPE_VALID;
    req.nav_id = (UINT16) pRxChan->pUdmaDrv->tisciDevId;
    req.index = (UINT16) pRxChan->pRchan->id;
    req.rx_fetch_size = 16;
    req.rxcq_qnum = (UINT16) rRingId;
    if (pRxChan->pRflow->id != pRxChan->pRchan->id)
        {
        req.flowid_start = (UINT16) pRxChan->pRflow->id;
        req.flowid_cnt = 1;
        req.valid_params |= TISCI_MSG_VALUE_RM_UDMAP_CH_RX_FLOWID_START_VALID
                            | TISCI_MSG_VALUE_RM_UDMAP_CH_RX_FLOWID_CNT_VALID;
        }
    req.rx_chan_type = (UINT8) mode;

    (void) tisci_rm_udmap_rx_ch_cfg (&req);

    flow_req.valid_params = 
            TISCI_MSG_VALUE_RM_UDMAP_FLOW_EINFO_PRESENT_VALID 
            | TISCI_MSG_VALUE_RM_UDMAP_FLOW_PSINFO_PRESENT_VALID 
            | TISCI_MSG_VALUE_RM_UDMAP_FLOW_ERROR_HANDLING_VALID 
            | TISCI_MSG_VALUE_RM_UDMAP_FLOW_DESC_TYPE_VALID 
            | TISCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_QNUM_VALID 
            | TISCI_MSG_VALUE_RM_UDMAP_FLOW_SRC_TAG_HI_SEL_VALID 
            | TISCI_MSG_VALUE_RM_UDMAP_FLOW_SRC_TAG_LO_SEL_VALID 
            | TISCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_TAG_HI_SEL_VALID 
            | TISCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_TAG_LO_SEL_VALID 
            | TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ0_SZ0_QNUM_VALID 
            | TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ1_QNUM_VALID 
            | TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ2_QNUM_VALID 
            | TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ3_QNUM_VALID 
            | TISCI_MSG_VALUE_RM_UDMAP_FLOW_PS_LOCATION_VALID;

    flow_req.nav_id = (UINT16) pRxChan->pUdmaDrv->tisciDevId;
    flow_req.flow_index = (UINT16) pRxChan->pRflow->id;

    if (pRxChan->needEpib)
        {
        flow_req.rx_einfo_present = 1;
        }
    else
        {
        flow_req.rx_einfo_present = 0;
        }

    if (pRxChan->psdSize != 0)
        {
        flow_req.rx_psinfo_present = 1;
        }
    else
        {
        flow_req.rx_psinfo_present = 0;
        }

    flow_req.rx_error_handling = 0;
    flow_req.rx_desc_type = 0;
    flow_req.rx_dest_qnum = (UINT16) rRingId;
    flow_req.rx_src_tag_hi_sel = 2;
    flow_req.rx_src_tag_lo_sel = 4;
    flow_req.rx_dest_tag_hi_sel = 5;
    flow_req.rx_dest_tag_lo_sel = 4;
    flow_req.rx_fdq0_sz0_qnum = (UINT16) fdRingId;
    flow_req.rx_fdq1_qnum = (UINT16) fdRingId;
    flow_req.rx_fdq2_qnum = (UINT16) fdRingId;
    flow_req.rx_fdq3_qnum = (UINT16) fdRingId;
    flow_req.rx_ps_location = 0;

    (void) tisci_rm_udmap_rx_flow_cfg (&flow_req);

    return OK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaTxChanSciCfg - set the configuration of DMA TX channel to the HW
*
* This routine configures the specific DMA TX channel on the HW.
*
* RETURNS: OK, or ERROR if configure failed.
*
* ERRNO: N/A
*/

LOCAL STATUS tiK3NavssUdmaTxChanSciCfg
    (
    TI_K3_NAVSS_UDMA_CHAN * pTxChan
    )
    {
    struct ti_sci_msg_rm_udmap_tx_ch_cfg req = {0};
    UINT32 mode;

    if (pTxChan == NULL)
        {
        return ERROR;
        }

    if (pTxChan->pktMode == UDMA_PKT_MODE)
        {
        mode = TISCI_RM_UDMAP_CHAN_TYPE_PKT_PBRR;
        }
    else
        {
        mode = TISCI_RM_UDMAP_CHAN_TYPE_3RDP_PBRR;
        }

    req.valid_params = TISCI_MSG_VALUE_RM_UDMAP_CH_CHAN_TYPE_VALID 
                       | TISCI_MSG_VALUE_RM_UDMAP_CH_FETCH_SIZE_VALID
                       | TISCI_MSG_VALUE_RM_UDMAP_CH_CQ_QNUM_VALID;
    req.nav_id = (UINT16) pTxChan->pUdmaDrv->tisciDevId;
    req.index = (UINT16) pTxChan->pTchan->id;
    req.tx_chan_type = (UINT8) mode;
    req.tx_fetch_size = 16;
    req.txcq_qnum = (UINT16) pTxChan->pTchan->pTcRing->id;

    (void) tisci_rm_udmap_tx_ch_cfg (&req);

    return OK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaRxChanCfg - configure a DMA channel to RX channel
*
* This routine configures the specific DMA channel to receive packet.
*
* RETURNS: OK, or ERROR if configure failed.
*
* ERRNO: N/A
*/

LOCAL STATUS tiK3NavssUdmaRxChanCfg
    (
    TI_K3_NAVSS_UDMA_CHAN * pRxChan
    )
    {
    TI_K3_NAVSS_RINGACC_RING_SET    ringCfg;
    UINT32                          fdRingId;

    pRxChan->dir = UDMA_DEV_TO_MEM;
    pRxChan->pktMode = UDMA_PKT_MODE;
    pRxChan->psdSize = 16;
    pRxChan->needEpib = TRUE;

    /* alloc one HW channel, using RX channel 0 */

    pRxChan->pRchan = vxbMemAlloc (sizeof(TI_K3_NAVSS_UDMA_RCHAN));
    if (pRxChan->pRchan == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }
    pRxChan->pRchan->id = 0;
    pRxChan->pRchan->rchanRegBase = pRxChan->pUdmaDrv->rchanrtRegBase 
                                    + pRxChan->pRchan->id * 0x1000;

    if (tiK3NavssUdmaChanIsRun (pRxChan))
        {
        (void) tiK3NavssUdmaStop (pRxChan);
        }

    /* alloc Rflow channel, using the same ID of RX channel */

    pRxChan->pRflow = vxbMemAlloc (sizeof(TI_K3_NAVSS_UDMA_RFLOW));
    if (pRxChan->pRflow == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pRxChan->pRchan);
        return ERROR;
        }
    pRxChan->pRflow->id = pRxChan->pRchan->id;

    /* request ringacc rings for RX channel */

    fdRingId = pRxChan->pUdmaDrv->tchanCnt + pRxChan->pUdmaDrv->echanCnt
               + pRxChan->pRchan->id;

    pRxChan->pRchan->pFdRing = tiK3NavssRingaccRequestRing (fdRingId);
    if (pRxChan->pRchan->pFdRing == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pRxChan->pRchan);
        (void) vxbMemFree (pRxChan->pRflow);
        return ERROR;
        }

    pRxChan->pRchan->pRRing = tiK3NavssRingaccRequestRing (
                                                TI_K3_NAVSS_UDMA_CHAN_RRING_ID);
    if (pRxChan->pRchan->pRRing == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pRxChan->pRchan);
        (void) vxbMemFree (pRxChan->pRflow);
        return ERROR;
        }

    ringCfg.elemNum = TI_K3_NAVSS_UDMA_RX_DESC_NUM;
    ringCfg.elemSize = TI_K3_NAVSS_RINGACC_RING_ELSIZE_8;
    ringCfg.ringMode = TI_K3_NAVSS_RINGACC_RING_MODE_MESSAGE;
    if (tiK3NavssRingaccRingCfg (pRxChan->pRchan->pFdRing, &ringCfg) != OK)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pRxChan->pRchan);
        (void) vxbMemFree (pRxChan->pRflow);
        return ERROR;
        }
    if (tiK3NavssRingaccRingCfg (pRxChan->pRchan->pRRing, &ringCfg) != OK)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pRxChan->pRchan);
        (void) vxbMemFree (pRxChan->pRflow);
        return ERROR;
        }

    pRxChan->srcThread = TI_K3_NAVSS_CPSW_PSIL_BASE;
    pRxChan->dstThread = (pRxChan->pUdmaDrv->psilBase + pRxChan->pRchan->id)
                         | 0x8000;

    /* RX channel tisci cfg */

    (void) tiK3NavssUdmaRxchanSciCfg (pRxChan);

    /* PSI-L pairing */

    (void) tisci_rm_psil_pair (TI_K3_NAVSS_PSIL_TISCI_DEV_ID,
                               pRxChan->srcThread, pRxChan->dstThread);

    /* dma buffer init */

    pRxChan->hdescSize = tiK3NavssUdmaHdescSizeCalc (pRxChan->needEpib,
                                                     pRxChan->psdSize,
                                                     0);
    pRxChan->hdescSize = (UINT32) ROUND_UP (pRxChan->hdescSize,
                                            _CACHE_ALIGN_SIZE);

    pRxChan->pRxDesc = cacheDmaMalloc (pRxChan->hdescSize 
                                       * TI_K3_NAVSS_UDMA_RX_DESC_NUM);
    if (pRxChan->pRxDesc == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pRxChan->pRchan);
        (void) vxbMemFree (pRxChan->pRflow);
        return ERROR;
        }
    pRxChan->rxDescPhyBase = udmaVirToPhy ((VIRT_ADDR) pRxChan->pRxDesc);
    memset (pRxChan->pRxDesc, 0, 
            pRxChan->hdescSize * TI_K3_NAVSS_UDMA_RX_DESC_NUM);

    pRxChan->pRxBuf = cacheDmaMalloc (TI_K3_NAVSS_UDMA_RX_BUF_SIZE
                                      * TI_K3_NAVSS_UDMA_RX_DESC_NUM);
    if (pRxChan->pRxBuf == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pRxChan->pRchan);
        (void) vxbMemFree (pRxChan->pRflow);
        (void) cacheDmaFree (pRxChan->pRxDesc);
        return ERROR;
        }
    pRxChan->rxBufPhyBase = udmaVirToPhy ((VIRT_ADDR) pRxChan->pRxBuf);
    memset (pRxChan->pRxBuf, 0, 
            TI_K3_NAVSS_UDMA_RX_BUF_SIZE * TI_K3_NAVSS_UDMA_RX_DESC_NUM);

    pRxChan->isValid = TRUE;

    return OK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaTxChanCfg - configure a DMA channel to TX channel
*
* This routine configures the specific DMA channel to transmit packet.
*
* RETURNS: OK, or ERROR if configure failed.
*
* ERRNO: N/A
*/

LOCAL STATUS tiK3NavssUdmaTxChanCfg
    (
    TI_K3_NAVSS_UDMA_CHAN * pTxChan
    )
    {
    TI_K3_NAVSS_RINGACC_RING_SET    ringCfg;

    pTxChan->dir = UDMA_MEM_TO_DEV;
    pTxChan->pktMode = UDMA_PKT_MODE;
    pTxChan->psdSize = 16;
    pTxChan->needEpib = TRUE;

    /* alloc one HW channel, using TX channel 0 */

    pTxChan->pTchan = vxbMemAlloc (sizeof(TI_K3_NAVSS_UDMA_TCHAN));
    if (pTxChan->pTchan == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    pTxChan->pTchan->id = 0;
    pTxChan->pTchan->tchanRegBase = pTxChan->pUdmaDrv->tchanrtRegBase
                                    + pTxChan->pTchan->id * 0x1000;

    if (tiK3NavssUdmaChanIsRun (pTxChan))
        {
        (void) tiK3NavssUdmaStop (pTxChan);
        }

    /* request ringacc rings for TX channel */

    pTxChan->pTchan->pTRing = tiK3NavssRingaccRequestRing (pTxChan->pTchan->id);
    if (pTxChan->pTchan->pTRing == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pTxChan->pTchan);
        return ERROR;
        }

    pTxChan->pTchan->pTcRing = tiK3NavssRingaccRequestRing (
                                               TI_K3_NAVSS_UDMA_CHAN_TCRING_ID);
    if (pTxChan->pTchan->pTcRing == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pTxChan->pTchan);
        return ERROR;
        }

    ringCfg.elemNum = 16;
    ringCfg.elemSize = TI_K3_NAVSS_RINGACC_RING_ELSIZE_8;
    ringCfg.ringMode = TI_K3_NAVSS_RINGACC_RING_MODE_MESSAGE;
    if (tiK3NavssRingaccRingCfg (pTxChan->pTchan->pTRing, &ringCfg) != OK)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pTxChan->pTchan);
        return ERROR;
        }
    if (tiK3NavssRingaccRingCfg (pTxChan->pTchan->pTcRing, &ringCfg) != OK)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pTxChan->pTchan);
        return ERROR;
        }

    pTxChan->srcThread = pTxChan->pUdmaDrv->psilBase + pTxChan->pTchan->id;
    pTxChan->dstThread = TI_K3_NAVSS_CPSW_PSIL_BASE | 0x8000;

    /* TX channel tisci cfg */

    (void) tiK3NavssUdmaTxChanSciCfg (pTxChan);

    /* PSI-L pairing */

    (void) tisci_rm_psil_pair (TI_K3_NAVSS_PSIL_TISCI_DEV_ID,
                               pTxChan->srcThread, pTxChan->dstThread);

    /* dma buffer init */

    pTxChan->hdescSize = tiK3NavssUdmaHdescSizeCalc (pTxChan->needEpib,
                                                     pTxChan->psdSize,
                                                     0);
    pTxChan->hdescSize = (UINT32) ROUND_UP (pTxChan->hdescSize,
                                            _CACHE_ALIGN_SIZE);

    DEBUG_MSG (K3_NAVSS_UDMA_DBG_INFO,
               "%s(%d) pTxChan->hdescSize:%d\n", 
               __func__, __LINE__, pTxChan->hdescSize);

    /* for polling mode, just alloc one desc for TX */

    pTxChan->pTxDesc = cacheDmaMalloc (pTxChan->hdescSize);
    if (pTxChan->pTxDesc == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pTxChan->pTchan);
        return ERROR;
        }
    pTxChan->txDescPhyBase = udmaVirToPhy ((VIRT_ADDR) pTxChan->pTxDesc);
    memset (pTxChan->pTxDesc, 0, pTxChan->hdescSize);

    pTxChan->txBusy = FALSE;
    pTxChan->isValid = TRUE;

    return OK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaEnable - enable DMA channel
*
* This routine enables the specific DMA channel.
*
* RETURNS: OK, or ERROR if enable failed.
*
* ERRNO: N/A
*/

LOCAL STATUS tiK3NavssUdmaEnable
    (
    TI_K3_NAVSS_UDMA_CHAN *         pChan
    )
    {
    TI_K3_NAVSS_UDMA_HOST_DESC *    pRxDesc;
    PHYS_ADDR                       rxDescPhyAddr;
    PHYS_ADDR                       dmaPhyAddr; /* 32-bit DDR address support */
    UINT32                          i;

    if (tiK3NavssUdmaStart (pChan) != OK)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    /* for RX channel */

    if (pChan->dir == UDMA_DEV_TO_MEM)
        {
        for (i = 0; i < TI_K3_NAVSS_UDMA_RX_DESC_NUM; i++)
            {
            pRxDesc = (TI_K3_NAVSS_UDMA_HOST_DESC *) ((UINT8 *) pChan->pRxDesc
                                                      + (pChan->hdescSize * i));
            dmaPhyAddr = udmaVirToPhy ((VIRT_ADDR) pChan->pRxBuf
                                       + (TI_K3_NAVSS_UDMA_RX_BUF_SIZE * i));

            (void) tiK3NavssUdmaHdescRst (pRxDesc);

            (void) tiK3NavssUdmaHdescInit (
                                     pRxDesc,
                                     (pChan->needEpib ?
                                      TI_K3_NAVSS_UDMA_HDESC_INFO0_EPIB_PRESENT
                                      : 0),
                                     pChan->psdSize);

            (void) tiK3NavssUdmaHdescPktlenSet (pRxDesc,
                                                TI_K3_NAVSS_UDMA_RX_BUF_SIZE);

            (void) tiK3NavssUdmaHdescBufSet (pRxDesc,
                                             dmaPhyAddr,
                                             TI_K3_NAVSS_UDMA_RX_BUF_SIZE,
                                             dmaPhyAddr,
                                             TI_K3_NAVSS_UDMA_RX_BUF_SIZE);

            rxDescPhyAddr = udmaVirToPhy ((VIRT_ADDR) pRxDesc);
            (void) tiK3NavssRingaccRingPush (pChan->pRchan->pFdRing, 
                                             &rxDescPhyAddr);
            }
        }

    return OK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaSendTry - check DMA TX channel state
*
* This routine checks the busy state of the DMA TX channel. This routine
* must be called before use tiK3NavssUdmaSend() to send packet.
*
* RETURNS: OK, or ERROR if the DMA TX channel is busy.
*
* ERRNO: N/A
*/

STATUS tiK3NavssUdmaSendTry (void)
    {
    TI_K3_NAVSS_UDMA_CHAN *         pTxChan;
    PHYS_ADDR                       phyAddr;

    pTxChan = &tiK3NavssUdmaDrv.pChans[TI_K3_NAVSS_UDMA_TX_CHAN_ID];

    if (!pTxChan->txBusy)
        {
        return OK;
        }
    else
        {
        if (tiK3NavssRingaccRingGetOcc (pTxChan->pTchan->pTcRing) != 0)
            {
            if (tiK3NavssRingaccRingPop (pTxChan->pTchan->pTcRing, &phyAddr)
                == OK)
                {
                pTxChan->txBusy = FALSE;
                return OK;
                }
            }

        return ERROR;
        }
    }

/*******************************************************************************
*
* tiK3NavssUdmaSend - send a packet to TX channel
*
* This routine sends a packet to the DMA TX channel. Before call this routine
* to send packet, must call tiK3NavssUdmaSendTry() to check the TX channel 
* state. This routine can only be called when tiK3NavssUdmaSendTry() return
* OK.
*
* RETURNS: OK, or ERROR if send failed.
*
* ERRNO: N/A
*/

STATUS tiK3NavssUdmaSend
    (
    void *  pBuf,
    UINT32  len,
    void *  pData
    )
    {
    TI_K3_NAVSS_UDMA_PKT_DATA       pktData;
    TI_K3_NAVSS_UDMA_CHAN *         pTxChan;
    PHYS_ADDR                       bufPhyAddr; /* 32-bit DDR address support */
    TI_K3_NAVSS_UDMA_HOST_DESC *    pTxDesc;
    PHYS_ADDR                       txDescPhyAddr;

    if (pBuf == NULL)
        {
        return ERROR;
        }

    pTxChan = &tiK3NavssUdmaDrv.pChans[TI_K3_NAVSS_UDMA_TX_CHAN_ID];

    if (pData != NULL)
        {
        memcpy (&pktData, pData, sizeof(TI_K3_NAVSS_UDMA_PKT_DATA));
        }
    else
        {
        memset (&pktData, 0, sizeof(TI_K3_NAVSS_UDMA_PKT_DATA));
        }

    pTxDesc = pTxChan->pTxDesc;

    (void) tiK3NavssUdmaHdescRst (pTxDesc);

    (void) tiK3NavssUdmaHdescInit (pTxDesc,
                                   (pTxChan->needEpib ? 
                                    TI_K3_NAVSS_UDMA_HDESC_INFO0_EPIB_PRESENT
                                    : 0),
                                   pTxChan->psdSize);

    (void) tiK3NavssUdmaHdescPktlenSet (pTxDesc, len);

    (void) cacheFlush (DATA_CACHE, pBuf, len);
    bufPhyAddr = udmaVirToPhy ((VIRT_ADDR) pBuf); 
    (void) tiK3NavssUdmaHdescBufSet (pTxDesc, bufPhyAddr, len, bufPhyAddr, len);

    (void) tiK3NavssUdmaHdescPktIdSet (&pTxDesc->hdr, pTxChan->id, 0x3FFF);

    (void) tiK3NavssUdmaHdescRetPolicySet (&pTxDesc->hdr, 0, 
                                           pTxChan->pTchan->pTcRing->id);

    (void) tiK3NavssUdmaHdescPktTypeSet (pTxDesc, pktData.pktType);
    (void) tiK3NavssUdmaHdescTagIdSet (&pTxDesc->hdr, 0, pktData.dstTag);

    txDescPhyAddr = udmaVirToPhy ((VIRT_ADDR) pTxChan->pTxDesc);
    (void) tiK3NavssRingaccRingPush (pTxChan->pTchan->pTRing, &txDescPhyAddr);

    pTxChan->txBusy = TRUE;

    return OK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaRecieve - receive a packet from RX channel
*
* This routine receives a packet from the DMA RX channel. If no packet, the 
* return packet length is 0.
*
* RETURNS: The received packet length, or 0 if receive no packet.
*
* ERRNO: N/A
*/

INT32 tiK3NavssUdmaRecieve
    (
    void *  pBuf,
    void *  pData
    )
    {
    TI_K3_NAVSS_UDMA_CHAN *         pRxChan;
    TI_K3_NAVSS_UDMA_HOST_DESC *    pRxDesc;
    PHYS_ADDR                       descPhyAddr;
    VIRT_ADDR                       dmaBufVirAddr;
    PHYS_ADDR                       dmaBufPhyAddr;
    UINT32                          dmaBufLen;
    UINT32                          pktLen;

    pRxChan = &tiK3NavssUdmaDrv.pChans[TI_K3_NAVSS_UDMA_RX_CHAN_ID];

    if (tiK3NavssRingaccRingPop (pRxChan->pRchan->pRRing, &descPhyAddr) != OK)
        {
        return 0;
        }

    pRxDesc = (TI_K3_NAVSS_UDMA_HOST_DESC *) 
              udmaPhyToVir (pRxChan,
                            UDMA_PHY_ADDR_RX_DESC_TYPE,
                            descPhyAddr);

    (void) tiK3NavssUdmaHdescObufGet (pRxDesc, &dmaBufPhyAddr, &dmaBufLen);

    pktLen = tiK3NavssUdmaHdescPktlenGet (pRxDesc);

    dmaBufVirAddr = udmaPhyToVir (pRxChan,
                                  UDMA_PHY_ADDR_RX_BUF_TYPE,
                                  dmaBufPhyAddr);

    memcpy(pBuf, (void *) dmaBufVirAddr, pktLen);

    (void) tiK3NavssUdmaHdescRst (pRxDesc);

    (void) tiK3NavssUdmaHdescInit (pRxDesc,
                                   (pRxChan->needEpib ? 
                                    TI_K3_NAVSS_UDMA_HDESC_INFO0_EPIB_PRESENT
                                    : 0),
                                   pRxChan->psdSize);

    (void) tiK3NavssUdmaHdescPktlenSet (pRxDesc, TI_K3_NAVSS_UDMA_RX_BUF_SIZE);

    (void) tiK3NavssUdmaHdescBufSet (pRxDesc,
                                     dmaBufPhyAddr,
                                     TI_K3_NAVSS_UDMA_RX_BUF_SIZE,
                                     dmaBufPhyAddr,
                                     TI_K3_NAVSS_UDMA_RX_BUF_SIZE);

    (void) tiK3NavssRingaccRingPush (pRxChan->pRchan->pFdRing, &descPhyAddr);

    return (INT32) pktLen;
    }

/*******************************************************************************
*
* tiK3NavssUdmaInit - initialize TI K3 MCU NAVSS UDMA driver
*
* This is the TI K3 MCU NAVSS UDMA driver initialization routine. 
* It will create two channels for CPSW MAC TX & RX.
*
* RETURNS: OK, or ERROR if initialization failed.
*
* ERRNO: N/A
*/

STATUS tiK3NavssUdmaInit (void)
    {
    UINT32                      regVal;
    TI_K3_NAVSS_UDMA_DRVCTRL *  pUdmaDrv = &tiK3NavssUdmaDrv;
    UINT32                      i;

    /* based on ringacc to bind ring queues */

    if (tiK3NavssRingaccInit () != OK)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        goto initFail;
        }

    pUdmaDrv->cfgRegBase = (VIRT_ADDR) pmapGlobalMap (
                                                TI_K3_NAVSS_UDMA_CFG_BASE, 
                                                0x100, VXB_REG_MAP_MMU_ATTR);
    if (pUdmaDrv->cfgRegBase == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        goto initFail;
        }

    pUdmaDrv->rchanrtRegBase = (VIRT_ADDR) pmapGlobalMap (
                                                TI_K3_NAVSS_UDMA_RCHANRT_BASE, 
                                                0x40000, VXB_REG_MAP_MMU_ATTR);
    if (pUdmaDrv->rchanrtRegBase == NULL) 
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        goto initFail;
        }

    pUdmaDrv->tchanrtRegBase = (VIRT_ADDR) pmapGlobalMap (
                                                TI_K3_NAVSS_UDMA_TCHANRT_BASE, 
                                                0x40000, VXB_REG_MAP_MMU_ATTR);
    if (pUdmaDrv->rchanrtRegBase == NULL) 
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        goto initFail;
        }

    pUdmaDrv->chanCnt = TI_K3_NAVSS_UDMA_CHAN_NUM;
    pUdmaDrv->psilBase = TI_K3_NAVSS_UDMA_PSIL_BASE;
    pUdmaDrv->tisciDevId = TI_K3_NAVSS_UDMA_TISCI_DEV_ID;

    regVal = REG_READ_4 (pUdmaDrv->cfgRegBase, TI_K3_NAVSS_UDMA_CFG_CAP2_OFS);
    pUdmaDrv->rchanCnt = (regVal >> 18) & 0x1FF;
    pUdmaDrv->echanCnt = (regVal >> 9) & 0x1FF;
    pUdmaDrv->tchanCnt = regVal & 0x1FF;

    regVal = REG_READ_4 (pUdmaDrv->cfgRegBase, TI_K3_NAVSS_UDMA_CFG_CAP3_OFS);
    pUdmaDrv->rflowCnt = regVal & 0x3FFF;

    DEBUG_MSG (K3_NAVSS_UDMA_DBG_INFO, 
               "rchanCnt:%d echanCnt:%d tchanCnt:%d rflowCnt:%d\n",
               pUdmaDrv->rchanCnt, pUdmaDrv->echanCnt, pUdmaDrv->tchanCnt,
               pUdmaDrv->rflowCnt);

    if ((pUdmaDrv->tchanCnt + pUdmaDrv->rchanCnt) != pUdmaDrv->chanCnt)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_INFO, "chanCnt(%d) != "
                   "tchanCnt(%d) + rchanCnt(%d)!\n",
                   pUdmaDrv->chanCnt,
                   pUdmaDrv->tchanCnt,
                   pUdmaDrv->rchanCnt);
        pUdmaDrv->chanCnt = pUdmaDrv->tchanCnt + pUdmaDrv->rchanCnt;
        }

    /* alloc channels resource */

    pUdmaDrv->pChans = vxbMemAlloc (pUdmaDrv->chanCnt 
                                   * sizeof(TI_K3_NAVSS_UDMA_CHAN));
    if (pUdmaDrv->pChans == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        goto initFail;
        }

    for (i = 0; i < pUdmaDrv->chanCnt; i++)
        {
        pUdmaDrv->pChans[i].pUdmaDrv = pUdmaDrv;
        pUdmaDrv->pChans[i].id = i;
        pUdmaDrv->pChans[i].pTchan = NULL;
        pUdmaDrv->pChans[i].pRchan = NULL;
        pUdmaDrv->pChans[i].pRflow = NULL;
        }

    /* here we only use 2 channels, chans[0] for TX, chans[1] for RX */

    if (tiK3NavssUdmaTxChanCfg (&pUdmaDrv->pChans[TI_K3_NAVSS_UDMA_TX_CHAN_ID])
        != OK)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pUdmaDrv->pChans);
        goto initFail;
        }

    if (tiK3NavssUdmaRxChanCfg (&pUdmaDrv->pChans[TI_K3_NAVSS_UDMA_RX_CHAN_ID])
        != OK)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        (void) vxbMemFree (pUdmaDrv->pChans);
        goto initFail;
        }

    (void) tiK3NavssUdmaEnable (&pUdmaDrv->pChans[TI_K3_NAVSS_UDMA_TX_CHAN_ID]);
    (void) tiK3NavssUdmaEnable (&pUdmaDrv->pChans[TI_K3_NAVSS_UDMA_RX_CHAN_ID]);

    return OK;

initFail:
    return ERROR;
    }

