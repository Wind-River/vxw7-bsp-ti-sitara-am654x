/* vxbTiK3NavssRingacc.c - TI AM65x MCU NAVSS RingAcc Module driver */

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
This module implements the required MCU NAVSS RingAcc driver to support AM65x 
MCU Ethernet card.

The MCU domain UDMA driver in vxbTiK3NavssUdma.c relies on this driver. 
The TX&RX channels will bind its own ring queues which get from RingAcc module
and use the ring queues to maintain packet descriptors.

Currently, this driver only supports the messaging mode for queue.

INCLUDE FILES: vxWorks.h stdio.h string.h vmLib.h cacheLib.h pmapLib.h
               vmLibCommon.h vxbAccessArchLib.h vxBus.h 
               tiSciPredefs.h vxbTiK3NavssRingacc.h
*/

/* includes */

#include <vxWorks.h>
#include <stdio.h>
#include <string.h>
#include <vmLib.h>
#include <cacheLib.h>
#include <pmapLib.h>
#include <vmLibCommon.h>
#include <arch/arm/vxbAccessArchLib.h>
#include <hwif/vxBus.h>
#include <tiSciClientApi.h>
#include "vxbTiK3NavssRingacc.h"

/* defines */

/* debug macro */

#undef  DEBUG_MSG
#undef  K3_NAVSS_RINGACC_DBG
#ifdef  K3_NAVSS_RINGACC_DBG

/* turning local symbols into global symbols */

#ifdef  LOCAL
#undef  LOCAL
#define LOCAL
#endif

#include <private/kwriteLibP.h>         /* _func_kprintf */
#define K3_NAVSS_UDMA_RINGACC_DBG_OFF           0x00000000
#define K3_NAVSS_UDMA_RINGACC_DBG_ISR           0x00000001
#define K3_NAVSS_UDMA_RINGACC_DBG_ERR           0x00000002
#define K3_NAVSS_UDMA_RINGACC_DBG_INFO          0x00000004
#define K3_NAVSS_UDMA_RINGACC_DBG_ALL           0xffffffff

LOCAL UINT32 k3NavssUdmaRingaccDbgMask = K3_NAVSS_UDMA_RINGACC_DBG_ALL;

#define DEBUG_MSG(mask, ...)                                                  \
    do                                                                        \
        {                                                                     \
        if ((k3NavssUdmaRingaccDbgMask & (mask)) \
            || ((mask) == K3_NAVSS_UDMA_RINGACC_DBG_ALL))  \
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
#endif  /* K3_NAVSS_RINGACC_DBG */

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

/* typedefs */

typedef enum tiK3RingaccAccessMode
    {
    TI_K3_RINGACC_ACCESS_MODE_POP_HEAD,
    TI_K3_RINGACC_ACCESS_MODE_PUSH_TAIL,
    } TI_K3_RINGACC_ACCESS_MODE;

/* imports */

/* forward declarations */
LOCAL PHYS_ADDR ringaccVirToPhy (VIRT_ADDR virtAdrs);
LOCAL STATUS tiK3NavssRingaccRingIoAccess (TI_K3_NAVSS_RINGACC_RING * pRing,
                                           void * pElem,
                                           TI_K3_RINGACC_ACCESS_MODE accessMode);
LOCAL STATUS tiK3NavssRingaccRingIoPushTail (TI_K3_NAVSS_RINGACC_RING * pRing,
                                             void * pElem);
LOCAL STATUS tiK3NavssRingaccRingIoPopHead (TI_K3_NAVSS_RINGACC_RING * pRing,
                                            void * pElem);
LOCAL UINT32 tiK3NavssRingaccRingGetFree (TI_K3_NAVSS_RINGACC_RING * pRing);

/* locals */

LOCAL TI_K3_NAVSS_RINGACC_DRVCTRL tiK3NavssRingaccDrv = {0};

/*****************************************************************************
*
* ringaccVirToPhy - translate the virtual address to physical address
*
* This routine translates the virtual address to physical address.
*
* RETURNS: The physical address.
*
* ERRNO: N/A
*/

LOCAL PHYS_ADDR ringaccVirToPhy
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

/*****************************************************************************
*
* tiK3NavssRingaccRingIoAccess - ringacc ring queue messaging mode access
*
* This routine provides the messaging access mode on ring queues.
*
* RETURNS: OK, or ERROR if access failed.
*
* ERRNO: N/A
*/

LOCAL STATUS tiK3NavssRingaccRingIoAccess
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing,
    void *                      pElem,
    TI_K3_RINGACC_ACCESS_MODE   accessMode
    )
    {
    void *  ptr;

    switch (accessMode)
        {
        case TI_K3_RINGACC_ACCESS_MODE_POP_HEAD:
            ptr = (void *) (pRing->ringFifoRegBase 
                            + TI_K3_RING_REG_FIFO_HEADDATA_OFS);

            ptr = (UINT8 *) ptr + (TI_K3_RING_FIFO_WINDOW_SIZE 
                                   - (4u << pRing->elemSize));

            memcpy (pElem, ptr, (4u << pRing->elemSize));
            pRing->occ--;
            break;

        case TI_K3_RINGACC_ACCESS_MODE_PUSH_TAIL:
            ptr = (void *) (pRing->ringFifoRegBase
                            + TI_K3_RING_REG_FIFO_TAILDATA_OFS);

            ptr = (UINT8 *) ptr + (TI_K3_RING_FIFO_WINDOW_SIZE
                                   - (4u << pRing->elemSize));

            memcpy (ptr, pElem, (4u << pRing->elemSize));
            pRing->free--;
            break;

        default:
            DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                       "ERROR! %s() Line:%d\n", __func__, __LINE__);
            return ERROR;
        }

    return OK;
    }

/*****************************************************************************
*
* tiK3NavssRingaccRingIoPushTail - push entry to messaging mode queue tail
*
* This routine pushs the entry to the tail of specific messaging mode queue.
*
* RETURNS: OK, or ERROR if push failed.
*
* ERRNO: N/A
*/

LOCAL STATUS tiK3NavssRingaccRingIoPushTail
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing,
    void *                      pElem
    )
    {
    return tiK3NavssRingaccRingIoAccess (pRing, pElem, 
                                         TI_K3_RINGACC_ACCESS_MODE_PUSH_TAIL);
    }

/*****************************************************************************
*
* tiK3NavssRingaccRingIoPopHead - pop entry from messaging mode queue head
*
* This routine pop the head entry from the specific messaging mode queue.
*
* RETURNS: OK, or ERROR if pop failed.
*
* ERRNO: N/A
*/

LOCAL STATUS tiK3NavssRingaccRingIoPopHead
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing,
    void *                      pElem
    )
    {
    return tiK3NavssRingaccRingIoAccess (pRing, pElem,
                                         TI_K3_RINGACC_ACCESS_MODE_POP_HEAD);
    }

/*****************************************************************************
*
* tiK3NavssRingaccRingGetFree - get the number of free entries on the ring
*
* This routine gets the number of free entries in the specific ring queue.
*
* RETURNS: The number of free entry.
*
* ERRNO: N/A
*/

LOCAL UINT32 tiK3NavssRingaccRingGetFree
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing
    )
    {
    if (pRing->free == 0)
        {
        pRing->free = pRing->elemNum -
                      REG_READ_4 (pRing->ringRtRegBase,
                                  TI_K3_RING_REG_RT_OCC_OFS);
        }

    return pRing->free;
    }

/*****************************************************************************
*
* tiK3NavssRingaccRequestRing - request one ring
*
* This routine requests a ring control block by the specified ring ID.
*
* RETURNS: The address of requested ring control block.
*
* ERRNO: N/A
*/

TI_K3_NAVSS_RINGACC_RING * tiK3NavssRingaccRequestRing
    (
    UINT32  ringId
    )
    {
    TI_K3_NAVSS_RINGACC_DRVCTRL *   pRingaccDrv = &tiK3NavssRingaccDrv;

    return &pRingaccDrv->pRings[ringId];
    }

/*****************************************************************************
*
* tiK3NavssRingaccRingCfg - configure one ring
*
* This routine initializes the resources of the specific ring and sets them
* to hardware.
*
* RETURNS: OK, or ERROR if configure failed.
*
* ERRNO: N/A
*/

STATUS tiK3NavssRingaccRingCfg
    (
    TI_K3_NAVSS_RINGACC_RING *      pRing,
    TI_K3_NAVSS_RINGACC_RING_SET *  pCfg
    )
    {

    if (pRing == NULL || pCfg == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    pRing->elemNum = pCfg->elemNum;
    pRing->elemSize = pCfg->elemSize;
    pRing->mode = pCfg->ringMode;
    pRing->occ = 0;
    pRing->free = 0;

    /* here we only support messaging mode */

    if (pRing->mode != TI_K3_NAVSS_RINGACC_RING_MODE_MESSAGE) 
        {
        DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    pRing->ringBufVirtAddr = (VIRT_ADDR) cacheDmaMalloc (
                                                    pRing->elemNum * 
                                                    (4u << pRing->elemSize));
    if (pRing->ringBufVirtAddr == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }
    memset ((void *) pRing->ringBufVirtAddr, 
            0, pRing->elemNum * (4u << pRing->elemSize));
    pRing->ringBufDmaPhyAddr = ringaccVirToPhy (pRing->ringBufVirtAddr);

    /* tisci ring config */

    (void) tisci_rm_ring_cfg (TISCI_MSG_VALUE_RM_ALL_NO_ORDER,
                              (UINT16) pRing->pRingaccDrv->tisciDevId,
                              (UINT16) pRing->id,
                              (UINT32) pRing->ringBufDmaPhyAddr,
                              0,          /* 32-bit DDR address support */
                              pRing->elemNum,
                              (UINT8) pRing->mode,
                              (UINT8) pRing->elemSize,
                              0);

    pRing->isUse = TRUE;

    return OK;
    }

/*****************************************************************************
*
* tiK3NavssRingaccRingPush - push entry to the ring queue
*
* This routine pushs one entry content to the specific ring queue. The new entry
* will be pushed at the queue tail.
*
* RETURNS: OK, or ERROR if push failed.
*
* ERRNO: N/A
*/

STATUS tiK3NavssRingaccRingPush
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing,
    void *                      pElem
    )
    {
    if (pRing == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    if (pRing->isUse != TRUE)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    if (tiK3NavssRingaccRingGetFree (pRing) == 0)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    return tiK3NavssRingaccRingIoPushTail (pRing, pElem);
    }

/*****************************************************************************
*
* tiK3NavssRingaccRingGetOcc - get the number of valid entires on the ring
*
* This routine gets the number of valid entires on the ring queue.
*
* RETURNS: The number of valid entries.
*
* ERRNO: N/A
*/

UINT32 tiK3NavssRingaccRingGetOcc
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing
    )
    {
    return REG_READ_4 (pRing->ringRtRegBase, TI_K3_RING_REG_RT_OCC_OFS);
    }

/*****************************************************************************
*
* tiK3NavssRingaccRingPop - pop entry from the ring queue
*
* This routine pops one entry content from the specific ring queue. The entry
* is from the queue head.
*
* RETURNS: OK, or ERROR if pop failed.
*
* ERRNO: N/A
*/

STATUS tiK3NavssRingaccRingPop
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing,
    void *                      pElem
    )
    {
    if (pRing == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    if (pRing->isUse != TRUE)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    if (pRing->occ == 0)
        {
        pRing->occ = tiK3NavssRingaccRingGetOcc (pRing);
        }

    if (pRing->occ == 0)
        {
        return ERROR;
        }

    return tiK3NavssRingaccRingIoPopHead (pRing, pElem);
    }

/*****************************************************************************
*
* tiK3NavssRingaccRingRst - reset the ring queue
*
* This routine resets the specific ring queue.
*
* RETURNS: OK, or ERROR if reset failed.
*
* ERRNO: N/A
*/

STATUS tiK3NavssRingaccRingRst
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing
    )
    {
    if (pRing == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    if (pRing->isUse != TRUE)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    pRing->occ = 0;
    pRing->free = 0;

    (void) tisci_rm_ring_cfg (TISCI_MSG_VALUE_RM_RING_COUNT_VALID,
                              (UINT16) pRing->pRingaccDrv->tisciDevId,
                              (UINT16) pRing->id,
                              0,
                              0,
                              pRing->elemNum,
                              0,
                              0,
                              0);

    return OK;
    }

/*****************************************************************************
*
* tiK3NavssRingaccRingRstDma - reset the ring queue
*
* This routine resets the specific ring queue.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

void tiK3NavssRingaccRingRstDma
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing,
    UINT32                      occ
    )
    {
    UINT32  dbCntMax;
    UINT32  dbCnt;
    UINT32  occVal;

    if (pRing == NULL)
    {
    DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
               "ERROR! %s() Line:%d\n", __func__, __LINE__);
    return;
    }

    if (pRing->isUse != TRUE)
    {
    DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
               "ERROR! %s() Line:%d\n", __func__, __LINE__);
    return;
    }

    occVal = occ;
    if (occVal == 0)
        {
        occVal = REG_READ_4 (pRing->ringRtRegBase, TI_K3_RING_REG_RT_OCC_OFS);
        }

    if (occVal != 0)
        {
        (void) tiK3NavssRingaccRingRst (pRing);

        /* set the ring to ring mode */

        if (pRing->mode != TI_K3_NAVSS_RINGACC_RING_MODE_RING)
            {
            tisci_rm_ring_cfg (TISCI_MSG_VALUE_RM_RING_MODE_VALID,
                               (UINT16) pRing->pRingaccDrv->tisciDevId,
                               (UINT16) pRing->id,
                               0,
                               0,
                               0,
                               TI_K3_NAVSS_RINGACC_RING_MODE_RING,
                               0,
                               0);
            }

        /*
         * ring the (doorbell 2^22 – OccNum) times
         * this will set the internal UDMAP ring state occupancy counter to 0
         */

        dbCntMax = (1u << 22) - occVal;
        while (dbCntMax != 0)
            {
            if (dbCntMax > TI_K3_NAVSS_RINGACC_MAX_DB_RING_CNT)
                {
                dbCnt = TI_K3_NAVSS_RINGACC_MAX_DB_RING_CNT;
                }
                else
                {
                dbCnt = dbCntMax;
                }

            REG_WRITE_4 (pRing->ringRtRegBase, TI_K3_RING_REG_RT_DB_OFS,
                         dbCnt);

            dbCntMax -= dbCnt;
            }

        /* set the original ring mode */

        if (pRing->mode != TI_K3_NAVSS_RINGACC_RING_MODE_RING)
            {
            (void) tisci_rm_ring_cfg (TISCI_MSG_VALUE_RM_RING_MODE_VALID,
                                      (UINT16) pRing->pRingaccDrv->tisciDevId,
                                      (UINT16) pRing->id,
                                      0,
                                      0,
                                      0,
                                      (UINT8) pRing->mode,
                                      0,
                                      0);
            }
        }

    (void) tiK3NavssRingaccRingRst (pRing);
    }

/*******************************************************************************
*
* tiK3NavssRingaccInit - initialize TI K3 NAVSS Ringacc driver
*
* This is the TI K3 NAVSS UDMA driver initialization routine. 
*
* RETURNS: OK, or ERROR if initialization failed.
* 
* ERRNO: N/A
*/

STATUS tiK3NavssRingaccInit (void)
    {
    TI_K3_NAVSS_RINGACC_DRVCTRL *   pRingaccDrv = &tiK3NavssRingaccDrv;
    UINT32                          i;

    pRingaccDrv->rtRegBase = (VIRT_ADDR) pmapGlobalMap (TI_K3_RING_REG_RT_BASE, 
                                                        0x400000, 
                                                        VXB_REG_MAP_MMU_ATTR);
    if (pRingaccDrv->rtRegBase == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    pRingaccDrv->fifoRegBase = (VIRT_ADDR) pmapGlobalMap (
                                                    TI_K3_RING_REG_FIFO_BASE, 
                                                    0x400000, 
                                                    VXB_REG_MAP_MMU_ATTR);
    if (pRingaccDrv->fifoRegBase == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    pRingaccDrv->ringNum = TI_K3_NAVSS_RINGACC_RING_NUM;
    pRingaccDrv->tisciDevId = TI_K3_NAVSS_RINGACC_TISCI_DEV_ID;

    pRingaccDrv->pRings = vxbMemAlloc (pRingaccDrv->ringNum 
                                       * sizeof(TI_K3_NAVSS_RINGACC_RING));
    if (pRingaccDrv->pRings == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_RINGACC_DBG_ERR,
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }
    
    for (i = 0; i < pRingaccDrv->ringNum; i++)
        {
        pRingaccDrv->pRings[i].pRingaccDrv = pRingaccDrv;
        pRingaccDrv->pRings[i].id = i;
        pRingaccDrv->pRings[i].ringRtRegBase = pRingaccDrv->rtRegBase
                                               + 0x1000 * i;
        pRingaccDrv->pRings[i].ringFifoRegBase = pRingaccDrv->fifoRegBase
                                                 + 0x1000 * i;
        }

    return OK;
    }

