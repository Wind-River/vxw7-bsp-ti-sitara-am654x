/* vxbTiK3NavssRingacc.h - TI AM65x MCU NAVSS RingAcc driver header */

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

#ifndef __INCvxbTiK3NavssRingacch
#define __INCvxbTiK3NavssRingacch

#ifdef __cplusplus
extern "C"
{
#endif

#define TI_K3_NAVSS_RINGACC_RING_NUM            (286u)
#define TI_K3_NAVSS_RINGACC_TISCI_DEV_ID        (195u)

#define TI_K3_RING_REG_RT_BASE                  (0X2B800000u)
#define TI_K3_RING_REG_RT_DB_OFS                (0x10u)
#define TI_K3_RING_REG_RT_OCC_OFS               (0x18u)
#define TI_K3_RING_REG_RT_INDX_OFS              (0x1Cu)
#define TI_K3_RING_REG_RT_HWOCC_OFS             (0x20u)
#define TI_K3_RING_REG_RT_HWINDX_OFS            (0x24u)

#define TI_K3_RING_REG_FIFO_BASE                (0X2B000000u)
#define TI_K3_RING_REG_FIFO_HEADDATA_OFS        (0x0u)
#define TI_K3_RING_REG_FIFO_TAILDATA_OFS        (0x200u)
#define TI_K3_RING_REG_FIFO_PEEKHEADDATA_OFS    (0x400u)
#define TI_K3_RING_REG_FIFO_PEEKTAILDATA_OFS    (0x600u)

#define TI_K3_RING_FIFO_WINDOW_SIZE             (512u)
#define TI_K3_NAVSS_RINGACC_MAX_DB_RING_CNT     (127u)


typedef enum tiK3NavssRingaccRingElemSize
    {
    TI_K3_NAVSS_RINGACC_RING_ELSIZE_4 = 0,
    TI_K3_NAVSS_RINGACC_RING_ELSIZE_8,
    TI_K3_NAVSS_RINGACC_RING_ELSIZE_16,
    TI_K3_NAVSS_RINGACC_RING_ELSIZE_32,
    TI_K3_NAVSS_RINGACC_RING_ELSIZE_64,
    TI_K3_NAVSS_RINGACC_RING_ELSIZE_128,
    TI_K3_NAVSS_RINGACC_RING_ELSIZE_256,
    TI_K3_NAVSS_RINGACC_RING_ELSIZE_INVALID
    } TI_K3_NAVSS_RINGACC_RING_ELEM_SIZE;

typedef enum tiK3NavssRingaccRingMode 
    {
    TI_K3_NAVSS_RINGACC_RING_MODE_RING = 0,
    TI_K3_NAVSS_RINGACC_RING_MODE_MESSAGE,
    TI_K3_NAVSS_RINGACC_RING_MODE_CREDENTIALS,
    TI_K3_NAVSS_RINGACC_RING_MODE_QM,
    TI_K3_NAVSS_RINGACC_RING_MODE_INVALID
    } TI_K3_NAVSS_RINGACC_RING_MODE;

typedef struct tiK3NavssRingaccRingSet
    {
    UINT32                              elemNum;
    TI_K3_NAVSS_RINGACC_RING_ELEM_SIZE  elemSize;
    TI_K3_NAVSS_RINGACC_RING_MODE       ringMode;
    } TI_K3_NAVSS_RINGACC_RING_SET;

typedef struct tiK3NavssRingaccRing
    {
    struct tiK3NavssRingaccDrvctrl *    pRingaccDrv;
    UINT32                              id;
    VIRT_ADDR                           ringRtRegBase;
    VIRT_ADDR                           ringFifoRegBase;
    PHYS_ADDR                           ringBufDmaPhyAddr;
    VIRT_ADDR                           ringBufVirtAddr;
    UINT32                              elemNum;
    TI_K3_NAVSS_RINGACC_RING_ELEM_SIZE  elemSize;
    TI_K3_NAVSS_RINGACC_RING_MODE       mode;

    UINT32                              free;
    UINT32                              occ;

    BOOL                                isUse;
    } TI_K3_NAVSS_RINGACC_RING;

typedef struct tiK3NavssRingaccDrvctrl
    {
    UINT32                      ringNum;
    UINT32                      tisciDevId;
    VIRT_ADDR                   rtRegBase;
    VIRT_ADDR                   fifoRegBase;
    
    TI_K3_NAVSS_RINGACC_RING *  pRings;
    } TI_K3_NAVSS_RINGACC_DRVCTRL;

TI_K3_NAVSS_RINGACC_RING * tiK3NavssRingaccRequestRing
    (
    UINT32  ringId
    );

STATUS tiK3NavssRingaccRingCfg
    (
    TI_K3_NAVSS_RINGACC_RING *      pRing,
    TI_K3_NAVSS_RINGACC_RING_SET *  pCfg
    );

STATUS tiK3NavssRingaccRingPush
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing,
    void *                      pElem
    );

UINT32 tiK3NavssRingaccRingGetOcc
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing
    );

STATUS tiK3NavssRingaccRingPop
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing,
    void *                      pElem
    );

STATUS tiK3NavssRingaccRingRst
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing
    );

void tiK3NavssRingaccRingRstDma
    (
    TI_K3_NAVSS_RINGACC_RING *  pRing,
    UINT32                      occ
    );

STATUS tiK3NavssRingaccInit (void);

#ifdef __cplusplus
}
#endif

#endif /* __INCvxbTiK3NavssRingacch */

