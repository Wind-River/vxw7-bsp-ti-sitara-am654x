/* vxbTiK3NavssUdma.h - TI AM65x MCU NAVSS UDMA driver header */

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

#ifndef __INCvxbTiK3NavssUdmah
#define __INCvxbTiK3NavssUdmah

#include "vxbTiK3NavssUdmaDesc.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* defines */

/* register address */

#ifndef BIT
#define BIT(n)                              (1u << (n))
#endif

#define TI_K3_NAVSS_UDMA_CFG_BASE           (0x285C0000u)
#define TI_K3_NAVSS_UDMA_RCHANRT_BASE       (0x2A800000u)
#define TI_K3_NAVSS_UDMA_TCHANRT_BASE       (0x2AA00000u)

#define TI_K3_NAVSS_UDMA_CFG_CAP2_OFS       (0x28u)
#define TI_K3_NAVSS_UDMA_CFG_CAP3_OFS       (0x2Cu)

#define TI_K3_NAVSS_UDMA_RCHAN_RT_CTL_OFS   (0x0u)

#define TI_K3_NAVSS_UDMA_TCHAN_RT_CTL_OFS   (0x0u)

#define TI_K3_NAVSS_UDMA_CHAN_RT_CTL_EN     BIT(31)
#define TI_K3_NAVSS_UDMA_CHAN_RT_CTL_TDOWN  BIT(30)

#define TI_K3_NAVSS_UDMA_RCHAN_RT_PCNT_OFS  (0x400u)
#define TI_K3_NAVSS_UDMA_RCHAN_RT_BCNT_OFS  (0x408u)
#define TI_K3_NAVSS_UDMA_RCHAN_RT_SBCNT_OFS (0x410u)

#define TI_K3_NAVSS_UDMA_TCHAN_RT_PCNT_OFS  (0x400u)
#define TI_K3_NAVSS_UDMA_TCHAN_RT_BCNT_OFS  (0x408u)
#define TI_K3_NAVSS_UDMA_TCHAN_RT_SBCNT_OFS (0x410u)

#define TI_K3_NAVSS_UDMA_RCHAN_RT_PEERN_OFS(i)  (0x200u + (i * 0x4u))
#define TI_K3_NAVSS_UDMA_RCHAN_RT_PEER_BCNT_OFS     \
            TI_K3_NAVSS_UDMA_RCHAN_RT_PEERN_OFS(4)      /* PSI-L: 0x404 */
#define TI_K3_NAVSS_UDMA_RCHAN_RT_PEER_RT_EN_OFS    \
            TI_K3_NAVSS_UDMA_RCHAN_RT_PEERN_OFS(8)      /* PSI-L: 0x408 */

#define TI_K3_NAVSS_UDMA_TCHAN_RT_PEERN_OFS(i)  (0x200u + (i * 0x4u))
#define TI_K3_NAVSS_UDMA_TCHAN_RT_PEER_BCNT_OFS     \
            TI_K3_NAVSS_UDMA_TCHAN_RT_PEERN_OFS(4)      /* PSI-L: 0x404 */
#define TI_K3_NAVSS_UDMA_TCHAN_RT_PEER_RT_EN_OFS    \
            TI_K3_NAVSS_UDMA_TCHAN_RT_PEERN_OFS(8)      /* PSI-L: 0x408 */

#define TI_K3_NAVSS_UDMA_PEER_RT_EN_ENABLE      BIT(31)
#define TI_K3_NAVSS_UDMA_PEER_RT_EN_TEARDOWN    BIT(30)


#define TI_K3_NAVSS_UDMA_CHAN_NUM           (96u)       /* for MCU NAVSS UDMA */
#define TI_K3_NAVSS_UDMA_PSIL_BASE          (0x6000u)   /* for MCU NAVSS UDMA */
#define TI_K3_NAVSS_UDMA_TISCI_DEV_ID       (194u)      /* for MCU NAVSS UDMA */
#define TI_K3_NAVSS_UDMA_RX_DESC_NUM        (512u)
#define TI_K3_NAVSS_UDMA_RX_BUF_SIZE        (1536u)

#define TI_K3_NAVSS_UDMA_TX_CHAN_ID         (0u)
#define TI_K3_NAVSS_UDMA_RX_CHAN_ID         (1u)

#define TI_K3_NAVSS_UDMA_CHAN_TCRING_ID     (96u)       /* General ring 96 */
#define TI_K3_NAVSS_UDMA_CHAN_RRING_ID      (97u)       /* General ring 97 */

#define TI_K3_NAVSS_CPSW_PSIL_BASE          (0x7000u)   /* for MCU NAVSS CPSW */

#define TI_K3_NAVSS_PSIL_TISCI_DEV_ID       (119u)      /* for MCU NAVSS PSI-L */

typedef enum tiK3NavssUdmaPktMode
    {
    UDMA_TR_MODE = 0,
    UDMA_PKT_MODE,
    }TI_K3_NAVSS_UDMA_PKT_MODE;

typedef enum tiK3NavssUdmaDir
    {
    UDMA_DEV_TO_MEM = 0,
    UDMA_MEM_TO_DEV,
    }TI_K3_NAVSS_UDMA_DIR;

typedef struct tiK3NavssUdmaTchan
    {
    UINT32                      id;
    VIRT_ADDR                   tchanRegBase;

    TI_K3_NAVSS_RINGACC_RING *  pTRing;      /* transmit ring */
    TI_K3_NAVSS_RINGACC_RING *  pTcRing;     /* transmit completion ring */
    }TI_K3_NAVSS_UDMA_TCHAN;

typedef struct tiK3NavssUdmaRchan
    {
    UINT32                      id;
    VIRT_ADDR                   rchanRegBase;

    TI_K3_NAVSS_RINGACC_RING *  pFdRing;     /* free descriptor ring */
    TI_K3_NAVSS_RINGACC_RING *  pRRing;      /* receive ring */
    }TI_K3_NAVSS_UDMA_RCHAN;

typedef struct tiK3NavssUdmaRflow
    {
    UINT32              id;
    }TI_K3_NAVSS_UDMA_RFLOW;

typedef struct tiK3NavssUdmaChan
    {
    struct tiK3NavssUdmaDrvctrl *   pUdmaDrv;
    
    UINT32                          id;
    TI_K3_NAVSS_UDMA_TCHAN *        pTchan;
    TI_K3_NAVSS_UDMA_RCHAN *        pRchan;
    TI_K3_NAVSS_UDMA_RFLOW *        pRflow;

    UINT32                          srcThread;
    UINT32                          dstThread;
    
    TI_K3_NAVSS_UDMA_PKT_MODE       pktMode;
    BOOL                            needEpib;
    UINT32                          psdSize;

    TI_K3_NAVSS_UDMA_DIR            dir;

    UINT32                          hdescSize;
    TI_K3_NAVSS_UDMA_HOST_DESC *    pTxDesc;
    PHYS_ADDR                       txDescPhyBase;
    void *                          pRxDesc;
    PHYS_ADDR                       rxDescPhyBase;
    void *                          pRxBuf;
    PHYS_ADDR                       rxBufPhyBase;

    BOOL                            txBusy;
    BOOL                            isValid;
    } TI_K3_NAVSS_UDMA_CHAN;

typedef struct tiK3NavssUdmaDrvctrl
    {
    VIRT_ADDR                   cfgRegBase;
    VIRT_ADDR                   rchanrtRegBase;
    VIRT_ADDR                   tchanrtRegBase;

    UINT32                      rchanCnt;
    UINT32                      echanCnt;
    UINT32                      tchanCnt;
    UINT32                      rflowCnt;
    UINT32                      chanCnt;

    UINT32                      psilBase;
    UINT32                      tisciDevId;

    TI_K3_NAVSS_UDMA_CHAN *     pChans;

    TI_K3_NAVSS_UDMA_TCHAN *    pTchans;
    TI_K3_NAVSS_UDMA_RCHAN *    pRchans;
    TI_K3_NAVSS_UDMA_RFLOW *    pRflows;
    } TI_K3_NAVSS_UDMA_DRVCTRL;

typedef struct tiK3NavssUdmaPktData 
    {
    UINT32  pktType;
    UINT32  dstTag;
    } TI_K3_NAVSS_UDMA_PKT_DATA;

STATUS tiK3NavssUdmaSendTry (void);

STATUS tiK3NavssUdmaSend
    (
    void *  pBuf,
    UINT32  len,
    void *  pData
    );

INT32 tiK3NavssUdmaRecieve
    (
    void *  pBuf,
    void *  pData
    );

STATUS tiK3NavssUdmaInit (void);

#ifdef __cplusplus
}
#endif

#endif /* __INCvxbTiK3NavssUdmah */

