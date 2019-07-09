/* vxbTiK3NavssUdmaDesc.c - TI AM65x MCU NAVSS UDMA Descriptor driver */

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
This module implements the common operations to consturt a pakcet descriptor of
MCU NAVSS UDMA.

The MCU NAVSS UDMA driver (which is in vxbTiK3NavssUdma.c) will use this driver
to build the host packet descriptor.

INCLUDE FILES: vxWorks.h stdio.h string.h
               vxbTiK3NavssUdmaDesc.h
*/

/* includes */

#include <vxWorks.h>
#include <stdio.h>
#include <string.h>
#include "vxbTiK3NavssUdmaDesc.h"

/* defines */

/* debug macro */

#undef  DEBUG_MSG
#undef  K3_NAVSS_UDMA_DESC_DBG
#ifdef  K3_NAVSS_UDMA_DESC_DBG

/* turning local symbols into global symbols */

#ifdef  LOCAL
#undef  LOCAL
#define LOCAL
#endif

#include <private/kwriteLibP.h>         /* _func_kprintf */
#define K3_NAVSS_UDMA_DESC_DBG_OFF           0x00000000
#define K3_NAVSS_UDMA_DESC_DBG_ISR           0x00000001
#define K3_NAVSS_UDMA_DESC_DBG_ERR           0x00000002
#define K3_NAVSS_UDMA_DESC_DBG_INFO          0x00000004
#define K3_NAVSS_UDMA_DESC_DBG_ALL           0xffffffff

LOCAL UINT32 k3NavssUdmaDescDbgMask = K3_NAVSS_UDMA_DESC_DBG_ALL;

#define DEBUG_MSG(mask, ...)                                                  \
    do                                                                        \
        {                                                                     \
        if ((k3NavssUdmaDescDbgMask & (mask)) \
            || ((mask) == K3_NAVSS_UDMA_DESC_DBG_ALL))  \
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
#endif  /* K3_NAVSS_UDMA_DESC_DBG */

/*******************************************************************************
*
* tiK3NavssUdmaHdescSizeCalc - calculate the length of host packet descriptor
*
* This routine calculates the length of a host packet descriptor with optional 
* fields. The length is aligned to the minimum size of host packet descriptor.
*
* RETURNS: The length of host packet descriptor.
*
* ERRNO: N/A
*/

UINT32 tiK3NavssUdmaHdescSizeCalc
    (
    BOOL    epib,
    UINT32  psDataSize,
    UINT32  swDataSize
    )
    {
    UINT32  descSize;

    if (psDataSize > TI_K3_NAVSS_UDMA_HDESC_INFO0_PSDATA_MAX_SIZE)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DESC_DBG_ERR, 
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return 0;
        }

    descSize = sizeof(TI_K3_NAVSS_UDMA_HOST_DESC) + psDataSize + swDataSize;

    if (epib)
        {
        descSize += TI_K3_NAVSS_UDMA_HDESC_INFO0_EPIB_SIZE;
        }

    return (UINT32) ROUND_UP (descSize, TI_K3_NAVSS_UDMA_DESC_ALIGN);
    }

/*******************************************************************************
*
* tiK3NavssUdmaHdescRst - reset the host packet descriptor
*
* This routine resets the host packet descriptor.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

void tiK3NavssUdmaHdescRst
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc
    )
    {
    if (pDesc == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DESC_DBG_ERR, 
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return;
        }

    memset (&pDesc->hdr, 0, sizeof(TI_K3_NAVSS_UDMA_DESC_HDR));
    pDesc->nextDesc = 0;
    }

/*******************************************************************************
*
* tiK3NavssUdmaHdescInit - initialize the host packet descriptor
*
* This routine initializes the host packet descriptor.
*
* RETURNS: OK, or ERROR if initialize failed.
*
* ERRNO: N/A
*/

STATUS tiK3NavssUdmaHdescInit
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc,
    UINT32                          flags,
    UINT32                          psDataSize
    )
    {
    if (pDesc == NULL 
        || psDataSize > TI_K3_NAVSS_UDMA_HDESC_INFO0_PSDATA_MAX_SIZE
        || flags & (UINT32) ~(TI_K3_NAVSS_UDMA_HDESC_INFO0_EPIB_PRESENT |
                              TI_K3_NAVSS_UDMA_HDESC_INFO0_PS_LOCATION))
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DESC_DBG_ERR, 
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return ERROR;
        }

    pDesc->hdr.pktInfo0 = (TI_K3_NAVSS_UDMA_HDESC_INFO0_HOST_TYPE <<
                           TI_K3_NAVSS_UDMA_HDESC_INFO0_TYPE_OFS) | (flags);

    pDesc->hdr.pktInfo0 |= ((psDataSize / 4) << 
                            TI_K3_NAVSS_UDMA_HDESC_INFO0_PS_SIZE_OFS)
                           & TI_K3_NAVSS_UDMA_HDESC_INFO0_PS_SIZE_MASK;

    pDesc->nextDesc = 0;

    return OK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaHdescPktlenGet - set the packet length to the host packet 
                                descriptor header
*
* This routine sets the packet length to the host packet descriptor header.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

void tiK3NavssUdmaHdescPktlenSet
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc,
    UINT32                          pktLen
    )
    {
    if (pDesc == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DESC_DBG_ERR, 
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return;
        }

    pDesc->hdr.pktInfo0 |= (pktLen & TI_K3_NAVSS_UDMA_HDESC_INFO0_PKTLEN_MASK);
    }

/*******************************************************************************
*
* tiK3NavssUdmaHdescPktIdSet - set the packet buffer address and length and 
*                              original buffer address and length to the host 
*                              packet descriptor
*
* This routine sets the packet buffer address and length and original buffer
* address and length to the host packet descriptor.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

void tiK3NavssUdmaHdescBufSet
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc,
    PHYS_ADDR                       bufAddr,
    UINT32                          bufLen,
    PHYS_ADDR                       oBufAddr,
    UINT32                          oBufLen
    )
    {
    if (pDesc == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DESC_DBG_ERR, 
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return;
        }

    pDesc->bufPtr = bufAddr;
    pDesc->bufInfo = bufLen & TI_K3_NAVSS_UDMA_HDESC_BUFINFO_LEN_MASK;
    pDesc->orgBufPtr = oBufAddr;
    pDesc->orgBufLen = oBufLen & TI_K3_NAVSS_UDMA_HDESC_OBUFINFO_LEN_MASK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaHdescPktIdSet - set the packet ID and flow ID to the host 
*                              packet descriptor header
*
* This routine sets the packet ID and flow ID to the host packet descriptor
* header.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

void tiK3NavssUdmaHdescPktIdSet
    (
    TI_K3_NAVSS_UDMA_DESC_HDR * pDescHdr,
    UINT32                      pktId,
    UINT32                      flowId
    )
    {
    if (pDescHdr == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DESC_DBG_ERR, 
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return;
        }

    pDescHdr->pktInfo1 |= (pktId << TI_K3_NAVSS_UDMA_HDESC_INFO1_PKTID_OFS)
                          & TI_K3_NAVSS_UDMA_HDESC_INFO1_PKTID_MASK;
    pDescHdr->pktInfo1 |= (flowId << TI_K3_NAVSS_UDMA_HDESC_INFO1_FLOWID_OFS)
                          & TI_K3_NAVSS_UDMA_HDESC_INFO1_FLOWID_MASK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaHdescRetPolicySet - set the packet return ring ID to the host 
*                                packet descriptor header
*
* This routine sets the packet return ring ID to the host packet descriptor
* header.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

void tiK3NavssUdmaHdescRetPolicySet
    (
    TI_K3_NAVSS_UDMA_DESC_HDR * pDescHdr,
    UINT32                      flags,
    UINT32                      retRingId
    )
    {
    if (pDescHdr == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DESC_DBG_ERR, 
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return;
        }

    pDescHdr->pktInfo2 |= flags;
    pDescHdr->pktInfo2 |= retRingId & TI_K3_NAVSS_UDMA_HDESC_INFO2_RETRING_MASK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaHdescPktTypeSet - set the packet type to the host packet 
*                                descriptor header
*
* This routine sets the packet type to the host packet descriptor header.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

void tiK3NavssUdmaHdescPktTypeSet
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc,
    UINT32                          pktType
    )
    {
    if (pDesc == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DESC_DBG_ERR, 
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return;
        }

    pDesc->hdr.pktInfo2 |= (pktType << TI_K3_NAVSS_UDMA_HDESC_INFO2_PKTTYPE_OFS)
                           & TI_K3_NAVSS_UDMA_HDESC_INFO2_PKTTYPE_MASK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaHdescTagIdSet - set the source tag and dest tag to the host 
*                              packet descriptor header
*
* This routine sets the source tag and dest tag to the host packet descriptor
* header.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

void tiK3NavssUdmaHdescTagIdSet
    (
    TI_K3_NAVSS_UDMA_DESC_HDR * pDescHdr,
    UINT32                      srcTagId,
    UINT32                      dstTagId
    )
    {
    if (pDescHdr == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DESC_DBG_ERR, 
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return;
        }

    pDescHdr->srcDstTag = (srcTagId << TI_K3_NAVSS_UDMA_HDESC_INFO3_SRCTAG_OFS)
                          & TI_K3_NAVSS_UDMA_HDESC_INFO3_SRCTAG_MASK;
    pDescHdr->srcDstTag |= dstTagId & TI_K3_NAVSS_UDMA_HDESC_INFO3_DSTTAG_MASK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaHdescObufGet - get the original buffer address and length from
*                             the host packet descriptor
*
* This routine gets the original buffer address and length form the host 
* packet descriptor.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

void tiK3NavssUdmaHdescObufGet
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc,
    PHYS_ADDR *                     pObufAddr,
    UINT32 *                        pObufLen
    )
    {
    if (pDesc == NULL || pObufAddr == NULL || pObufLen == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DESC_DBG_ERR, 
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return;
        }

    *pObufAddr = pDesc->orgBufPtr;
    *pObufLen = pDesc->orgBufLen & TI_K3_NAVSS_UDMA_HDESC_OBUFINFO_LEN_MASK;
    }

/*******************************************************************************
*
* tiK3NavssUdmaHdescPktlenGet - get the packet length from the host packet 
                                descriptor header
*
* This routine gets the packet length form the host packet descriptor header.
*
* RETURNS: The packet length.
*
* ERRNO: N/A
*/

UINT32 tiK3NavssUdmaHdescPktlenGet
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc
    )
    {
    if (pDesc == NULL)
        {
        DEBUG_MSG (K3_NAVSS_UDMA_DESC_DBG_ERR, 
                   "ERROR! %s() Line:%d\n", __func__, __LINE__);
        return 0;
        }

    return (pDesc->hdr.pktInfo0 & TI_K3_NAVSS_UDMA_HDESC_INFO0_PKTLEN_MASK);
    }

