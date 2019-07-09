/* vxbTiK3NavssUdmaDesc.h - TI AM65x MCU NAVSS UDMA Descriptor header */

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

#ifndef __INCvxbTiK3NavssUdmaDesch
#define __INCvxbTiK3NavssUdmaDesch

#ifdef __cplusplus
extern "C"
{
#endif

/* defines */

#ifndef BIT
#define BIT(n)     (1u << (n))
#endif

#define TI_K3_NAVSS_UDMA_DESC_ALIGN                     (16u)

#define TI_K3_NAVSS_UDMA_HDESC_INFO0_PSDATA_MAX_SIZE    (128u)
#define TI_K3_NAVSS_UDMA_HDESC_INFO0_EPIB_SIZE          (16u)
#define TI_K3_NAVSS_UDMA_HDESC_INFO0_EPIB_PRESENT       BIT(29)
#define TI_K3_NAVSS_UDMA_HDESC_INFO0_PS_SIZE_OFS        (22u)
#define TI_K3_NAVSS_UDMA_HDESC_INFO0_PS_LOCATION        BIT(28)
#define TI_K3_NAVSS_UDMA_HDESC_INFO0_PS_SIZE_MASK       (0xFC00000u)
#define TI_K3_NAVSS_UDMA_HDESC_INFO0_PKTLEN_MASK        (0X3FFFFFu)
#define TI_K3_NAVSS_UDMA_HDESC_INFO0_TYPE_OFS           (30u)
#define TI_K3_NAVSS_UDMA_HDESC_INFO0_HOST_TYPE          (1u)

#define TI_K3_NAVSS_UDMA_HDESC_BUFINFO_LEN_MASK         (0X3FFFFFu)
#define TI_K3_NAVSS_UDMA_HDESC_OBUFINFO_LEN_MASK        (0X3FFFFFu)

#define TI_K3_NAVSS_UDMA_HDESC_INFO1_PKTID_OFS          (14u)
#define TI_K3_NAVSS_UDMA_HDESC_INFO1_PKTID_MASK         (0XFFC000u)
#define TI_K3_NAVSS_UDMA_HDESC_INFO1_FLOWID_OFS         (0)
#define TI_K3_NAVSS_UDMA_HDESC_INFO1_FLOWID_MASK        (0X3FFFu)

#define TI_K3_NAVSS_UDMA_HDESC_INFO2_RETRING_MASK       (0XFFFFu)
#define TI_K3_NAVSS_UDMA_HDESC_INFO2_PKTTYPE_OFS        (27u)
#define TI_K3_NAVSS_UDMA_HDESC_INFO2_PKTTYPE_MASK       (0XF8000000u)

#define TI_K3_NAVSS_UDMA_HDESC_INFO3_SRCTAG_OFS         (16u)
#define TI_K3_NAVSS_UDMA_HDESC_INFO3_SRCTAG_MASK        (0XFFFF0000u)
#define TI_K3_NAVSS_UDMA_HDESC_INFO3_DSTTAG_MASK        (0XFFFFu)

/* descriptor header */

typedef struct tiK3NavssUdmaDescHdr
    {
    UINT32  pktInfo0;    /* Packet info word 0 */
    UINT32  pktInfo1;    /* Packet info word 1 */
    UINT32  pktInfo2;    /* Packet info word 2 */
    UINT32  srcDstTag;   /* Packet info word 3 */
    } WRS_PACK_ALIGN(1) TI_K3_NAVSS_UDMA_DESC_HDR;

typedef struct tiK3NavssUdmaHostDesc
    {
    TI_K3_NAVSS_UDMA_DESC_HDR   hdr;
    UINT64  nextDesc;   /* Linking word 0,1 */
    UINT64  bufPtr;     /* Buffer 0 pointer */
    UINT32  bufInfo;    /* Buffer 0 valid data length */
    UINT32  orgBufLen;  /* Original buffer length */
    UINT64  orgBufPtr;  /* Original buffer pointer */
    /* Extended Packet Info Block (optional, 16 bytes) */
    /* Protocol Specific Data (optional) */
    /* Other Software Data (optional) */
    } WRS_PACK_ALIGN(1) TI_K3_NAVSS_UDMA_HOST_DESC;

UINT32 tiK3NavssUdmaHdescSizeCalc
    (
    BOOL    epib, 
    UINT32  psDataSize,
    UINT32  swDataSize
    );

void tiK3NavssUdmaHdescRst
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc
    );

STATUS tiK3NavssUdmaHdescInit
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc,
    UINT32                          flags,
    UINT32                          psDataSize
    );

void tiK3NavssUdmaHdescPktlenSet
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc,
    UINT32                          pktLen
    );

void tiK3NavssUdmaHdescBufSet
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc,
    PHYS_ADDR                       bufAddr,
    UINT32                          bufLen,
    PHYS_ADDR                       oBufAddr,
    UINT32                          oBufLen
    );

void tiK3NavssUdmaHdescPktIdSet
    (
    TI_K3_NAVSS_UDMA_DESC_HDR * pDescHdr,
    UINT32                      pktId,
    UINT32                      flowId
    );

void tiK3NavssUdmaHdescRetPolicySet
    (
    TI_K3_NAVSS_UDMA_DESC_HDR * pDescHdr,
    UINT32                      flags,
    UINT32                      retRingId
    );

void tiK3NavssUdmaHdescPktTypeSet
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc,
    UINT32                          pktType
    );

void tiK3NavssUdmaHdescTagIdSet
    (
    TI_K3_NAVSS_UDMA_DESC_HDR * pDescHdr,
    UINT32                      srcTagId,
    UINT32                      dstTagId
    );

void tiK3NavssUdmaHdescObufGet
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc,
    PHYS_ADDR *                     pObufAddr,
    UINT32 *                        pObufLen
    );

UINT32 tiK3NavssUdmaHdescPktlenGet
    (
    TI_K3_NAVSS_UDMA_HOST_DESC *    pDesc
    );

#ifdef __cplusplus
}
#endif

#endif /* __INCvxbTiK3NavssUdmaDesch */

