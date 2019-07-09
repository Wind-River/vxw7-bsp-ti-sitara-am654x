/* tisciClientApi.h - TI SCI client API header file */

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
23jun19,whe  created (VXWPG-114)
*/

/*
DESCRIPTION
This library provides macro / struct definitions which will be used by
tisciClientApi.c
*/

#ifndef __INCtiSciClientApih
#define __INCtiSciClientApih

#ifdef __cplusplus
extern "C" {
#endif

#include <tiSciPredefs.h>
#include <ti_sci.h>

#ifndef BIT
#define BIT(n) (1u << n)
#endif

#define TISCI_RM_NULL_U8                            ((UINT8)~0U)
#define TISCI_RM_NULL_U16                           ((UINT16)~0U)
#define TISCI_RM_NULL_U32                           ((UINT32)~0U)

#define TISCI_RING_MODE_RING                        (0)
#define TISCI_RING_MODE_MESSAGE                     (1)
#define TISCI_RING_MODE_CREDENTIALS                 (2)
#define TISCI_RING_MODE_QM                          (3)

#define TISCI_MSG_VALUE_RM_RING_ADDR_LO_VALID       BIT(0)
#define TISCI_MSG_VALUE_RM_RING_ADDR_HI_VALID       BIT(1) 
#define TISCI_MSG_VALUE_RM_RING_COUNT_VALID         BIT(2)
#define TISCI_MSG_VALUE_RM_RING_MODE_VALID          BIT(3)
#define TISCI_MSG_VALUE_RM_RING_SIZE_VALID          BIT(4)
#define TISCI_MSG_VALUE_RM_RING_ORDER_ID_VALID      BIT(5)

#define TISCI_MSG_VALUE_RM_ALL_NO_ORDER          \
        (TISCI_MSG_VALUE_RM_RING_ADDR_LO_VALID | \
         TISCI_MSG_VALUE_RM_RING_ADDR_HI_VALID | \
         TISCI_MSG_VALUE_RM_RING_COUNT_VALID   | \
         TISCI_MSG_VALUE_RM_RING_MODE_VALID    | \
         TISCI_MSG_VALUE_RM_RING_SIZE_VALID)

#define TISCI_RM_UDMAP_CHAN_TYPE_PKT_PBRR                       2
#define TISCI_RM_UDMAP_CHAN_TYPE_PKT_PBRR_SB                    3
#define TISCI_RM_UDMAP_CHAN_TYPE_3RDP_PBRR                      10
#define TISCI_RM_UDMAP_CHAN_TYPE_3RDP_PBVR                      11
#define TISCI_RM_UDMAP_CHAN_TYPE_3RDP_BCOPY_PBRR                12
#define TISCI_RM_UDMAP_CHAN_TYPE_3RDP_BCOPY_PBVR                13

#define TISCI_RM_UDMAP_ATYPE_PHYS                               0
#define TISCI_RM_UDMAP_ATYPE_INTERMEDIATE                       1
#define TISCI_RM_UDMAP_ATYPE_VIRTUAL                            2

#define TISCI_RM_UDMAP_SCHED_PRIOR_HIGH                         0
#define TISCI_RM_UDMAP_SCHED_PRIOR_MEDHIGH                      1
#define TISCI_RM_UDMAP_SCHED_PRIOR_MEDLOW                       2
#define TISCI_RM_UDMAP_SCHED_PRIOR_LOW                          3

#define TISCI_RM_UDMAP_RX_FLOW_DESC_HOST                        0
#define TISCI_RM_UDMAP_RX_FLOW_DESC_MONO                        2

#define TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERR_VALID          BIT(0)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_VALID                 BIT(1)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_CHAN_TYPE_VALID             BIT(2)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_FETCH_SIZE_VALID            BIT(3)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_CQ_QNUM_VALID               BIT(4)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_PRIORITY_VALID              BIT(5)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_QOS_VALID                   BIT(6)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_ORDER_ID_VALID              BIT(7)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIORITY_VALID        BIT(8)

#define TISCI_MSG_VALUE_RM_UDMAP_CH_TX_FILT_EINFO_VALID         BIT(9)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_TX_FILT_PSWORDS_VALID       BIT(10)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_TX_SUPR_TDPKT_VALID         BIT(11)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_TX_CREDIT_COUNT_VALID       BIT(12)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_TX_FDEPTH_VALID             BIT(13)

#define TISCI_MSG_VALUE_RM_UDMAP_CH_RX_FLOWID_START_VALID       BIT(9)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_RX_FLOWID_CNT_VALID         BIT(10)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_RX_IGNORE_SHORT_VALID       BIT(11)
#define TISCI_MSG_VALUE_RM_UDMAP_CH_RX_IGNORE_LONG_VALID        BIT(12)

#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_EINFO_PRESENT_VALID       BIT(0)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_PSINFO_PRESENT_VALID      BIT(1)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_ERROR_HANDLING_VALID      BIT(2)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_DESC_TYPE_VALID           BIT(3)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_SOP_OFFSET_VALID          BIT(4)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_QNUM_VALID           BIT(5)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_SRC_TAG_HI_VALID          BIT(6)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_SRC_TAG_LO_VALID          BIT(7)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_TAG_HI_VALID         BIT(8)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_TAG_LO_VALID         BIT(9)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_SRC_TAG_HI_SEL_VALID      BIT(10)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_SRC_TAG_LO_SEL_VALID      BIT(11)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_TAG_HI_SEL_VALID     BIT(12)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_DEST_TAG_LO_SEL_VALID     BIT(13)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ0_SZ0_QNUM_VALID       BIT(14)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ1_QNUM_VALID           BIT(15)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ2_QNUM_VALID           BIT(16)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_FDQ3_QNUM_VALID           BIT(17)
#define TISCI_MSG_VALUE_RM_UDMAP_FLOW_PS_LOCATION_VALID         BIT(18)

/* structures */

struct ti_sci_msg_rm_udmap_tx_ch_cfg {
    UINT32 valid_params;
    UINT16 nav_id;
    UINT16 index;
    UINT8  tx_pause_on_err;
    UINT8  tx_filt_einfo;
    UINT8  tx_filt_pswords;
    UINT8  tx_atype;
    UINT8  tx_chan_type;
    UINT8  tx_supr_tdpkt;
    UINT16 tx_fetch_size;
    UINT8  tx_credit_count;
    UINT16 txcq_qnum;
    UINT8  tx_priority;
    UINT8  tx_qos;
    UINT8  tx_orderid;
    UINT16 fdepth;
    UINT8  tx_sched_priority;
    UINT8  tx_burst_size;
};

struct ti_sci_msg_rm_udmap_rx_ch_cfg {
    UINT32 valid_params;
    UINT16 nav_id;
    UINT16 index;
    UINT16 rx_fetch_size;
    UINT16 rxcq_qnum;
    UINT8  rx_priority;
    UINT8  rx_qos;
    UINT8  rx_orderid;
    UINT8  rx_sched_priority;
    UINT16 flowid_start;
    UINT16 flowid_cnt;
    UINT8  rx_pause_on_err;
    UINT8  rx_atype;
    UINT8  rx_chan_type;
    UINT8  rx_ignore_short;
    UINT8  rx_ignore_long;
    UINT8  rx_burst_size;
};

struct ti_sci_msg_rm_udmap_flow_cfg {
    UINT32 valid_params;
    UINT16 nav_id;
    UINT16 flow_index;
    UINT8  rx_einfo_present;
    UINT8  rx_psinfo_present;
    UINT8  rx_error_handling;
    UINT8  rx_desc_type;
    UINT16 rx_sop_offset;
    UINT16 rx_dest_qnum;
    UINT8  rx_src_tag_hi;
    UINT8  rx_src_tag_lo;
    UINT8  rx_dest_tag_hi;
    UINT8  rx_dest_tag_lo;
    UINT8  rx_src_tag_hi_sel;
    UINT8  rx_src_tag_lo_sel;
    UINT8  rx_dest_tag_hi_sel;
    UINT8  rx_dest_tag_lo_sel;
    UINT16 rx_fdq0_sz0_qnum;
    UINT16 rx_fdq1_qnum;
    UINT16 rx_fdq2_qnum;
    UINT16 rx_fdq3_qnum;
    UINT8  rx_ps_location;
};

void tisci_rm_udmap_tx_ch_cfg (struct ti_sci_msg_rm_udmap_tx_ch_cfg *);
void tisci_rm_udmap_rx_ch_cfg (struct ti_sci_msg_rm_udmap_rx_ch_cfg *);
void tisci_rm_udmap_rx_flow_cfg (struct ti_sci_msg_rm_udmap_flow_cfg *);
void tisci_rm_ring_cfg (UINT32, UINT16, UINT16, UINT32, UINT32, UINT32, UINT8,
                        UINT8, UINT8);
void tisci_rm_psil_pair (UINT32, UINT32, UINT32);
void tisci_rm_psil_unpair (UINT32, UINT32, UINT32);
void tisci_set_clock (UINT32, UINT8, UINT8);
void tisci_get_freq (UINT32, UINT8, UINT64 *);
void tisci_set_device (UINT32, UINT8);

#ifdef __cplusplus
}

#endif /* __INCtiSciClientApih */
#endif
