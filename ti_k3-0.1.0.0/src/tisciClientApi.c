/* tisciClientApi.c - TISCI client API lib */

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
10jun19,whe  created (VXWPG-114)
*/

/*
DESCRIPTION
This library provides implementation of some TISCI client APIs.
The API naming method is to turn upper case TISCI Message name to lower case
VxWorks function name without "msg" field. For example:

TISCI_MSG_RM_UDMAP_TX_CH_CFG ==> tisci_rm_udmap_tx_ch_cfg ()
TISCI_MSG_RM_RING_CFG ==> tisci_rm_ring_cfg ()
TISCI_MSG_SET_CLOCK ==> tisci_set_clock ()

INCLUDE FILES: tisciClientApi.h
*/

/* includes */

#include <vxWorks.h>
#include <vsbConfig.h>
#include <string.h>
#include <subsys/timer/vxbTimerLib.h>
#include <tiSciClientApi.h>

/* defines */

#undef DEBUG_ENABLE
#ifdef DEBUG_ENABLE
#include <private/kwriteLibP.h>     /* _func_kprintf */

#undef LOCAL
#define LOCAL

#undef DEBUG_MSG
#define DEBUG_MSG(...)                                  \
    do                                                  \
        {                                               \
        if (_func_kprintf != NULL)                      \
            {                                           \
            (* _func_kprintf)(__VA_ARGS__);             \
            }                                           \
        }                                               \
    while ((FALSE))
#else
#undef DEBUG_MSG
#define DEBUG_MSG(...)
#endif  /* DEBUG_ENABLE */

/* typedefs */

struct k3SecProxyMsg
    {
    UINT32 len;
    UINT32 * buf;
    };

/* forward declarations */

/* externs */

IMPORT void k3SecProxySend (struct k3SecProxyMsg * data);
IMPORT void keSecProxyRecv (UINT32 * rxBuf);

/*******************************************************************************
*
* tisci_rm_udmap_tx_ch_cfg - UDMAP Transmit Channel Configure Request
*
* This routine implements TISCI API: TISCI_MSG_RM_UDMAP_TX_CH_CFG
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void tisci_rm_udmap_tx_ch_cfg
    (
    struct ti_sci_msg_rm_udmap_tx_ch_cfg * params
    )
    {
    struct ti_sci_msg_rm_udmap_tx_ch_cfg_resp * resp;
    struct ti_sci_msg_rm_udmap_tx_ch_cfg_req req;
    struct k3SecProxyMsg msg;
    struct ti_sci_msg_hdr * hdr = &req.hdr;
    char msgbuf[64];

    bzero (msgbuf, 64);

    msg.len = sizeof (req);
    msg.buf = (UINT32 *) &req;

    hdr->type = TISCI_MSG_RM_UDMAP_TX_CH_CFG;
    hdr->host = 12,
    hdr->seq = 0,
    hdr->flags = TI_SCI_FLAG_REQ_ACK_ON_PROCESSED;
    req.valid_params = params->valid_params;
    req.nav_id = params->nav_id;
    req.index = params->index;
    req.tx_pause_on_err = params->tx_pause_on_err;
    req.tx_filt_einfo = params->tx_filt_einfo;
    req.tx_filt_pswords = params->tx_filt_pswords;
    req.tx_atype = params->tx_atype;
    req.tx_chan_type = params->tx_chan_type;
    req.tx_supr_tdpkt = params->tx_supr_tdpkt;
    req.tx_fetch_size = params->tx_fetch_size;
    req.tx_credit_count = params->tx_credit_count;
    req.txcq_qnum = params->txcq_qnum;
    req.tx_priority = params->tx_priority;
    req.tx_qos = params->tx_qos;
    req.tx_orderid = params->tx_orderid;
    req.fdepth = params->fdepth;
    req.tx_sched_priority = params->tx_sched_priority;

    k3SecProxySend ((struct k3SecProxyMsg *) &msg);
    vxbUsDelay (1000);
    keSecProxyRecv ((UINT32*) msgbuf);
    resp = (struct ti_sci_msg_rm_udmap_tx_ch_cfg_resp *) msgbuf;
    DEBUG_MSG ("resp->hdr.flags=%d\n",resp->hdr.flags);
    }

/*******************************************************************************
*
* tisci_rm_udmap_rx_ch_cfg - UDMAP Receive Channel Configure Request
*
* This routine implements TISCI API: TISCI_MSG_RM_UDMAP_RX_CH_CFG
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void tisci_rm_udmap_rx_ch_cfg
    (
    struct ti_sci_msg_rm_udmap_rx_ch_cfg * params
    )
    {
    struct ti_sci_msg_rm_udmap_rx_ch_cfg_resp * resp;
    struct ti_sci_msg_rm_udmap_rx_ch_cfg_req req;
    struct k3SecProxyMsg msg;
    struct ti_sci_msg_hdr * hdr = &req.hdr;
    char msgbuf[64];

    bzero (msgbuf, 64);
    msg.len = sizeof (req);
    msg.buf = (UINT32 *) &req;

    hdr->type = TISCI_MSG_RM_UDMAP_RX_CH_CFG;
    hdr->host = 12,
    hdr->seq = 0,
    hdr->flags = TI_SCI_FLAG_REQ_ACK_ON_PROCESSED;

    req.valid_params = params->valid_params;
    req.nav_id = params->nav_id;
    req.index = params->index;
    req.rx_fetch_size = params->rx_fetch_size;
    req.rxcq_qnum = params->rxcq_qnum;
    req.rx_priority = params->rx_priority;
    req.rx_qos = params->rx_qos;
    req.rx_orderid = params->rx_orderid;
    req.rx_sched_priority = params->rx_sched_priority;
    req.flowid_start = params->flowid_start;
    req.flowid_cnt = params->flowid_cnt;
    req.rx_pause_on_err = params->rx_pause_on_err;
    req.rx_atype = params->rx_atype;
    req.rx_chan_type = params->rx_chan_type;
    req.rx_ignore_short = params->rx_ignore_short;
    req.rx_ignore_long = params->rx_ignore_long;

    k3SecProxySend ((struct k3SecProxyMsg *) &msg);
    vxbUsDelay (1000);
    keSecProxyRecv ((UINT32*) msgbuf);
    resp = (struct ti_sci_msg_rm_udmap_rx_ch_cfg_resp *) msgbuf;
    DEBUG_MSG ("resp->hdr.flags=%d\n",resp->hdr.flags);
    }

/*******************************************************************************
*
* tisci_rm_udmap_rx_flow_cfg - UDMAP Receive Flow Configure Request
*
* This routine implements TISCI API: TISCI_MSG_RM_UDMAP_FLOW_CFG
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void tisci_rm_udmap_rx_flow_cfg
    (
    struct ti_sci_msg_rm_udmap_flow_cfg * params
    )
    {
    struct ti_sci_msg_rm_udmap_flow_cfg_resp * resp;
    struct ti_sci_msg_rm_udmap_flow_cfg_req req;
    struct k3SecProxyMsg msg;
    struct ti_sci_msg_hdr * hdr = &req.hdr;
    char msgbuf[64];

    bzero (msgbuf, 64);
    msg.len = sizeof (req);
    msg.buf = (UINT32 *) &req;

    hdr->type = TISCI_MSG_RM_UDMAP_FLOW_CFG;
    hdr->host = 12,
    hdr->seq = 0,
    hdr->flags = TI_SCI_FLAG_REQ_ACK_ON_PROCESSED;

    req.valid_params = params->valid_params;
    req.nav_id = params->nav_id;
    req.flow_index = params->flow_index;
    req.rx_einfo_present = params->rx_einfo_present;
    req.rx_psinfo_present = params->rx_psinfo_present;
    req.rx_error_handling = params->rx_error_handling;
    req.rx_desc_type = params->rx_desc_type;
    req.rx_sop_offset = params->rx_sop_offset;
    req.rx_dest_qnum = params->rx_dest_qnum;
    req.rx_src_tag_hi = params->rx_src_tag_hi;
    req.rx_src_tag_lo = params->rx_src_tag_lo;
    req.rx_dest_tag_hi = params->rx_dest_tag_hi;
    req.rx_dest_tag_lo = params->rx_dest_tag_lo;
    req.rx_src_tag_hi_sel = params->rx_src_tag_hi_sel;
    req.rx_src_tag_lo_sel = params->rx_src_tag_lo_sel;
    req.rx_dest_tag_hi_sel = params->rx_dest_tag_hi_sel;
    req.rx_dest_tag_lo_sel = params->rx_dest_tag_lo_sel;
    req.rx_fdq0_sz0_qnum = params->rx_fdq0_sz0_qnum;
    req.rx_fdq1_qnum = params->rx_fdq1_qnum;
    req.rx_fdq2_qnum = params->rx_fdq2_qnum;
    req.rx_fdq3_qnum = params->rx_fdq3_qnum;
    req.rx_ps_location = params->rx_ps_location;

    k3SecProxySend ((struct k3SecProxyMsg *) &msg);
    vxbUsDelay (1000);
    keSecProxyRecv ((UINT32*) msgbuf);
    resp = (struct ti_sci_msg_rm_udmap_flow_cfg_resp *) msgbuf;
    DEBUG_MSG ("resp->hdr.flags=%d\n",resp->hdr.flags);
    }

/*******************************************************************************
*
* tisci_rm_ring_cfg - Ring Accelerator Ring Configure Request
*
* This routine implements TISCI API: TISCI_MSG_RM_RING_CFG
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void tisci_rm_ring_cfg
    (
    UINT32 valid_params,
    UINT16 nav_id,
    UINT16 index,
    UINT32 addr_lo,
    UINT32 addr_hi,
    UINT32 count,
    UINT8  mode,
    UINT8  size,
    UINT8  order_id
    )
    {
    struct ti_sci_msg_rm_ring_cfg_resp * resp;
    struct ti_sci_msg_rm_ring_cfg_req req;
    struct k3SecProxyMsg msg;
    struct ti_sci_msg_hdr * hdr = &req.hdr;
    char msgbuf[64];

    bzero (msgbuf, 64);
    msg.len = sizeof (req);
    msg.buf = (UINT32 *) &req;

    hdr->type = TI_SCI_MSG_RM_RING_CFG;
    hdr->host = 12,
    hdr->seq = 0,
    hdr->flags = TI_SCI_FLAG_REQ_ACK_ON_PROCESSED;

    req.valid_params = valid_params;
    req.nav_id = nav_id;
    req.index = index;
    req.addr_lo = addr_lo;
    req.addr_hi = addr_hi;
    req.count = count;
    req.mode = mode;
    req.size = size;
    req.order_id = order_id;

    k3SecProxySend ((struct k3SecProxyMsg *) &msg);
    vxbUsDelay (1000);
    keSecProxyRecv ((UINT32*) msgbuf);
    resp = (struct ti_sci_msg_rm_ring_cfg_resp *) msgbuf;
    DEBUG_MSG ("resp->hdr.flags=%d\n",resp->hdr.flags);
    }

/*******************************************************************************
*
* tisci_rm_psil_pair - PSI-L Pair
*
* This routine implements TISCI API: TISCI_MSG_RM_PSIL_PAIR
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void tisci_rm_psil_pair
    (
    UINT32 nav_id,
    UINT32 src_thread,
    UINT32 dst_thread
    )
    {
    struct ti_sci_msg_hdr * resp;
    struct ti_sci_msg_psil_pair req;
    struct k3SecProxyMsg msg;
    struct ti_sci_msg_hdr * hdr = &req.hdr;
    char msgbuf[64];

    bzero (msgbuf, 64);
    msg.len = sizeof (req);
    msg.buf = (UINT32 *) &req;

    hdr->type = TI_SCI_MSG_RM_PSIL_PAIR;
    hdr->host = 12,
    hdr->seq = 0,
    hdr->flags = TI_SCI_FLAG_REQ_ACK_ON_PROCESSED;

    req.nav_id = nav_id;
    req.src_thread = src_thread;
    req.dst_thread = dst_thread;

    k3SecProxySend ((struct k3SecProxyMsg *) &msg);
    vxbUsDelay (10000);
    keSecProxyRecv ((UINT32*) msgbuf);
    resp = (struct ti_sci_msg_hdr *) msgbuf;
    DEBUG_MSG ("resp->flags=%d\n",resp->flags);
    }

/*******************************************************************************
*
* tisci_rm_psil_unpair - PSI-L Unpair
*
* This routine implements TISCI API: TISCI_MSG_RM_PSIL_UNPAIR
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void tisci_rm_psil_unpair
    (
    UINT32 nav_id,
    UINT32 src_thread,
    UINT32 dst_thread
    )
    {
    struct ti_sci_msg_hdr * resp;
    struct ti_sci_msg_psil_unpair req;
    struct k3SecProxyMsg msg;
    struct ti_sci_msg_hdr * hdr = &req.hdr;
    char msgbuf[64];

    bzero (msgbuf, 64);
    msg.len = sizeof (req);
    msg.buf = (UINT32 *) &req;

    hdr->type = TI_SCI_MSG_RM_PSIL_UNPAIR;
    hdr->host = 12,
    hdr->seq = 0,
    hdr->flags = TI_SCI_FLAG_REQ_ACK_ON_PROCESSED;

    req.nav_id = nav_id;
    req.src_thread = src_thread;
    req.dst_thread = dst_thread;

    k3SecProxySend ((struct k3SecProxyMsg *) &msg);
    vxbUsDelay (10000);
    keSecProxyRecv ((UINT32*) msgbuf);
    resp = (struct ti_sci_msg_hdr *) msgbuf;
    DEBUG_MSG ("resp->flags=%d\n",resp->flags);
    }

/*******************************************************************************
*
* tisci_set_clock - Setup a hardware device’s clock state
*
* This routine implements TISCI API: TISCI_MSG_SET_CLOCK
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void tisci_set_clock
    (
    UINT32 dev_id,
    UINT8  clk_id,
    UINT8  state
    )
    {
    struct ti_sci_msg_hdr * resp;
    struct ti_sci_msg_req_set_clock_state req;
    struct k3SecProxyMsg msg;
    struct ti_sci_msg_hdr * hdr = &req.hdr;
    char msgbuf[64];

    bzero (msgbuf, 64);
    msg.len = sizeof (req);
    msg.buf = (UINT32 *) &req;

    hdr->type = TI_SCI_MSG_SET_CLOCK_STATE;
    hdr->host = 12,
    hdr->seq = 0,
    hdr->flags = TI_SCI_FLAG_REQ_ACK_ON_PROCESSED;

    req.dev_id = dev_id;
    req.clk_id = clk_id;
    req.request_state = state;

    k3SecProxySend ((struct k3SecProxyMsg *) &msg);
    vxbUsDelay (10000);
    keSecProxyRecv ((UINT32*) msgbuf);
    resp = (struct ti_sci_msg_hdr *) msgbuf;
    DEBUG_MSG ("resp->flags=%d\n",resp->flags);
    }

/*******************************************************************************
*
* tisci_get_freq - Get the clock frequency of a specific clock
*
* This routine implements TISCI API: TISCI_MSG_GET_FREQ
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void tisci_get_freq
    (
    UINT32   dev_id,
    UINT8    clk_id,
    UINT64 * freq
    )
    {
    struct ti_sci_msg_resp_get_clock_freq * resp;
    struct ti_sci_msg_req_get_clock_freq req;
    struct k3SecProxyMsg msg;
    struct ti_sci_msg_hdr * hdr = &req.hdr;
    char msgbuf[64];

    bzero (msgbuf, 64);
    msg.len = sizeof (req);
    msg.buf = (UINT32 *) &req;

    hdr->type = TI_SCI_MSG_GET_CLOCK_FREQ;
    hdr->host = 12,
    hdr->seq = 0,
    hdr->flags = TI_SCI_FLAG_REQ_ACK_ON_PROCESSED;

    req.dev_id = dev_id;
    req.clk_id = clk_id;

    k3SecProxySend ((struct k3SecProxyMsg *) &msg);
    vxbUsDelay (10000);
    bzero (msgbuf, 64);
    keSecProxyRecv ((UINT32*) msgbuf);
    resp = (struct ti_sci_msg_resp_get_clock_freq *) msgbuf;
    DEBUG_MSG ("resp->freq_hz=%lld\n",resp->freq_hz);
    *freq = resp->freq_hz;
    }

/*******************************************************************************
*
* tisci_set_device - Request for a device state to be set
*
* This routine implements TISCI API: TISCI_MSG_SET_DEVICE
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void tisci_set_device
    (
    UINT32 id,
    UINT8  state
    )
    {
    struct ti_sci_msg_hdr * resp;
    struct ti_sci_msg_req_set_device_state req;
    struct k3SecProxyMsg msg;
    struct ti_sci_msg_hdr * hdr = &req.hdr;
    char msgbuf[64];

    bzero (msgbuf, 64);
    msg.len = sizeof (req);
    msg.buf = (UINT32 *) &req;

    hdr->type = TI_SCI_MSG_SET_CLOCK_STATE;
    hdr->host = 12,
    hdr->seq = 0,
    hdr->flags = TI_SCI_FLAG_REQ_ACK_ON_PROCESSED | MSG_FLAG_DEVICE_EXCLUSIVE;

    req.id = id;
    req.state = state;

    k3SecProxySend ((struct k3SecProxyMsg *) &msg);
    vxbUsDelay (10000);
    keSecProxyRecv ((UINT32*) msgbuf);
    resp = (struct ti_sci_msg_hdr *) msgbuf;
    DEBUG_MSG ("resp->flags=%d\n",resp->flags);
    }

/*******************************************************************************
*
* k3PowerOnDev - power on a device
*
* This routine power on a device with device id
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void k3PowerOnDev
    (
    UINT32 id
    )
    {
    tisci_set_device (id, MSG_DEVICE_SW_STATE_ON);
    }

/*******************************************************************************
*
* k3PowerOffDev - power off a device
*
* This routine power off a device with device id
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void k3PowerOffDev
    (
    UINT32 id
    )
    {
    tisci_set_device (id, MSG_DEVICE_SW_STATE_AUTO_OFF);
    }

/*******************************************************************************
*
* k3EnableClk - enable clock of a device
*
* This routine enable a device clock with device id and clock id
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void k3EnableClk
    (
    UINT32 id,
    UINT8  clk_id
    )
    {
    tisci_set_clock (id, clk_id, MSG_CLOCK_SW_STATE_AUTO);
    k3PowerOnDev (id);
    }

/*******************************************************************************
*
* k3GetClkRate - get clock rate of a device clock
*
* This routine get a device clock rate with device id and clock id
*
* RETURNS: clock rate
*
* ERRNO: N/A
*
* \NOMANUAL
*/

UINT64 k3GetClkRate
    (
    UINT32 id,
    UINT8  clk_id
    )
    {
    UINT64 rate;
    tisci_get_freq (id, clk_id, &rate);
    return rate;
    }
