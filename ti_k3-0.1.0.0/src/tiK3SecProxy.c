/* tiK3SecProxy.c - TI AM6 Secure Proxy driver */

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
This library provides read and write function for the TI AM6 Secure Proxy.

INCLUDE FILES: am6xxx.h
*/

/* includes */

#include <vxWorks.h>
#include <vsbConfig.h>
#include <stdlib.h>
#include <string.h>
#include <semLib.h>
#include <taskLib.h>
#include <stdio.h>
#include <vmLib.h>
#include <arch/arm/vxbAccessArchLib.h>
#include <pmapLib.h>
#include <am6xxx.h>

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

#define RX_THREAD 11
#define TX_THREAD 13
#define MSG_OFFSET 4
#define MAX_READ_LEN 15
#define SEC_PROXY_THREAD(base, x) (base + (0x1000UL * (x)))
#define SEC_PROXY_TARGET_DATA     0x32c00000
#define SEC_PROXY_SCFG            0x32800000
#define SEC_PROXY_RT              0x32400000

#undef REG_READ_4
#define REG_READ_4(addr) *((volatile UINT32 *)(addr))

#undef REG_WRITE_4
#define REG_WRITE_4(addr, val) *((volatile UINT32 *)(addr)) = val

/* typedefs */

struct k3SecProxyMsg
    {
    UINT32   len;
    UINT32 * buf;
    };

/* forward declarations */

/* locals */

LOCAL VIRT_ADDR secProxyTargetData;
LOCAL VIRT_ADDR secProxyScfg;
LOCAL VIRT_ADDR secProxyRt;

/*******************************************************************************
*
* k3SecProxyInit - initialize the K3 secure proxy
*
* This routine initializes the K3 secure proxy module
*
* RETURNS: OK, or ERROR if any call to pmapGlobalMap() fails
*
* ERRNO: N/A
*
* \NOMANUAL
*/

STATUS k3SecProxyInit (void)
    {
    secProxyTargetData = (VIRT_ADDR) pmapGlobalMap (SEC_PROXY_TARGET_DATA,
                                                    0x100000,
                                                    VXB_REG_MAP_MMU_ATTR);
    secProxyScfg = (VIRT_ADDR) pmapGlobalMap (SEC_PROXY_SCFG, 0x100000,
                                              VXB_REG_MAP_MMU_ATTR);
    secProxyRt = (VIRT_ADDR) pmapGlobalMap (SEC_PROXY_RT, 0x100000,
                                            VXB_REG_MAP_MMU_ATTR);
    if ((void *)secProxyTargetData == PMAP_FAILED ||
        (void *)secProxyScfg == PMAP_FAILED ||
        (void *)secProxyRt == PMAP_FAILED)
        {
        return ERROR;
        }
    else
        {
        return OK;
        }
    }

/*******************************************************************************
*
* k3SecProxySend - send data through the K3 secure proxy
*
* This routine send data through the K3 secure proxy module
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void k3SecProxySend
    (
    struct k3SecProxyMsg * data
    )
    {
    struct k3SecProxyMsg * msg = data;
    VIRT_ADDR              dataReg;
    VIRT_ADDR              lastDataReg;
    UINT32                 words;
    UINT32                 leftBytes;
    UINT32 *               dataWord;

    if (data == NULL)
        {
        DEBUG_MSG ("%s fail: data == NULL !\n", __func__);
        return;
        }

    /* Send the message */

    dataReg = SEC_PROXY_THREAD (secProxyTargetData, TX_THREAD) + MSG_OFFSET;
    lastDataReg = SEC_PROXY_THREAD (secProxyTargetData, TX_THREAD) + 0x3c;
    DEBUG_MSG ("dataReg = %p, lastDataReg=%p\n", dataReg, lastDataReg);

    for (words = msg->len / 4, dataWord = msg->buf;
         words > 0;
         words--, dataReg += 4, dataWord++)
        {
        DEBUG_MSG ("dataReg=%p, dataWord=%p\n", dataReg, *dataWord);
        REG_WRITE_4 (dataReg, *dataWord);
        }

    leftBytes = msg->len % 4;
    if (leftBytes)
        {
        UINT32 leftData = *dataWord;

        /* Ensure all unused data is 0 */

        leftData &= 0xFFFFFFFFU >> (8 * (4 - leftBytes));
        REG_WRITE_4 (dataReg, leftData);
        DEBUG_MSG ("dataReg = %p, dataTrail=%p\n", dataReg, leftData);
        dataReg+=4;
        }

    /* write on last Tx reg to start transmit */

    if (dataReg <= lastDataReg)
        {
        DEBUG_MSG ("lastDataReg = %p\n",lastDataReg);
        REG_WRITE_4 (lastDataReg, 0);
        }

    DEBUG_MSG ("%s: successfully sent\n", __func__);
    }

/*******************************************************************************
*
* keSecProxyRecv - receive data through the K3 secure proxy
*
* This routine receive data through the K3 secure proxy module
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void keSecProxyRecv
    (
    UINT32 * rxBuf
    )
    {
    VIRT_ADDR dataReg;
    int       words;
    UINT32 *  dataWord;

    if (rxBuf == NULL)
        {
        DEBUG_MSG ("%s fail: rxBuf == NULL !\n", __func__);
        return;
        }

    DEBUG_MSG ("%s(rxBuf=%p)\n", __func__, rxBuf);

    dataReg = SEC_PROXY_THREAD (secProxyTargetData, RX_THREAD) + MSG_OFFSET;
    dataWord = rxBuf;
    for (words = MAX_READ_LEN;
         words > 0;
         words--, dataReg += sizeof(UINT32), dataWord++)
        {
        *dataWord = REG_READ_4 (dataReg);
        DEBUG_MSG ("%s dataReg=%p, dataWord=%p\n",__func__, dataReg, *dataWord);
        }

    DEBUG_MSG ("%s: successfully received\n", __func__);
    }
