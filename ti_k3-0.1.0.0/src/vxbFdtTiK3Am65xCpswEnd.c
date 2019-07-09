/* vxbFdtTiK3Am65xCpswEnd.c - TI K3 Gigabit Ethernet MAC VxBus END driver */

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
11jun19,g_x  written based on vxbFdtTiCpswEnd.c (VXWPG-114)
*/

/*
DESCRIPTION

This module implements an END driver for the TI K3 MCU domain Gigabit 
Ethernet MAC(MCU_CPSW0) network interface. The TI MCU_CPSW0 is 2 ports
L2 ethernet switch. The Port 0 is responsible for communication with the host
CPU, while Port 1 is responsible for receiving/transmiting packets from/to
external PHY. The CPSW module depends on the K3 DMA archicture for bulk of data
movement.

To use this driver, please add the following component as belows:

vxprj component add DRV_END_FDT_TI_K3CPSW

However this driver doesn't support CPSW operating in switch mode.
Instead this driver configures the CPSW to operate in independent EMAC mode
by disabling address learning on the external GMAC ports and constructing
the MAC address look-up table manually during intialization.

There are 2 VXBUS instances for the driver: one for the host port whose control
block is of type K3CPSW_SW_CTRL, one for the GMAC port whose control block is of
type K3CPSW_DRV_CTRL. The driver is designed in a way by which the external
ports are extended to more than 1 easily: just add a subnode DTS node.

This driver supports promisc mode by setting ALE (Address Lookup Engine)
into bypass mode. In bypass mode, all packets received will be forwarded
only to host port and ingress checking is ignored. if there are more than one
ethernet ports inside the CPSW,  if one port is put into promisc mode then 
the other will also be put into promisc mode automatically, but interface 
capability flags don't change automactically as because ALE is a shared resouce.
For example, calling:
\cs
ifconfig "cpsw0 inet promisc"
\ce
will put both cpsw0 and cpsw1 into promisc mode, but only cpsw0's
capability flags will show:

\cs
UP RUNNING SIMPLEX BROADCAST MULTICAST PROMISC ALLMULTI
\ce

cpsw1's capability flags will not be changed, the following command
must be called for cpsw1 to show up correctly:

\cs
ifconfig "cpsw1 inet promisc"
\ce

This same is true when removing promisc mode.

BOARD LAYOUT
The network interfaces are internal to the CPU. All configurations
are jumperless. See target.ref for connector locations.

DEVICE TREE BINDING
This driver is bound to device tree. An example of the device tree node is as 
following:

\cs
    mcu_cpsw: ethernet@46000000
        {
        compatible  = "ti,am65x-cpsw-nuss";
        reg = <0x0 0x46000000 0x0 0x200000>;

        cpsw_port1: port@1
            {
            device_type    = "network";
            compatible     = "ti,k3-cpsw-port";
            #address-cells = <1>;
            #size-cells    = <0>;
            cpsw-port-index  = <1>;
            cpsw-gmac-offset = <0x22330>;
            cpsw-port-offset = <0x208>;
            local-mac-address = [ 00 18 31 e0 a8 32 ];
            phy-handle = <&phy0>;
            phy0: ethernet-phy@0
                {
                compatible = "tiDpPhy";
                reg = <0>;
                rgmii-delay = <0x3>;
                rx-internal-delay = <0x8>;
                tx-internal-delay = <0xa>;
                fifo-depth = <0x1>;
                };
            };
        };
\ce

\cs

This routine queries the device tree to provide the ethernet address
for a given MAC.

\ce

RESTRICTIONS

SEE ALSO: VxBus, miiBus, ifLib
*/

#include <vxWorks.h>
#include <string.h>
#include <intLib.h>
#include <stdio.h>
#include <string.h>
#include <netLib.h>
#include <netBufLib.h>
#include <semLib.h>
#include <sysLib.h>
#include <wdLib.h>
#include <vmLib.h>
#include <etherMultiLib.h>
#include <end.h>
#define END_MACROS
#include <endLib.h>
#include <endMedia.h>
#include <cacheLib.h>
#include <spinLockLib.h>
#include <taskLib.h>
#include <hwif/vxBus.h>
#include <hwif/buslib/vxbFdtLib.h>
#include <subsys/clk/vxbClkLib.h>
#include <subsys/pinmux/vxbPinMuxLib.h>
#include <subsys/int/vxbIntLib.h>
#include <subsys/timer/vxbTimerLib.h>
#include <miiBus.h>
#include <vxbEndUtil.h>
#include "vxbTiK3NavssRingacc.h"
#include "vxbTiK3NavssUdma.h"
#include "vxbFdtTiK3Am65xCpswEnd.h"

/* defines */

/* debug macro */

#undef CPSW_DEBUG
#ifdef CPSW_DEBUG
#include <private/kwriteLibP.h>         /* _func_kprintf */

#ifdef LOCAL
#undef LOCAL
#endif
#define LOCAL
#define DBG_OFF             0x00000000
#define DBG_WARN            0x00000001
#define DBG_ERR             0x00000002
#define DBG_INFO            0x00000004
#define DBG_ALL             0xffffffff
LOCAL UINT32 dbgMask = DBG_ALL;

#undef DBG_MSG

#define DBG_MSG(mask,...)                                      \
do                                                             \
{                                                              \
    if ((dbgMask & (mask)) || ((mask) == DBG_ALL))             \
        {                                                      \
        if (_func_kprintf != NULL)                             \
            {                                                  \
            (* _func_kprintf)("%s,%d, ",__FUNCTION__,__LINE__);\
            (* _func_kprintf)(__VA_ARGS__);                    \
            }                                                  \
        }                                                      \
}while (0)
#else
#define DBG_MSG(...)
#endif  /* CPSW_DEBUG */

/* typedef */

typedef struct CpswSlaveDevInfo {
	struct vxbFdtDev  vxbFdtDev;
    VXB_RESOURCE_LIST vxbResList;
} CPSW_PORT_DEV_INFO;

#ifdef CPSW_DEBUG
K3CPSW_SW_CTRL * pDbgSwCtrl = NULL;
#endif /* CPSW_DEBUG */

/* import functions */

IMPORT FUNCPTR      _func_m2PollStatsIfPoll;
IMPORT STATUS       sysNetMacNVRamAddrGet (char *, INT32, UINT8 *, INT32);

/* switch controller routines */

LOCAL STATUS        am65xCpswSwCtrlProbe     (VXB_DEV_ID);
LOCAL STATUS        am65xCpswSwCtrlAttach    (VXB_DEV_ID);
LOCAL VXB_FDT_DEV * am65xCpswPortDevInfo     (VXB_DEV_ID, VXB_DEV_ID);
LOCAL VXB_RESOURCE* am65xCpswPortResAlloc    (VXB_DEV_ID, VXB_DEV_ID, UINT32);
LOCAL STATUS        am65xCpswPortsCreate     (VXB_DEV_ID);

LOCAL STATUS        am65xCpswHostPortInit    (VXB_DEV_ID);
LOCAL STATUS        am65xCpswGmacPortInit    (K3CPSW_DRV_CTRL *);
LOCAL STATUS        am65xCpswSoftReset       (K3CPSW_SW_CTRL *, UINT32, UINT32);

/* ALE routines */

LOCAL void          am65xCpswAleRead         (VXB_DEV_ID, CPSW_ALE_TBL * , UINT32);
LOCAL void          am65xCpswAleWrite        (VXB_DEV_ID, CPSW_ALE_TBL * , UINT32);
LOCAL INT32         am65xCpswAleFind         (VXB_DEV_ID);
LOCAL INT32         am65xCpswAleMatch        (VXB_DEV_ID, CPSW_ALE_TBL *);
LOCAL INT32         am65xCpswAleMatchVlan    (VXB_DEV_ID, INT32);
LOCAL INT32         am65xCpswAleAddUniCast
                        (
                        VXB_DEV_ID,
                        UINT8 *,
                        INT32, 
                        INT32,
                        INT32,
                        INT32
                        );
LOCAL void          am65xCpswAleDelUniCast
                        (
                        VXB_DEV_ID,
                        UINT8 *,
                        INT32,
                        INT32,
                        INT32,
                        INT32
                        );
LOCAL INT32         am65xCpswAleAddVlan      
                        (
                        VXB_DEV_ID, 
                        INT32,
                        INT32,
                        INT32,
                        INT32,
                        INT32
                        );
LOCAL INT32         am65xCpswAleAddMultiCast 
                        (
                        VXB_DEV_ID,
                        UINT8 *,
                        INT32,
                        INT32, 
                        INT32,
                        INT32
                        );
LOCAL void          am65xCpswAleDelMultiCast
                        (
                        VXB_DEV_ID,
                        UINT8 *,
                        INT32,
                        INT32,
                        INT32,
                        INT32
                        );

#ifndef _WRS_CONFIG_CERT
#ifdef CPSW_DEBUG
LOCAL void          am65xCpswAleEntryShow    (CPSW_ALE_TBL *);
LOCAL void          am65xCpswAleDump         (VXB_DEV_ID);
#endif /* CPSW_DEBUG */
#endif /* !_WRS_CONFIG_CERT */

/* END functions */

LOCAL void          am65xCpswRcvPollTask     (K3CPSW_DRV_CTRL * pDrvCtrl);
LOCAL END_OBJ *     am65xCpswEndLoad         (char *, void *);
LOCAL STATUS        am65xCpswEndUnload       (END_OBJ *);
LOCAL INT32         am65xCpswEndIoctl        (END_OBJ *, INT32, caddr_t);
LOCAL STATUS        am65xCpswEndMCastAddrAdd (END_OBJ *, char *);
LOCAL STATUS        am65xCpswEndMCastAddrDel (END_OBJ *, char *);
LOCAL STATUS        am65xCpswEndMCastAddrGet (END_OBJ *, MULTI_TABLE *);
LOCAL STATUS        am65xCpswEndStart        (END_OBJ *);
LOCAL STATUS        am65xCpswEndStop         (END_OBJ *);
LOCAL INT32         am65xCpswEndEncap        (K3CPSW_DRV_CTRL *, M_BLK_ID);
LOCAL INT32         am65xCpswEndSend         (END_OBJ *, M_BLK_ID);
LOCAL STATUS        am65xCpswEndPollSend     (END_OBJ *, M_BLK_ID);
LOCAL INT32         am65xCpswEndPollReceive  (END_OBJ *, M_BLK_ID);
LOCAL void          am65xCpswEndRxHandle     (void *);
LOCAL void          am65xCpswEndTxHandle     (void *);
LOCAL void          am65xCpswEndMiscHandle   (void *);
STATUS              am65xCpswPhyRead         (VXB_DEV_ID, UINT8, UINT8, UINT16 *);
STATUS              am65xCpswPhyWrite        (VXB_DEV_ID, UINT8, UINT8, UINT16);
LOCAL STATUS        am65xCpswLinkUpdate      (VXB_DEV_ID);
LOCAL void          am65xCpswMuxConnect      (VXB_DEV_ID, void *);
LOCAL STATUS        am65xCpswPortProbe       (VXB_DEV_ID);
LOCAL STATUS        am65xCpswPortAttach      (VXB_DEV_ID);

LOCAL NET_FUNCS am65xCpswNetFuncs =
    {
    am65xCpswEndStart,             /* start func. */
    am65xCpswEndStop,              /* stop func. */
    am65xCpswEndUnload,            /* unload func. */
    am65xCpswEndIoctl,             /* ioctl func. */
    am65xCpswEndSend,              /* send func. */
    am65xCpswEndMCastAddrAdd,      /* multicast add func. */
    am65xCpswEndMCastAddrDel,      /* multicast delete func. */
    am65xCpswEndMCastAddrGet,      /* multicast get fun. */
    am65xCpswEndPollSend,          /* am65xCpswPolling send func. */
    am65xCpswEndPollReceive,       /* am65xCpswPolling receive func. */
    endEtherAddressForm,           /* put address info into a NET_BUFFER */
    endEtherPacketDataGet,         /* get pointer to data in NET_BUFFER */
    endEtherPacketAddrGet          /* Get packet addresses */
    };

/* driver utility functions */

LOCAL VXB_DRV_METHOD am65xCpswSwCtrlMethods[] = {
    { VXB_DEVMETHOD_CALL(vxbDevProbe),     (FUNCPTR)am65xCpswSwCtrlProbe },
    { VXB_DEVMETHOD_CALL(vxbDevAttach),    (FUNCPTR)am65xCpswSwCtrlAttach},
    { VXB_DEVMETHOD_CALL(vxbFdtDevGet),    (FUNCPTR)am65xCpswPortDevInfo },
    { VXB_DEVMETHOD_CALL(vxbResourceAlloc),(FUNCPTR)am65xCpswPortResAlloc},
    { 0, 0 }
};

/* driver utility functions */

LOCAL VXB_DRV_METHOD am65xCpswPortMethods[] = {
    { VXB_DEVMETHOD_CALL(vxbDevProbe),    (FUNCPTR)am65xCpswPortProbe    },
    { VXB_DEVMETHOD_CALL(vxbDevAttach),   (FUNCPTR)am65xCpswPortAttach   },
    { VXB_DEVMETHOD_CALL(miiRead),        (FUNCPTR)am65xCpswPhyRead      },
    { VXB_DEVMETHOD_CALL(miiWrite),       (FUNCPTR)am65xCpswPhyWrite     },
    { VXB_DEVMETHOD_CALL(miiMediaUpdate), (FUNCPTR)am65xCpswLinkUpdate   },
    { 0, 0 }
};

VXB_DRV  k3Am65xCpswSwCtrlDrv =
    {
    { NULL } ,
    "cpsw switch controller",   /* drvName */
    "cpsw switch controller",   /* Description */
    VXB_BUSID_FDT,              /* Class */
    0,                          /* Flags */
    0,                          /* Reference count */
    am65xCpswSwCtrlMethods           /* Method table */
    };

VXB_DRV  k3Am65xCpswPortDrv =
    {
    { NULL } ,
    "cpsw port",        /* drvName */
    "cpsw port",        /* Description */
    VXB_BUSID_FDT,      /* Class */
    0,                  /* Flags */
    0,                  /* Reference count */
    am65xCpswPortMethods     /* Method table */
    };

VXB_DRV_DEF(k3Am65xCpswSwCtrlDrv)
VXB_DRV_DEF(k3Am65xCpswPortDrv)

LOCAL VXB_FDT_DEV_MATCH_ENTRY am65xCpswSwCtrlMatch[] =
    {
        {
        "ti,am65x-cpsw-nuss",  /* compatible */
        NULL,
        },
        {} /* Empty terminated list */
    };

LOCAL VXB_FDT_DEV_MATCH_ENTRY am65xCpswPortMatch[] =
    {
        {
        "ti,k3-cpsw-port",  /* compatible */
        NULL,
        },
        {} /* Empty terminated list */
    };

/*********************************************************************
*
* am65xCpswSwCtrlProbe - check whether device and driver go together
*
* This routine probe a device with a device driver.
*
* RETURNS: OK if the device and driver match up, ERROR otherwise
*
* ERROR: N/A
*/

LOCAL STATUS am65xCpswSwCtrlProbe
    (
    VXB_DEV_ID pDev
    )
    {
    return vxbFdtDevMatch (pDev, am65xCpswSwCtrlMatch, NULL);
    }

/*********************************************************************
*
* am65xCpswSwCtrlAttach - Initilize the cpsw switch controller device.
*
* This routine initilize the cpsw switch controller device.
*
* RETURNS: OK or ERROR if failed.
*
* ERROR: N/A
*/

LOCAL STATUS am65xCpswSwCtrlAttach
    (
    VXB_DEV_ID pDev
    )
    {
    VXB_RESOURCE_ADR * pResAdr = NULL;
    VXB_RESOURCE     * pRes;
    K3CPSW_SW_CTRL   * pSwCtrl;

    pSwCtrl = vxbMemAlloc (sizeof (K3CPSW_SW_CTRL));
    if (pSwCtrl == NULL)
        {
        DBG_MSG (DBG_ERR,"vxbMemAlloc failed\n");
        return ERROR;
        }

    /* create connect between pSwCtrl and it's pDev */

    pSwCtrl->pDev = pDev;
    vxbDevSoftcSet(pDev, pSwCtrl);

    /* init register offsets */

    pRes = vxbResourceAlloc(pDev, VXB_RES_MEMORY, 0);
    if(pRes == NULL)
        {
        DBG_MSG (DBG_ERR,"pRes is NULL\n");
        goto failed;
        }

    pResAdr = (VXB_RESOURCE_ADR *)pRes->pRes;
    if (pResAdr == NULL)
        {
        DBG_MSG (DBG_ERR,"pResAdr is NULL\n");
        goto failed;
        }

    pSwCtrl->regBase = (void*)pResAdr->virtual;
    pSwCtrl->handle  = pResAdr->pHandle;

    pSwCtrl->cpswOffset  = MCU_CPSW0_CONTROL_BASE;
    pSwCtrl->portOffset  = MCU_CPSW0_P0_BASE;
    pSwCtrl->statsOffset = MCU_CPSW0_STAT0_BASE;
    pSwCtrl->aleOffset   = MCU_CPSW0_ALE_BASE;
    pSwCtrl->mdioOffset  = MCU_CPSW0_MDIO_BASE;

    /*
     * set work mode to independent port mode,
     * in fact we only support this mode
     */

    pSwCtrl->workMode = CPSW_MODE_INDEPENDENT_PORT;

    /* set host port(which is connect to cpu) index to 0 */

    pSwCtrl->hostPortIndex = 0;

    pSwCtrl->cpswDevSem = semMCreate (SEM_Q_PRIORITY  |
                                      SEM_DELETE_SAFE |
                                      SEM_INVERSION_SAFE);
    if (pSwCtrl->cpswDevSem == NULL)
        {
        DBG_MSG (DBG_ERR,"Can not create semaphore\n");

        goto failed;
        }

    /* enable control registers & pinmux */

    (void) vxbPinMuxEnable (pDev);
    (void) vxbClkEnableAll (pDev);

    /* initialize host(cpu) port */

    if (am65xCpswHostPortInit (pDev) != OK)
        {
        DBG_MSG (DBG_ERR,"am65xCpswHostPortInit() return error\n");
        goto failed;
        }

    /* here we call slave port attach function. */

    if (am65xCpswPortsCreate (pDev) == ERROR)
        {
        DBG_MSG (DBG_ERR,"am65xCpswPortsCreate() return error\n");
        goto failed;
        }

#ifdef CPSW_DEBUG
    pDbgSwCtrl = pSwCtrl;
#endif /* CPSW_DEBUG */

    tiK3NavssUdmaInit ();

    return OK;

failed:

    if (pSwCtrl != NULL)
        {
        if (pSwCtrl->cpswDevSem)
            (void) semDelete (pSwCtrl->cpswDevSem);

        vxbMemFree (pSwCtrl);
        pSwCtrl = NULL;
        }

    vxbDevSoftcSet (pDev, NULL);

    return ERROR;
    }

/*******************************************************************************
*
* am65xCpswPortDevInfo - get the FDT child device information
*
* This routine gets the FDT child device information
*
* \NOMANUAL
*
* RETURNS: the device information pointer
*
* ERRNO: N/A
*/

LOCAL VXB_FDT_DEV *  am65xCpswPortDevInfo
    (
    VXB_DEV_ID pDev,
    VXB_DEV_ID pChild
    )
    {
    CPSW_PORT_DEV_INFO * pSlDevInfo;

    if (pChild == NULL)
        return NULL;

    pSlDevInfo = vxbDevIvarsGet(pChild);

    if (pSlDevInfo == NULL)
        return NULL;

    return &pSlDevInfo->vxbFdtDev;
    }


/*******************************************************************************
*
* am65xCpswPortResAlloc - vxbus alloc interface
*
* This routine will be used by child of this VXB_DEV to find the resource value.
*
* RETURNS: point of resource when success, NULL for others.
*
* ERRNO: N/A
*/

LOCAL VXB_RESOURCE * am65xCpswPortResAlloc
    (
    VXB_DEV_ID pDev,
    VXB_DEV_ID pChild,
    UINT32     id
    )
    {
    CPSW_PORT_DEV_INFO * pDevInfo;
    VXB_RESOURCE       * pVxbRes;

    pDevInfo = (CPSW_PORT_DEV_INFO *)vxbDevIvarsGet(pChild);

    if (pDevInfo == NULL)
        return NULL;

    pVxbRes = vxbResourceFind(&pDevInfo->vxbResList, id);

    if (pVxbRes == NULL)
        return NULL;

    if (((VXB_RES_TYPE(pVxbRes->id) == VXB_RES_MEMORY) ||
         (VXB_RES_TYPE(pVxbRes->id) == VXB_RES_IO)) &&
        (vxbRegMap (pVxbRes) == OK))
        {
        return pVxbRes;
        }
    else if ((VXB_RES_TYPE(pVxbRes->id) == VXB_RES_IRQ) &&
        (vxbIntMap (pVxbRes) == OK))
        {
        return pVxbRes;
        }
    else
        {
        return NULL;
        }
    }

/*******************************************************************************
*
* am65xCpswPortsCreate - create the port device
*
* This function implements the VxBus gen2 bus attach routine for CPSW port
* device instance.
*
* RETURNS: OK or ERROR.
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswPortsCreate
    (
    VXB_DEV_ID  pDev
    )
    {
    VXB_FDT_DEV        * pFdtDev;
    CPSW_PORT_DEV_INFO * pPortDevInfo;
    VXB_FDT_DEV        * pNewFdtDev;
    VXB_DEV_ID           pCur = NULL;
    INT32                offset;

    pFdtDev = vxbFdtDevGet(pDev);

    if (pFdtDev == NULL)
        {
        DBG_MSG (DBG_ERR, "pFdtDev is NULL\n");
        return ERROR;
        }
    offset = pFdtDev->offset;

    for (offset = VX_FDT_CHILD(offset); offset > 0;
         offset = VX_FDT_PEER(offset))
        {
        pCur = NULL;

        if (vxFdtIsEnabled(offset) == FALSE)
            continue;

        if (vxbDevCreate (&pCur) != OK)
            continue;

        pPortDevInfo = (CPSW_PORT_DEV_INFO *)
                        vxbMemAlloc(sizeof(*pPortDevInfo));
        if (pPortDevInfo == NULL)
            {
            (void)vxbDevDestroy(pCur);
            continue;
            }

        pNewFdtDev = &pPortDevInfo->vxbFdtDev;

        pNewFdtDev->offset = offset;

        /* get the device basic infomation  */

        vxbFdtDevSetup(offset, pNewFdtDev);
        vxbDevNameSet(pCur, pNewFdtDev->name, FALSE);

        /* assign the bus internal variable and type  */

        vxbDevIvarsSet(pCur, (void *)pPortDevInfo);
        vxbDevClassSet(pCur, VXB_BUSID_FDT);

        /* get the device register and interrupt infomation  */

        if (vxbResourceInit(&pPortDevInfo->vxbResList) != OK)
            {
            (void)vxbDevDestroy(pCur);
            vxbMemFree (pPortDevInfo);
            continue;
            }

        /* fetched out reg space info from DTS for this devide */

        if (vxbFdtRegGet(&pPortDevInfo->vxbResList, pNewFdtDev) != OK)
            {
            vxbMemFree(pPortDevInfo);
            (void)vxbDevDestroy(pCur);
            continue;
            }

        /* fetched out interrupt info from DTS for this devide */

        if (vxbFdtIntGet(&pPortDevInfo->vxbResList, pNewFdtDev) != OK)
            {
            vxbMemFree(pPortDevInfo);
            (void)vxbDevDestroy(pCur);
            continue;
            }

        (void)vxbDevAdd(pDev, pCur);
        }

    return OK;
    }

/*******************************************************************************
*
* am65xCpswHostPortInit - initialize CPSW host port
*
* This routine initializes the CPSW host port. It performs a soft reset, clears &
* enables the ALE module (NOTE: ALE must be enabled otherwise all packets will be
* dropped), and puts port in forward state. MDIO module is enabled in this routine
* (NOTE: if soft reset fails, the routine won't proceed. An error message will be
* print out if debug mode is enabled)
*
* RETURNS: OK or ERROR if reset timeout
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswHostPortInit
    (
    VXB_DEV_ID pDev
    )
    {
    K3CPSW_SW_CTRL * pSwCtrl = vxbDevSoftcGet(pDev);
    UINT32           base;
    UINT32           val;

    base = pSwCtrl->aleOffset;

    val = CPSW_ALE_EN_TABLE | CPSW_ALE_CLR_TABLE;

    /* let ALE enter vlan mode */

    if (pSwCtrl->workMode == CPSW_MODE_INDEPENDENT_PORT)
        {
        val |= CPSW_ALE_VLAN_AWARE;
        }

    CSR_WRITE_4 (pSwCtrl, base + CPSW_ALE_CONTROL, val);

    CSR_WRITE_4 (pSwCtrl, base + (UINT32)CPSW_ALE_PORTCTL(pSwCtrl->hostPortIndex),
                 CPSW_ALE_PORT_FW | CPSW_ALE_CTL_NO_LEARN);

    base = pSwCtrl->portOffset;
    CSR_WRITE_4 (pSwCtrl, base + CPSW_TX_PRI_MAP, CPSW_TX_PRI_MAP_DFTL);
    CSR_WRITE_4 (pSwCtrl, base + CPSW_FLOW_ID, 0);

    /* enable statistics for all ports */

    CSR_WRITE_4 (pSwCtrl, pSwCtrl->cpswOffset + CPSW_STAT_PORT_EN, 3);

    CSR_WRITE_4 (pSwCtrl, pSwCtrl->cpswOffset + CPSW_CONTROL, 
                 CPSW_CTL_P0_ENABLE | CPSW_CTL_REG_P0_TX_CRC_RM | 
                 CPSW_CTL_REG_P0_RX_PADBIT);

    CSR_WRITE_4 (pSwCtrl, pSwCtrl->cpswOffset + CPSW_PTYPE_REG, 0);

    base = pSwCtrl->mdioOffset;
    CSR_WRITE_4 (pSwCtrl, base + CPSW_MDIO_CONTROL,
                 CPSW_MDIO_EN | CPSW_MDIO_CLK_DIV);

    return OK;
    }

/*******************************************************************************
*
* am65xCpswSoftReset - perform soft reset on a given module
*
* This routine performs soft reset on a given module.
*
* RETURNS: OK or ERROR if timeout
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswSoftReset
    (
    K3CPSW_SW_CTRL * pSwCtrl,
    UINT32 reg,
    UINT32 timeout
    )
    {
    UINT32 i = 0;
    UINT32 ret;

    CSR_WRITE_4 (pSwCtrl, reg, 0x1);
    do
        {
        ret = CSR_READ_4 (pSwCtrl, reg);
        } while ((ret & 0x1) && (i++ < timeout));

    if (i >= timeout)
        return ERROR;

    return OK;
    }

/*******************************************************************************
*
* am65xCpswAleRead - read an ALE entry
*
* This routine reads an ALE entry from the given index
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am65xCpswAleRead
    (
    VXB_DEV_ID     pDev,
    CPSW_ALE_TBL * pTbl,
    UINT32         entry
    )
    {
    K3CPSW_SW_CTRL * pSwCtrl = vxbDevSoftcGet(pDev);

    CSR_WRITE_4 (pSwCtrl, pSwCtrl->aleOffset + CPSW_ALE_TBLCTL,
                 entry & CPSW_ALE_ENTRY_IDX_MASK);

    pTbl->word0 = CSR_READ_4 (pSwCtrl, pSwCtrl->aleOffset + CPSW_ALE_WORD0);
    pTbl->word1 = CSR_READ_4 (pSwCtrl, pSwCtrl->aleOffset + CPSW_ALE_WORD1);
    pTbl->word2 = (UINT8)(CSR_READ_4 (pSwCtrl, pSwCtrl->aleOffset + CPSW_ALE_WORD2)
                         & 0x1f);
    }

/*******************************************************************************
*
* am65xCpswAleWrite - write an ALE entry
*
* This routine writes data to an ALE entry with the given index
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am65xCpswAleWrite
    (
    VXB_DEV_ID     pDev,
    CPSW_ALE_TBL * pTbl,
    UINT32         entry
    )
    {
    K3CPSW_SW_CTRL * pSwCtrl = vxbDevSoftcGet(pDev);

    CSR_WRITE_4 (pSwCtrl, pSwCtrl->aleOffset + CPSW_ALE_WORD0, pTbl->word0);
    CSR_WRITE_4 (pSwCtrl, pSwCtrl->aleOffset + CPSW_ALE_WORD1, pTbl->word1);
    CSR_WRITE_4 (pSwCtrl, pSwCtrl->aleOffset + CPSW_ALE_WORD2, pTbl->word2);

    CSR_WRITE_4 (pSwCtrl, pSwCtrl->aleOffset + CPSW_ALE_TBLCTL,
                 (entry & CPSW_ALE_ENTRY_IDX_MASK) | CPSW_ALE_WRITE);
    }

/*******************************************************************************
*
* am65xCpswAleFind - find an emtpy ALE entry
*
* This routine finds an emtpy ALE entry.
*
* RETURNS: empty entry index or -1 if ALE is full
*
* ERRNO: N/A
*/

LOCAL INT32 am65xCpswAleFind
    (
    VXB_DEV_ID pDev
    )
    {
    int          i;
    CPSW_ALE_TBL tbl;

    for (i = 0; i < CPSW_ALE_ENTRY_NR; i++)
        {
        am65xCpswAleRead (pDev, &tbl, (UINT32)i);
        if ((tbl.word1 & CPSW_ALE_ENTRY_MASK) == 0x0)
            return i;
        }

    return -1;
    }

/*******************************************************************************
*
* am65xCpswAleMatch - find the entry with the given MAC address
*
* This routine finds the entry with the given MAC address
*
* RETURNS: entry found or -1 if the MAC address has not entered into ALE yet.
*
* ERRNO: N/A
*/

LOCAL INT32 am65xCpswAleMatch
    (
    VXB_DEV_ID      pDev,
    CPSW_ALE_TBL *  pTbl
    )
    {
    int          i;
    CPSW_ALE_TBL t;

    for (i = 0; i < CPSW_ALE_ENTRY_NR; i++)
        {
        am65xCpswAleRead (pDev, &t, (UINT32)i);
        if ((pTbl->word0 == t.word0) && (pTbl->word1 == t.word1) &&
            (pTbl->word2 == t.word2))
            return i;
        }
    return -1;
    }

#ifndef _WRS_CONFIG_CERT
#ifdef CPSW_DEBUG
/*******************************************************************************
*
* am65xCpswAleEntryShow - show an ALE entry
*
* This routine outputs verbose message of an ALE entry
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am65xCpswAleEntryShow
    (
    CPSW_ALE_TBL * tbl
    )
    {
    UINT8 * mac;
    UINT8 * macaddr1;
    INT32   block = 0, secure = 0, port = 0, portmask = 0;
    INT32   fwstate = 0, super = 0;
    INT32   unicasttype = 0;
    INT32   vmem = 0, umf = 0, rmf = 0, fue = 0;

    char  * str = NULL;

    static char * unicast [] =
        {
        "u/na",
        "u/a/nt",
        "oui",
        "u/a/t"
        };

    mac = (UINT8 *)&tbl->word0;
    macaddr1 = (UINT8 *)&tbl->word1;
    INT32 vid = (tbl->word1 >> 16) & 0xfff;
    INT32 type = (tbl->word1 >> 28) & 0x3;
    INT32 multicast = tbl->word1 & 0x100;

    if (type == 0x0)
        return;

    if (type == 1)
        {
        if (multicast)
            str = "multicast";
        else
            str = "unicast";
        }
    else if (type == 0x3)
        {
        if (multicast)
            str = "multi/vlan";
        else
            str = "uni/vlan";
        }

    if (type == 0x2)
        {
        str = "vlan";
        fue = (tbl->word0 >> 24) & 0x7;
        rmf = (tbl->word0 >> 16) & 0x7;
        umf = (tbl->word0 >> 8) & 0x7;
        vmem = tbl->word0  & 0x7;
        }
    else
        {
        if (!multicast)
            {
            unicasttype = (tbl->word1 >> 30) & 0x3;
            secure = (tbl->word2 & 0x1);
            block = (tbl->word2 & 0x2) >> 1;
            port = (tbl->word2 >> 0x2) & 0x3;
            }
        else
            {
            fwstate = (tbl->word1 >> 30) & 0x3;
            super = (tbl->word2 >> 1) & 0x1;
            portmask = (tbl->word2 >> 2) & 0x7;
            }
        }

    if (type == 0x3 || type == 0x1)
        {

        /* multicast entry */

        if (multicast)
            {
            (void)printf ("%-20s%-20s%-20s%-20s%-20s%-20s\n", "mac",
                          "ent type", "state", "super", "portmask", "vid");

            (void)printf ("---------------------------------------------"
                          "---------------------------------------------"
                          "-----------------------------------\n");

            (void)printf ("%02x:%02x:%02x:%02x:%02x:%02x   %-20s%"
                          "-20d%-20d%-20d%-20d\n\n", macaddr1[1], macaddr1[0],
                          mac[3], mac[2], mac[1],mac[0],
                          str, fwstate, super, portmask, vid);
            }

        /* unicast entry */

        else
            {
            (void)printf ("%-20s%-20s%-20s%-20s%-20s%-20s%-20s\n",
                          "mac", "ent type", "type", "secure",
                          "block", "port", "vid");

            (void)printf ("---------------------------------------------"
                          "---------------------------------------------"
                          "-----------------------------------\n");

            (void)printf ("%02x:%02x:%02x:%02x:%02x:%02x   %-20s%"
                          "-20s%-20d%-20d%-20d%-20d\n\n", macaddr1[1],
                          macaddr1[0], mac[3], mac[2], mac[1],
                          mac[0], str, unicast[unicasttype], secure,
                          block, port, vid);
            }

        /* vlan entry */

        }
    else if (type == 0x2)
        {
        (void)printf ("%-20s%-20s%-20s%-20s%-20s%-20s%-20s\n", "mac",
                      "ent type", "fue", "rmf", "umf", "vmem", "vid");
        (void)printf ("---------------------------------------------"
                      "---------------------------------------------"
                      "-----------------------------------\n");

        (void)printf ("%-20s%-20s%-20d%-20d%-20d%-20d%-20d\n\n", "N/A",
                      str, fue, rmf, umf, vmem, vid);
        }
    }

/*******************************************************************************
*
* am65xCpswAleDump - show all the ALE entries
*
* This routine outputs verbose message of all the ALE entries
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am65xCpswAleDump
    (
    VXB_DEV_ID pInst
    )
    {
    INT32 i;
    CPSW_ALE_TBL tbl;

    if (pInst)
        {
        for (i = 0; i < CPSW_ALE_ENTRY_NR; i++)
            {
            am65xCpswAleRead (pInst, &tbl, (UINT32)i);
            am65xCpswAleEntryShow (&tbl);
            }
        }
    }
#endif /* CPSW_DEBUG */
#endif /* !_WRS_CONFIG_CERT */

/*******************************************************************************
*
* am65xCpswAleAddMultiCast - add a multicast MAC address into ALE table
*
* This routine adds a multicast MAC address into ALE table. First, this routine
* will try to find a match. If the MAC address is already in the table, it
* just returns. If it cannot find a match entry, an empty entry will be 
* allocated and the MAC address will be added. 
*
* RETURNS: 0 on success, ENOMEM if can not find a emtpy entry
*
* ERRNO: N/A
*/

LOCAL INT32 am65xCpswAleAddMultiCast
    (
    VXB_DEV_ID   pDev,
    UINT8      * pMac,
    INT32        vid,
    INT32        fwstate,
    INT32        super,
    INT32        portmask
    )
    {
    CPSW_ALE_TBL    tbl;
    INT32           entry;

    tbl.word0 = (UINT32)((pMac[2] << 24) | (pMac[3] << 16) | (pMac[4] << 8) | (pMac[5]));
    tbl.word1 = (UINT32)((pMac[0] << 8) | pMac[1]);
    if (vid > 0)
        {
        /* multicast with vlan */

        tbl.word1 |= (UINT32)((((vid & 0xfff) << 16) | ((fwstate & 0x3) << 30) | 
                      (0x3 << 28)));
        }
    else
        {
        tbl.word1 |= (UINT32)((((fwstate & 0x3) << 30) | (0x1 << 28)));
        }

    tbl.word2 = (UINT8) (((super & 0x1) << 1) | ((portmask & 0x7) << 2));

    entry = am65xCpswAleMatch (pDev, &tbl);

    /* there is already one identical entry, don't re-write it */

    if (entry >= 0)
        return 0;

    entry = am65xCpswAleFind (pDev);
    if (entry < 0)
        {
        return ENOMEM;
        }

    am65xCpswAleWrite (pDev, &tbl, (UINT32)entry);

    return 0;
    }

/*******************************************************************************
*
* am65xCpswAleDelMultiCast - delete a multicast MAC address from ALE table
*
* This routine deletes a multicast MAC address from ALE table. First, it
* will try to find a match entry. If the MAC address is already in the table, it
* sets it as a free entry.
*
* RETURNS: NA
*
* ERRNO: N/A
*/

LOCAL void am65xCpswAleDelMultiCast
    (
    VXB_DEV_ID   pDev,
    UINT8      * pMac,
    INT32        vid,
    INT32        fwstate,
    INT32        super,
    INT32        portmask
    )
    {
    CPSW_ALE_TBL    tbl;
    INT32           entry;

    tbl.word0 = (UINT32)((pMac[2] << 24) | (pMac[3] << 16) | (pMac[4] << 8) | (pMac[5]));
    tbl.word1 = (UINT32)((pMac[0] << 8) | pMac[1]);
    if (vid > 0)
        {
        /* multicast with vlan */

        tbl.word1 |= (UINT32)(((vid & 0xfff) << 16) | ((fwstate & 0x3) << 30) | 
                      (0x3 << 28));
        }
    else
        {
        tbl.word1 |= (UINT32)(((fwstate & 0x3) << 30) | (0x1 << 28));
        }

    tbl.word2 = (UINT8) (((super & 0x1) << 1) | ((portmask & 0x7) << 2));

    entry = am65xCpswAleMatch (pDev, &tbl);

    /* there is already one identical entry, don't re-write it */

    if (entry < 0)
        return;

    tbl.word0 = 0;
    tbl.word1 = 0;
    tbl.word2 = 0;

    am65xCpswAleWrite (pDev, &tbl, (UINT32)entry);
    }


/*******************************************************************************
*
* am65xCpswAleAddUniCast - add a unicast MAC address into ALE table
*
* This routine adds a unicast MAC address into ALE table. First, this routine
* will try to find a match. If the MAC address is already in the table, it overwrites
* that entry content with new values. If it can not find a match,
* a empty entry will be allocated and the MAC address will be added.
*
* RETURNS: 0 on success or ENOMEM if can not find a emtpy entry
*
* ERRNO: N/A
*/

LOCAL INT32 am65xCpswAleAddUniCast
    (
    VXB_DEV_ID   pDev,
    UINT8      * pMac,
    INT32        vid,
    INT32        secure,
    INT32        block,
    INT32        port
    )
    {
    CPSW_ALE_TBL tbl;
    INT32        entry;

    tbl.word0 = (UINT32)((pMac[2] << 24) | (pMac[3] << 16) | (pMac[4] << 8) | (pMac[5]));
    tbl.word1 = (UINT32)((pMac[0] << 8) | (pMac[1]));
    if (vid > 0)
        {
        tbl.word1 |= (UINT32)((0x3 << 28) | CPSW_ALE_UNICAST_AGEABLE_NOT | 
                      ((vid & 0xfff) << 16));
        }
    else
        {
        tbl.word1 |= ((0x1 << 28) | CPSW_ALE_UNICAST_AGEABLE_NOT);
        }

    tbl.word2 = (UINT8) (((block & 0x1) << 1) | (secure & 0x1) |
                        ((port & 0x3) << 2));

    entry = am65xCpswAleMatch (pDev, &tbl);

    /* it is already in the table, just return */

    if (entry >= 0)
        return 0;

    entry = am65xCpswAleFind (pDev);

    if (entry < 0)
        {
        return ENOMEM;
        }

    am65xCpswAleWrite (pDev, &tbl, (UINT32)entry);

    return 0;
    }

/*******************************************************************************
*
* am65xCpswAleDelUniCast - delete the ALE entry with the given MAC address and vid
*
* This routine finds the ALE entry index with the given MAC address, VLAN
* ID, and other flags. Then delete it from the ALE table.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am65xCpswAleDelUniCast
    (
    VXB_DEV_ID   pDev,
    UINT8      * pMac,
    INT32        vid,
    INT32        secure,
    INT32        block,
    INT32        port
    )
    {
    CPSW_ALE_TBL tbl;
    INT32        entry;

    tbl.word0 = (UINT32)((pMac[2] << 24) | (pMac[3] << 16) | (pMac[4] << 8) | (pMac[5]));
    tbl.word1 = (UINT32)((pMac[0] << 8) | (pMac[1]));
    if (vid > 0)
        {
        tbl.word1 |= (UINT32)(((0x3 << 28) | CPSW_ALE_UNICAST_AGEABLE_NOT | 
                      ((vid & 0xfff) << 16)));
        }
    else
        {
        tbl.word1 |= ((0x1 << 28) | CPSW_ALE_UNICAST_AGEABLE_NOT);
        }

    tbl.word2 = (UINT8) (((block & 0x1) << 1) | (secure & 0x1) |
                        ((port & 0x3) << 2));

    entry = am65xCpswAleMatch (pDev, &tbl);

    /* it is already in the table, just return */

    if (entry < 0)
        return;

    tbl.word0 = 0;
    tbl.word1 = 0;
    tbl.word2 = 0;

    am65xCpswAleWrite (pDev, &tbl, (UINT32)entry);

    return;
    }

/*******************************************************************************
*
* am65xCpswAleMatchVlan - find the ALE entry with the given vid
*
* This routine finds the ALE entry index with the given vid
*
* RETURNS: ALE entry index on success or -1 if entry not found
*
* ERRNO: N/A
*/

LOCAL INT32 am65xCpswAleMatchVlan
    (
    VXB_DEV_ID pDev,
    INT32 vid
    )
    {
    INT32        i;
    CPSW_ALE_TBL t;

    for (i = 0; i < CPSW_ALE_ENTRY_NR; i++)
        {
        am65xCpswAleRead (pDev, &t, (UINT32)i);
        if (((t.word1 >> 28) & 0x3) == 0x2)
            {
            if (((t.word1 >> 16 ) & 0xfff)  == vid)
                return i;
            }
        }

    return -1;
    }

/*******************************************************************************
*
* am65xCpswAleAddVlan - add a vlan entry into ALE table
*
* This routine adds a vlan entry into ALE table
*
* RETURNS: 0 on success,  ENOMEM if can not find a emtpy entry or
*          EEXIST if the given vlan is alreay in ALE table
*
* ERRNO: N/A
*/

LOCAL INT32 am65xCpswAleAddVlan
    (
    VXB_DEV_ID pDev,
    INT32      vlanMemberList,
    INT32      unregFloodMask,
    INT32      regFloodMask,
    INT32      forceUntaggedEgress,
    INT32      vid
    )
    {
    CPSW_ALE_TBL t;
    INT32 entry;

    if (vid <= 0 || vid >= 4095)
        return EINVAL;

    entry = am65xCpswAleMatchVlan (pDev, vid);
    if (entry < 0)
        {
        entry = am65xCpswAleFind (pDev);
        if (entry < 0)
            return ENOMEM;
        }

    t.word0 = (UINT32)(((vlanMemberList & 0x7))      |
              ((unregFloodMask & 0x7) << 8) |
              ((regFloodMask & 0x7) << 16)  |
              ((forceUntaggedEgress & 0x7) << 24));
    t.word1 = (UINT32)(((vid & 0xfff) << 16) | (0x2 << 28));
    t.word2 = 0;
    am65xCpswAleWrite (pDev, &t, (UINT32)entry);

    return 0;
    }


/*******************************************************************************
*
* am65xCpswSlSetMiiDev - mii device setting function
*
* This function sets the mii device  for an end device.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswSlSetMiiDev
   (
   VXB_DEV_ID pDev,
   VXB_DEV_ID miiDev,
   INT32      phyAddr
   )
   {
   K3CPSW_DRV_CTRL * pDrvCtrl = (K3CPSW_DRV_CTRL *)vxbDevSoftcGet(pDev);
   MII_DRV_CTRL  * pMiiDrvCtrl;

   pDrvCtrl->cpswMiiDev = miiDev;

   pMiiDrvCtrl = (MII_DRV_CTRL *)vxbDevSoftcGet(miiDev);
   pMiiDrvCtrl->pEndInst = pDev;

   miiBusDevInstConnect(pDev);

   return OK;
   }

/*******************************************************************************
*
* am65xCpswPortProbe - check whether device and driver go together
*
* This routine probe a device with a device driver.
*
* RETURNS: OK if the device and driver match up, ERROR otherwise
*
* ERRNO
*/

LOCAL STATUS am65xCpswPortProbe
    (
    VXB_DEV_ID pDev
    )
    {
    return vxbFdtDevMatch (pDev, am65xCpswPortMatch, NULL);
    }

/*******************************************************************************
*
* am65xCpswGmacPortInit - initialize CPSW GMAC port
*
* This routine initializes the CPSW GMAC port. It performs a soft reset, sets up
* neccessary reigsters, and finally get MAC address from BSP. (NOTE: if soft
* reset fails, the routine won't proceed. An error message will be print out if
* debug mode is enabled)
*
* RETURNS: OK or ERROR if reset timeout
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswGmacPortInit
    (
    K3CPSW_DRV_CTRL * pDrvCtrl
    )
    {
    UINT32           lo, hi;
    K3CPSW_SW_CTRL * pSwCtrl = pDrvCtrl->pSwCtrl;
    VXB_DEV_ID       pDev    = pSwCtrl->pDev;
    UINT32           base;

    pDrvCtrl->pCpswRcvChainTail = &pDrvCtrl->cpswRcvChain;

   /*
    * put MAC port into forward state and disable learning on this port,
    * because we are operating in dual emac mode, not switch mode, so
    * we add our MAC address manually.
    */

    base = pSwCtrl->aleOffset;
    CSR_WRITE_4 (pSwCtrl, base + (UINT32)CPSW_ALE_PORTCTL(pDrvCtrl->portIndex),
                 CPSW_ALE_PORT_FW | CPSW_ALE_CTL_NO_LEARN);

    base = pDrvCtrl->portOffset;

    /* reset the gmac module */

    if (am65xCpswSoftReset (pSwCtrl, base + CPSW_SL_MAC_SOFT_RESET,
        CPSW_TIMEOUT_VAL) != OK)
        {
        DBG_MSG (DBG_ERR,"gmac port soft reset timeout\n");
        return ERROR;
        }

    CSR_WRITE_4 (pSwCtrl, base + CPSW_TX_PRI_MAP, CPSW_TX_PRI_MAP_DFTL);

    /*
     +-------+-------+-------+-------+-------+-------+
     | byte5 | byte4 | byte3 | byte2 | byte1 | byte0 |
     +-------+-------+-------+-------+-------+-------+
     */

    lo = (UINT32)(pDrvCtrl->mac[5] << 8   | pDrvCtrl->mac[4]);
    hi = (UINT32)(pDrvCtrl->mac[3] << 24  | pDrvCtrl->mac[2] << 16 |
         pDrvCtrl->mac[1] << 8   | pDrvCtrl->mac[0]);

    CSR_WRITE_4 (pSwCtrl, base + CPSW_SL_SA_L0, lo);
    CSR_WRITE_4 (pSwCtrl, base + CPSW_SL_SA_HI, hi);
    CSR_WRITE_4 (pSwCtrl, base + CPSW_SL_RX_PRI_MAP, CPDMA_TX_PRI_MAP_DFTL);
    CSR_WRITE_4 (pSwCtrl, base + CPSW_SL_RX_MAXLEN, CPSW_MTU);
    CSR_WRITE_4 (pSwCtrl, base + CPSW_SL_MAC_CTL, CPSW_SL_MAC_CTL_DEF);

    if (pSwCtrl->workMode == CPSW_MODE_INDEPENDENT_PORT)
        {
        CSR_WRITE_4 (pSwCtrl, base + CPSW_PORT_VLAN,
                     pDrvCtrl->portVlan);

        (void)am65xCpswAleAddVlan(pDev, (0x1 << pSwCtrl->hostPortIndex) |
                                   (0x1 << pDrvCtrl->portIndex),
                             0, 0, 0, (INT32)pDrvCtrl->portVlan);

        (void)am65xCpswAleAddUniCast (pDev, pDrvCtrl->mac,
                                 (INT32)pDrvCtrl->portVlan,
                                 0, 0, pSwCtrl->hostPortIndex);
        }

    taskSpawn ("cpswRcvPollTask", 255, 0, 0x2000, (FUNCPTR)am65xCpswRcvPollTask, (_Vx_usr_arg_t)pDrvCtrl,
               2, 3, 4, 5, 6, 7, 8, 9, 10);

    return OK;
    }

/*******************************************************************************
*
* am65xCpswPortAttach - attach a vxbus device
*
* This is the am65xCpswSl initialization routine.
*
* RETURNS: OK, or ERROR if initialization failed.
*
* ERRNO: N/A
*/


LOCAL STATUS am65xCpswPortAttach
    (
    VXB_DEV_ID pDev
    )
    {
    K3CPSW_SW_CTRL   * pSwCtrl;
    K3CPSW_DRV_CTRL  * pDrvCtrl;
    VXB_FDT_DEV      * pFdtDev;
    UINT32           * prop;
    VXB_DEV_ID         pParentDev;
    INT32              i;
    INT32              proplen = 0;

    if (pDev == NULL)
        {
        DBG_MSG (DBG_ERR,"parameter error\n");
        return ERROR;
        }

    pFdtDev = vxbFdtDevGet(pDev);
    if (pFdtDev == NULL)
        {
        DBG_MSG (DBG_ERR,"no fdtDev info\n");
        return ERROR;
        }

    pParentDev = vxbDevParent (pDev);
    if (pParentDev == NULL)
        {
        DBG_MSG (DBG_ERR,"It's an orphan device\n");
        return ERROR;
        }

    pSwCtrl = (K3CPSW_SW_CTRL *)vxbDevSoftcGet (pParentDev);
    if (pSwCtrl == NULL)
        {
        DBG_MSG (DBG_ERR,"No driver control block for the parent device\n");
        return ERROR;
        }

    pDrvCtrl = vxbMemAlloc (sizeof (K3CPSW_DRV_CTRL));
    if (pDrvCtrl == NULL)
        {
        DBG_MSG (DBG_ERR,"not enough memory\n");
        goto portAttachFail;
        }

    /* create connect between pDrvCtrl and pDev */

    vxbDevSoftcSet(pDev, pDrvCtrl);
    pDrvCtrl->pDev = pDev;

    prop = (UINT32 *)vxFdtPropGet(pFdtDev->offset, "cpsw-port-index", &proplen);
    if (prop == NULL)
        {
        DBG_MSG (DBG_ERR,"No cpsw-port-index property found\n");
        goto portAttachFail;
        }

    i  = (INT32)vxFdt32ToCpu(*prop);

    /*
     * verify the portIndex , the host port(connect to CPU) is 0,
     * so the other port should start from 1 and not beyond NR_MAC_PORTS
     */

    if (i < 1 || i > NR_MAC_PORTS)
        {
        DBG_MSG (DBG_ERR,"wrong portIndex\n");
        goto portAttachFail;
        }
    pSwCtrl->port[i-1] = pDrvCtrl;
    pDrvCtrl->pSwCtrl  = pSwCtrl;

    /* get gmac resources */

    pDrvCtrl->portIndex     = (UINT32)i;

    pDrvCtrl->portOffset = pSwCtrl->portOffset + (UINT32)(0x1000 * pDrvCtrl->portIndex);

    DBG_MSG (DBG_INFO,"%s%d portOffset = 0x%08x\n",
             CPSW_PORT_NAME, pDrvCtrl->portIndex - 1, pDrvCtrl->portOffset);

    /* set the default value of macSaveWay*/

    pDrvCtrl->macSaveWay = 0;

    prop = (UINT32 *)vxFdtPropGet(pFdtDev->offset,
                                  "cpsw-mac-save-way", &proplen);
    if (prop != NULL)
        {
        DBG_MSG (DBG_ERR,"%s%d find cpsw-mac-save-way property\n",
                 CPSW_PORT_NAME, pDrvCtrl->portIndex - 1);
        pDrvCtrl->macSaveWay = vxFdt32ToCpu(*prop);
        }
    DBG_MSG (DBG_INFO,"%s%d macSaveWay = 0x%08x\n",
             CPSW_PORT_NAME, pDrvCtrl->portIndex - 1, pDrvCtrl->macSaveWay);

    /*
     * fetched out mac address with vxbEndMacAddrGet()
     */
    
    if (vxbEndMacAddrGet (pDev, pDrvCtrl->mac) != OK)
        {
        goto portAttachFail;
        }        

    /*
     * prepare vlan value which will be programmed,
     * use the portIndex as vlan ID
     */

    pDrvCtrl->portVlan = (UINT32)CPSW_PORT_VLAN_PROG(pDrvCtrl->portIndex);

    /* search the phy from DTS and probe/attach the phy to system */

    if (phyAttach (pDev) != OK)
        {
        /* did't find phy node from DTS or did't match */

        DBG_MSG (DBG_ERR,"%s%d phyAttach failed\n",
                 CPSW_PORT_NAME, pDrvCtrl->portIndex - 1);

        goto portAttachFail;
        }

    /*
     * search the phy instance from mii system and make relation
     * ship between ethernet port and phy.
     */

    if (miiFind (pDev, am65xCpswSlSetMiiDev) != OK)
        {
        DBG_MSG (DBG_ERR,"%s%d miiFind failed\n",
                 CPSW_PORT_NAME, pDrvCtrl->portIndex - 1);

        goto portAttachFail;
        }

    /* initialize mac port(s) */

    if (am65xCpswGmacPortInit (pDrvCtrl) != OK)
        {
        DBG_MSG (DBG_ERR,"%s%d am65xCpswGmacPortInit failed\n",
                 CPSW_PORT_NAME, pDrvCtrl->portIndex - 1);

        goto portAttachFail;
        }

    am65xCpswMuxConnect (pDev, NULL);

    return OK;

portAttachFail:

    if (pDrvCtrl)
        {
        vxbMemFree (pDrvCtrl);

        vxbDevSoftcSet(pDev, NULL);
        }

    return ERROR;
    }

/*******************************************************************************
*
* am65xCpswPhyRead - miiBus miiRead method
*
* This function implements an miiRead() method that allows PHYs
* on the miiBus to access our MII management registers.
*
* RETURNS: ERROR if invalid PHY addr, else OK
*
* ERRNO: N/A
*/

STATUS am65xCpswPhyRead
    (
    VXB_DEV_ID pPhyParent,
    UINT8      phyAddr,
    UINT8      regAddr,
    UINT16   * pDataVal
    )
    {
    UINT32           ret;
    VXB_DEV_ID       pDev;
    UINT32           index = 0;
    K3CPSW_SW_CTRL * pSwCtrl;
    K3CPSW_DRV_CTRL *pDrvCtrl;

    if (pPhyParent == NULL)
        {
        return ERROR;
        }

    pDrvCtrl = (K3CPSW_DRV_CTRL *)vxbDevSoftcGet (pPhyParent);

    if (pDrvCtrl == NULL)
        {
        return ERROR;
        }

    pSwCtrl = pDrvCtrl->pSwCtrl;

    if (pSwCtrl == NULL)
        {
        return ERROR;
        }

    pDev = pSwCtrl->pDev;

    if (phyAddr >= 32)
        {
        return ERROR;
        }

    do
        {
        ret = CSR_READ_4 (pSwCtrl, pSwCtrl->mdioOffset + CPSW_USERACCESSn(index));
        } while (ret & CPSW_MDIO_GO);

    ret = (UINT32)(phyAddr << CPSW_PHY_ADDR_SHIFT) |
          (UINT32)(regAddr << CPSW_REG_ADDR_SHIFT) |
          CPSW_MDIO_GO;


    CSR_WRITE_4 (pSwCtrl, pSwCtrl->mdioOffset + CPSW_USERACCESSn(index), ret);

    do
        {
        ret = CSR_READ_4 (pSwCtrl, pSwCtrl->mdioOffset + CPSW_USERACCESSn(index));
        } while (ret & CPSW_MDIO_GO);

    *pDataVal = ret & 0xffff;

    return OK;
    }

/*******************************************************************************
*
* am65xCpswPhyWrite - miiBus miiWrite method
*
* This function implements an miiWrite() method that allows PHYs
* on the miiBus to access our MII management registers. This routine
* works in much the same way as am65xCpswPhyRead(), using the shortcut
* PHY management registers to make it look like there's a single
* PHY at MII address 0.
*
* RETURNS: ERROR if invalid PHY addr, else OK
*
* ERRNO: N/A
*/

STATUS am65xCpswPhyWrite
    (
    VXB_DEV_ID pPhyParent,
    UINT8      phyAddr,
    UINT8      regAddr,
    UINT16     dataVal
    )
    {
    UINT32            ret;
    VXB_DEV_ID        pDev;
    UINT32            index = 0;
    K3CPSW_SW_CTRL  * pSwCtrl;
    K3CPSW_DRV_CTRL * pDrvCtrl;

    if (phyAddr >= 32)
        {
        return ERROR;
        }

    if (pPhyParent == NULL)
        {
        return ERROR;
        }

    pDrvCtrl = (K3CPSW_DRV_CTRL *)vxbDevSoftcGet (pPhyParent);

    if (pDrvCtrl == NULL)
        {
        return ERROR;
        }

    pSwCtrl = pDrvCtrl->pSwCtrl;

    if (pSwCtrl == NULL)
        {
        return ERROR;
        }

    pDev = pSwCtrl->pDev;

    do
        {
        ret = CSR_READ_4 (pSwCtrl, pSwCtrl->mdioOffset + CPSW_USERACCESSn(index));
        } while (ret & CPSW_MDIO_GO);

    ret = (UINT32)(phyAddr << CPSW_PHY_ADDR_SHIFT) |
          (UINT32)(regAddr << CPSW_REG_ADDR_SHIFT) |
          CPSW_MDIO_GO | CPSW_MDIO_WRITE | dataVal;

    CSR_WRITE_4 (pSwCtrl, pSwCtrl->mdioOffset + CPSW_USERACCESSn(index), ret);

    do
        {
        ret = CSR_READ_4 (pSwCtrl, pSwCtrl->mdioOffset + CPSW_USERACCESSn(index));
        } while (ret & CPSW_MDIO_GO);

    return OK;
    }

/*****************************************************************************
*
* am65xCpswLinkUpdate - miiBus miiLinkUpdate method
*
* This function implements an miiLinkUpdate() method that allows
* miiBus to notify us of link state changes. This routine will be
* invoked by the miiMonitor task when it detects a change in link
* status. Normally, the miiMonitor task checks for link events every
* two seconds.
*
* Once we determine the new link state, we will announce the change
* to any bound protocols via muxError(). We also update the ifSpeed
* fields in the MIB2 structures so that SNMP queries can detect the
* correct link speed.
*
* RETURNS: ERROR if obtaining the new media setting fails, else OK
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswLinkUpdate
    (
    VXB_DEV_ID      pPhyParent
    )
    {
    K3CPSW_SW_CTRL  * pSwCtrl;
    K3CPSW_DRV_CTRL * pDrvCtrl;
    VXB_DEV_ID        pDev;
    UINT32            oldStatus;
    UINT32            val;

    pDrvCtrl = (K3CPSW_DRV_CTRL *) vxbDevSoftcGet(pPhyParent);

    if (pDrvCtrl == NULL)
        {
        return ERROR;
        }

    pSwCtrl = pDrvCtrl->pSwCtrl;
    
    pDev     = pSwCtrl->pDev;

    if (pDrvCtrl->cpswMiiDev== NULL)
        return ERROR;

    (void)semTake (pSwCtrl->cpswDevSem, WAIT_FOREVER);

    oldStatus = pDrvCtrl->cpswCurStatus;

    if (miiBusModeGet(pDrvCtrl->cpswMiiDev,
        &pDrvCtrl->cpswCurMedia, &pDrvCtrl->cpswCurStatus) == ERROR)
        {
        (void)semGive(pSwCtrl->cpswDevSem);
        return ERROR;
        }

    if (!(pDrvCtrl->cpswEndObj.flags & IFF_UP))
        {
        (void)semGive(pSwCtrl->cpswDevSem);
        return OK;
        }

    val = CSR_READ_4 (pSwCtrl, pDrvCtrl->portOffset + CPSW_SL_MAC_CTL);
    switch(IFM_SUBTYPE(pDrvCtrl->cpswCurMedia))
        {
        case(IFM_1000_T):
        case(IFM_1000_SX):
            val &= (UINT32)(~CPSW_EXT_EN);
            CSR_WRITE_4 (pSwCtrl, pDrvCtrl->portOffset + CPSW_SL_MAC_CTL,
                         GAMC_CTL_GIG | val);

            pDrvCtrl->cpswEndObj.mib2Tbl.ifSpeed = 1000000000;
            break;
        case(IFM_100_TX):
            val &= (UINT32)(~CPSW_EXT_EN);
            CSR_WRITE_4 (pSwCtrl, pDrvCtrl->portOffset + CPSW_SL_MAC_CTL,
                    val & (UINT32)(~GAMC_CTL_GIG));

            pDrvCtrl->cpswEndObj.mib2Tbl.ifSpeed = 100000000;
            break;
        case(IFM_10_T):
            val |= CPSW_EXT_EN;
            CSR_WRITE_4 (pSwCtrl, pDrvCtrl->portOffset + CPSW_SL_MAC_CTL,
                         val & (UINT32)(~GAMC_CTL_GIG));

            pDrvCtrl->cpswEndObj.mib2Tbl.ifSpeed = 10000000;
            break;
        default:
            pDrvCtrl->cpswEndObj.mib2Tbl.ifSpeed = 0;
            break;
        }

    val = CSR_READ_4 (pSwCtrl, pDrvCtrl->portOffset + CPSW_SL_MAC_CTL);
    if ((pDrvCtrl->cpswCurMedia & IFM_GMASK) == IFM_FDX)
        CSR_WRITE_4 (pSwCtrl, pDrvCtrl->portOffset + \
                     CPSW_SL_MAC_CTL, val | GMAC_CTL_FULLDUPLEX);
    else
        CSR_WRITE_4 (pSwCtrl, pDrvCtrl->portOffset + CPSW_SL_MAC_CTL,
                     val & (UINT32)(~GMAC_CTL_FULLDUPLEX));

    if (pDrvCtrl->cpswEndObj.pMib2Tbl != NULL)
        pDrvCtrl->cpswEndObj.pMib2Tbl->m2Data.mibIfTbl.ifSpeed =
            pDrvCtrl->cpswEndObj.mib2Tbl.ifSpeed;

    if (!(pDrvCtrl->cpswEndObj.flags & IFF_UP))
        {
        (void)semGive (pSwCtrl->cpswDevSem);
        return (OK);
        }

    /* If status went from down to up, announce link up. */

    if (pDrvCtrl->cpswCurStatus & IFM_ACTIVE && !(oldStatus & IFM_ACTIVE))
        {
        DBG_MSG (DBG_INFO,"LINK UP - %s%d, Speed: %d %s\n",
                 pDrvCtrl->cpswEndObj.devObject.name,
                 pDrvCtrl->cpswEndObj.devObject.unit,
                 pDrvCtrl->cpswEndObj.mib2Tbl.ifSpeed/1000000,
                 ((pDrvCtrl->cpswCurMedia & IFM_FDX) ? "FDX":"HDX"));

        (void)jobQueueStdPost (pDrvCtrl->cpswJobQueue,
                               NET_TASK_QJOB_PRI, muxLinkUpNotify,
                               &pDrvCtrl->cpswEndObj, NULL, NULL, NULL, NULL);
        }

    /* If status went from up to down, announce link down. */

    if (!(pDrvCtrl->cpswCurStatus & IFM_ACTIVE) && oldStatus & IFM_ACTIVE)
        {
        DBG_MSG (DBG_INFO,"LINK DOWN - %s%d, Speed: %d %s\n",
                 pDrvCtrl->cpswEndObj.devObject.name,
                 pDrvCtrl->cpswEndObj.devObject.unit,
                 pDrvCtrl->cpswEndObj.mib2Tbl.ifSpeed/1000000,
                 ((pDrvCtrl->cpswCurMedia & IFM_FDX) ? "FDX":"HDX"));

        (void)jobQueueStdPost (pDrvCtrl->cpswJobQueue,
                               NET_TASK_QJOB_PRI, muxLinkDownNotify,
                               &pDrvCtrl->cpswEndObj, NULL, NULL, NULL, NULL);
        }

    (void)semGive(pSwCtrl->cpswDevSem);

    return (OK);
    }

/*******************************************************************************
*
* am65xCpswMuxConnect - muxConnect method handler
*
* This function handles muxConnect() events, which may be triggered
* manually or(more likely) by the bootstrap code. Most VxBus
* initialization occurs before the MUX has been fully initialized,
* so the usual muxDevLoad()/muxDevStart() sequence must be defered
* until the networking subsystem is ready. This routine will ultimately
* trigger a call to am65xCpswEndLoad() to create the END interface instance.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am65xCpswMuxConnect
    (
    VXB_DEV_ID pDev,
    void     * unused
    )
    {
    K3CPSW_DRV_CTRL  * pDrvCtrl;
    int              index;

    pDrvCtrl = (K3CPSW_DRV_CTRL*) vxbDevSoftcGet(pDev);
    index    = (int)(pDrvCtrl->portIndex - 1);

    (void) miiBusMediaListGet (pDrvCtrl->cpswMiiDev, &pDrvCtrl->cpswMediaList);

    pDrvCtrl->cookie = muxDevLoad (index,
                                   am65xCpswEndLoad, "", TRUE, pDev);

    if (pDrvCtrl->cookie)
        {
        (void)muxDevStart (pDrvCtrl->cookie);
        }

    if (_func_m2PollStatsIfPoll != NULL &&
        pDrvCtrl->cpswStatsConf.ifWatchdog == NULL)
        {
        (void)endPollStatsInit (pDrvCtrl->cookie,
                                _func_m2PollStatsIfPoll);
        }
    }

/*******************************************************************************
*
* am65xCpswEndLoad - END driver entry point
*
* This routine initializes the END interface instance associated
* with this device. In traditional END drivers, this function is
* the only public interface, and it's typically invoked by a BSP
* driver configuration stub. With VxBus, the BSP stub code is no
* longer needed, and this function is now invoked automatically
* whenever this driver's muxConnect() method is called.
*
* For older END drivers, the load string would contain various
* configuration parameters, but with VxBus this use is deprecated.
* The load string should just be an empty string. The second
* argument should be a pointer to the VxBus device instance
* associated with this device. Like older END drivers, this routine
* will still return the device name if the init string is empty,
* since this behavior is still expected by the MUX. The MUX will
* invoke this function twice: once to obtain the device name,
* and then again to create the actual END_OBJ instance.
*
* When this function is called the second time, it will initialize
* the END object, perform MIB2 setup, allocate a buffer pool, and
* initialize the supported END capabilities. The only special
* capability we support is VLAN_MTU, since we can receive slightly
* larger than normal frames.
*
* RETURNS: An END object pointer, or NULL on error, or 0 and the name
* of the device if the <loadStr> was empty.
*
* ERRNO: N/A
*/

LOCAL END_OBJ * am65xCpswEndLoad
    (
    char * loadStr,
    void * pArg
    )
    {
    K3CPSW_DRV_CTRL * pDrvCtrl;
    VXB_DEV_ID      pDev;
    int             unit;

    if (loadStr == NULL)
        return NULL;

    if (loadStr[0] == 0)
        {
        bcopy (CPSW_PORT_NAME, loadStr, sizeof (CPSW_PORT_NAME));
        return NULL;
        }

    pDev = pArg;
    pDrvCtrl = (K3CPSW_DRV_CTRL*) vxbDevSoftcGet(pDev);

    /* get our instance number from load string */

    unit = atoi (loadStr);

    if (END_OBJ_INIT (&pDrvCtrl->cpswEndObj, NULL, CPSW_PORT_NAME,
        unit, &am65xCpswNetFuncs, "CPSW END") != OK)
        {
        return NULL;
        }

    if (endM2Init (&pDrvCtrl->cpswEndObj, M2_ifType_ethernet_csmacd,
                   pDrvCtrl->mac, ETHER_ADDR_LEN, ETHERMTU, 100000000,
                   IFF_NOTRAILERS | IFF_SIMPLEX | IFF_MULTICAST | IFF_BROADCAST)
                   == ERROR)
        {
        return NULL;
        }

    pDrvCtrl->mtu = CPSW_MTU;

    if (endPoolCreate ((CPSW_DESC_CNT * 3),
        &pDrvCtrl->cpswEndObj.pNetPool) != OK)
        {
        return NULL;
        }
    pDrvCtrl->cpswPollbuf = endPoolTupleGet (pDrvCtrl->cpswEndObj.pNetPool);

    /* Set up cpswPolling stats. */

    pDrvCtrl->cpswStatsConf.ifEndObj = &pDrvCtrl->cpswEndObj;
    pDrvCtrl->cpswStatsConf.ifWatchdog = NULL;
    pDrvCtrl->cpswStatsConf.ifValidCounters = (
                        END_IFINMULTICASTPKTS_VALID |
                        END_IFINBROADCASTPKTS_VALID |
                        END_IFINOCTETS_VALID |
                        END_IFINERRORS_VALID |
                        END_IFINDISCARDS_VALID |
                        END_IFOUTMULTICASTPKTS_VALID |
                        END_IFOUTBROADCASTPKTS_VALID |
                        END_IFOUTOCTETS_VALID |
                        END_IFOUTERRORS_VALID);

    /* Set up capabilities. */

    pDrvCtrl->cpswCaps.cap_available = IFCAP_VLAN_MTU;
    pDrvCtrl->cpswCaps.cap_enabled   = IFCAP_VLAN_MTU;

    return (&pDrvCtrl->cpswEndObj);
    }

/*******************************************************************************
*
* am65xCpswEndUnload - unload END driver instance
*
* This routine undoes the effects of am65xCpswEndLoad(). The END object
* is destroyed, our network pool is released, the endM2 structures
* are released, and the cpswPolling stats watchdog is terminated.
*
* Note that the END interface instance can't be unloaded if the
* device is still running. The device must be stopped with muxDevStop()
* first.
*
* RETURNS: ERROR if device is still in the IFF_UP state, otherwise OK
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswEndUnload
    (
    END_OBJ * pEnd
    )
    {
    K3CPSW_DRV_CTRL * pDrvCtrl;

    if (pEnd->flags & IFF_UP)
        return ERROR;

    pDrvCtrl = (K3CPSW_DRV_CTRL *)pEnd;

    /* free the poll mode temp buffer */

    (void) endPoolTupleFree (pDrvCtrl->cpswPollbuf);

    /* release our buffer pool */

    (void) endPoolDestroy (pDrvCtrl->cpswEndObj.pNetPool);

    /* terminate stats polling */

    (void) wdDelete (pDrvCtrl->cpswStatsConf.ifWatchdog);

    (void) endM2Free (&pDrvCtrl->cpswEndObj);

    END_OBJECT_UNLOAD (&pDrvCtrl->cpswEndObj);

    /* prevent freeing of pDrvCtrl */

    return (EALREADY);
    }

/*******************************************************************************
*
* am65xCpswEndMCastAddrAdd - add a multicast address for the device
*
* This routine adds a multicast address to whatever the driver
* is already listening for. It then resets the address filter.
*
* RETURNS: OK, or ERROR when 1) the parameter is not a valid multicast address
* and 2) there is no enough resource for etherMultiAdd()
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswEndMCastAddrAdd
    (
    END_OBJ * pEnd,
    char    * pAddr
    )
    {
    INT32           retVal;
    INT32           addMulRet;
    K3CPSW_DRV_CTRL * pDrvCtrl =(K3CPSW_DRV_CTRL *)pEnd;
    K3CPSW_SW_CTRL *  pSwCtrl = pDrvCtrl->pSwCtrl;

    (void)semTake (pSwCtrl->cpswDevSem, WAIT_FOREVER);

    if ((pDrvCtrl->cpswEndObj.flags & IFF_UP) == 0)
        {
        (void)semGive (pSwCtrl->cpswDevSem);
        return OK;
        }

    retVal = etherMultiAdd (&pEnd->multiList, pAddr);

    if (retVal == ENETRESET)
        {
        pEnd->nMulti++;
        addMulRet = am65xCpswAleAddMultiCast (pSwCtrl->pDev, (UINT8 *)pAddr, 
                                         (INT32)pDrvCtrl->portVlan, 
                                         CPSW_ALE_MCAST_FWD_2, 
                                         CPSW_ALE_MCAST_SUPER,
                                         1 << pSwCtrl->hostPortIndex);
        }

    (void)semGive (pSwCtrl->cpswDevSem);

    if (((retVal == ENETRESET) && (addMulRet == 0)) || (retVal == OK))
        {
        return OK;
        }
    else
        {
        return ERROR;
        }
    }

/*******************************************************************************
*
* am65xCpswEndMCastAddrDel - delete a multicast address for the device
*
* This routine removes a multicast address from whatever the driver
* is listening for. It then resets the address filter.
*
* RETURNS: OK, or ERROR when the multicast address does not exist.
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswEndMCastAddrDel
    (
    END_OBJ * pEnd,
    char    * pAddr
    )
    {
    INT32           retVal;
    K3CPSW_DRV_CTRL * pDrvCtrl =(K3CPSW_DRV_CTRL *) pEnd;
    K3CPSW_SW_CTRL *  pSwCtrl = pDrvCtrl->pSwCtrl;

    (void)semTake (pSwCtrl->cpswDevSem, WAIT_FOREVER);

    if ((pDrvCtrl->cpswEndObj.flags & IFF_UP) == 0)
        {
        (void)semGive (pSwCtrl->cpswDevSem);
        return OK;
        }

    retVal = etherMultiDel (&pEnd->multiList, pAddr);

    if (retVal == ENETRESET)
        {
        pEnd->nMulti--;
        am65xCpswAleDelMultiCast (pSwCtrl->pDev, (UINT8 *)pAddr, 
                                     (INT32)pDrvCtrl->portVlan, 
                                     CPSW_ALE_MCAST_FWD_2, 
                                     CPSW_ALE_MCAST_SUPER,
                                     1 << pSwCtrl->hostPortIndex);
        }

    (void)semGive (pSwCtrl->cpswDevSem);

    if ((retVal == ENETRESET) || (retVal == OK))
        {
        return OK;
        }
    else
        {
        return ERROR;
        }
    }

/*******************************************************************************
*
* am65xCpswEndMCastAddrGet - get the multicast address list for the device
*
* This routine gets the multicast list of whatever the driver
* is already listening for.
*
* RETURNS: OK, always.
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswEndMCastAddrGet
    (
    END_OBJ     * pEnd,
    MULTI_TABLE * pTable
    )
    {
    INT32 retVal;
    K3CPSW_DRV_CTRL * pDrvCtrl =(K3CPSW_DRV_CTRL *) pEnd;
    K3CPSW_SW_CTRL  * pSwCtrl  = pDrvCtrl->pSwCtrl;

    (void)semTake (pSwCtrl->cpswDevSem, WAIT_FOREVER);

    if ((pDrvCtrl->cpswEndObj.flags & IFF_UP) == 0)
        {
        (void)semGive (pSwCtrl->cpswDevSem);
        return OK;
        }

    retVal = etherMultiGet (&pEnd->multiList, pTable);

    if (retVal == ENETRESET)
        {
        pEnd->nMulti++;
        }

    (void)semGive (pSwCtrl->cpswDevSem);

    return OK;
    }

/*******************************************************************************
*
* am65xCpswEndStatsDump - return polled statistics counts
*
* This routine is automatically invoked periodically by the polled
* statistics watchdog.
*
* RETURNS: always OK
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswEndStatsDump
    (
    K3CPSW_DRV_CTRL * pDrvCtrl
    )
    {
    UINT32              i;
    K3CPSW_SW_CTRL * pSwCtrl = pDrvCtrl->pSwCtrl;
    UINT32         * temp = (UINT32 *)&pSwCtrl->cpswStat;
    END_IFCOUNTERS * pEndStatsCounters;

    /*
     * hardware statistic counters are write-to-decrement,
     * after a read, we write the value read to clear
     * the counters
     */

    for (i = 0; i < sizeof (CPSW_STAT) / sizeof (UINT32); i++)
        {
        *temp = CSR_READ_4 (pSwCtrl, pSwCtrl->statsOffset + i*4);
        CSR_WRITE_4 (pSwCtrl, pSwCtrl->statsOffset + i*4, *temp);
        temp++;
        }

    pEndStatsCounters = &pDrvCtrl->cpswStatsCounters;

    pEndStatsCounters->ifInOctets = pSwCtrl->cpswStat.rxoctets;
    pEndStatsCounters->ifInMulticastPkts = pSwCtrl->cpswStat.rxmulticast;
    pEndStatsCounters->ifInBroadcastPkts = pSwCtrl->cpswStat.rxbroadcast;
    pEndStatsCounters->ifInErrors = pSwCtrl->cpswStat.rxpause +
                                    pSwCtrl->cpswStat.rxcrcerros +
                                    pSwCtrl->cpswStat.rxalignmenterrors +
                                    pSwCtrl->cpswStat.rxoversized +
                                    pSwCtrl->cpswStat.rxjabber +
                                    pSwCtrl->cpswStat.rxundersized;
    pEndStatsCounters->ifInDiscards = pDrvCtrl->cpswInDropped;
    pEndStatsCounters->ifOutOctets = pSwCtrl->cpswStat.txoctets;
    pEndStatsCounters->ifOutMulticastPkts = pSwCtrl->cpswStat.txmulticast;
    pEndStatsCounters->ifOutBroadcastPkts = pSwCtrl->cpswStat.txbroadcast;
    pEndStatsCounters->ifOutErrors = pSwCtrl->cpswStat.txpause +
                                     pSwCtrl->cpswStat.txdefered +
                                     pSwCtrl->cpswStat.txcollision +
                                     pSwCtrl->cpswStat.txexceesive +
                                     pSwCtrl->cpswStat.txsinglecol+
                                     pSwCtrl->cpswStat.txmulticol +
                                     pSwCtrl->cpswStat.txlatecol +
                                     pSwCtrl->cpswStat.txunderrun;

    return OK;
    }

/*******************************************************************************
*
* am65xCpswEndIoctl - the driver I/O control routine
*
* This function processes ioctl requests supplied via the muxIoctl()
* routine. In addition to the normal boilerplate END ioctls, this
* driver supports the IFMEDIA ioctls, END capabilities ioctls, and
* polled stats ioctls.
*
* RETURNS: A command specific response, usually OK or ERROR.
*
* ERRNO: N/A
*/

LOCAL INT32 am65xCpswEndIoctl
    (
    END_OBJ * pEnd,
    INT32     cmd,
    caddr_t   data
    )
    {
    K3CPSW_DRV_CTRL    * pDrvCtrl = (K3CPSW_DRV_CTRL *)pEnd;
    K3CPSW_SW_CTRL     * pSwCtrl = pDrvCtrl->pSwCtrl;
    VXB_DEV_ID           pDev;
    END_MEDIALIST      * mediaList;
    END_CAPABILITIES   * hwCaps;
    END_MEDIA          * pMedia;
    END_RCVJOBQ_INFO   * qinfo;
    UINT32               nQs;
    INT32                value;
    INT32                error = OK;

    pSwCtrl = pDrvCtrl->pSwCtrl;
    pDev    = pSwCtrl->pDev;

    if (cmd != EIOCPOLLSTART && cmd != EIOCPOLLSTOP)
        (void)semTake (pSwCtrl->cpswDevSem, WAIT_FOREVER);

    switch (cmd)
        {
        case EIOCSADDR:
            if (data == NULL)
                error = EINVAL;
            else
                {
                bcopy ((char *)data,
                    (char *)pEnd->mib2Tbl.ifPhysAddress.phyAddress,
                    ETHER_ADDR_LEN);
                if (pEnd->pMib2Tbl != NULL)
                    bcopy ((char *)data,
                    (char *)pEnd->pMib2Tbl->m2Data.mibIfTbl.ifPhysAddress.phyAddress,
                    ETHER_ADDR_LEN);

                (void)am65xCpswAleAddUniCast (pDev, (UINT8 *)data, (INT32)pDrvCtrl->portVlan,
                                         0, 0, pSwCtrl->hostPortIndex);
                am65xCpswAleDelUniCast (pDev, pDrvCtrl->mac, (INT32)pDrvCtrl->portVlan, 
                                   0, 0, pSwCtrl->hostPortIndex);
                bcopy ((char *)data, (char *)pDrvCtrl->mac, ETHER_ADDR_LEN);
                }
            break;

        case EIOCGADDR:
            if (data == NULL)
                error = EINVAL;
            else
                bcopy ((char *)pDrvCtrl->mac, (char *)data, ETHER_ADDR_LEN);
            break;

        case EIOCSFLAGS:
            {
            INT32 oldFlags;
            INT32 newFlags;
            oldFlags = END_FLAGS_GET (pEnd);
            value = (INT32)(long)data;
            if (value < 0)
                {
                value = -value;
                value--;
                END_FLAGS_CLR (pEnd, value);
                }
            else
                {
                END_FLAGS_SET (pEnd, value);
                }

            newFlags = END_FLAGS_GET (pEnd);

            if (!(newFlags & IFF_PROMISC) && (oldFlags & IFF_PROMISC))
                {
                UINT32 tmp = CPSW_ALE_EN_TABLE;
                if (pSwCtrl->workMode == CPSW_MODE_INDEPENDENT_PORT)
                    tmp |= CPSW_ALE_VLAN_AWARE;
                CSR_WRITE_4 (pSwCtrl, pSwCtrl->aleOffset + CPSW_ALE_CONTROL, tmp);
                }
            if ((newFlags & IFF_PROMISC) && !(oldFlags & IFF_PROMISC))
                {
                CSR_WRITE_4 (pSwCtrl, pSwCtrl->aleOffset + CPSW_ALE_CONTROL,
                             CPSW_ALE_EN_TABLE | CPSW_ALE_BYPASS);
                }
            }

            break;

        case EIOCGFLAGS:
            if (data == NULL)
                error = EINVAL;
            else
                *(INT32 *)data = END_FLAGS_GET(pEnd);

            break;

        case EIOCMULTIADD:
            error = am65xCpswEndMCastAddrAdd (pEnd,(char *) data);
            break;

        case EIOCMULTIDEL:
            error = am65xCpswEndMCastAddrDel (pEnd,(char *) data);
            break;

        case EIOCMULTIGET:
            error = am65xCpswEndMCastAddrGet (pEnd,(MULTI_TABLE *) data);
            break;

        case EIOCPOLLSTART:
            /* set poll flag */

            pDrvCtrl->cpswPolling = TRUE;

            break;

        case EIOCPOLLSTOP:

            pDrvCtrl->cpswPolling = FALSE;

            break;

        case EIOCGMIB2233:
        case EIOCGMIB2:
            error = endM2Ioctl (&pDrvCtrl->cpswEndObj, cmd, data);
            break;

        case EIOCGPOLLCONF:
            if (data == NULL)
                error = EINVAL;
            else
                {
                pDrvCtrl->cpswStatsConf.ifPollInterval = (int)sysClkRateGet();
                *((END_IFDRVCONF **)data) = &pDrvCtrl->cpswStatsConf;
                }
            break;

        case EIOCGPOLLSTATS:
            if (data == NULL)
                error = EINVAL;
            else
                {
                error = am65xCpswEndStatsDump (pDrvCtrl);
                if (error == OK)
                    *((END_IFCOUNTERS **)data) = &pDrvCtrl->cpswStatsCounters;
                }
            break;

        case EIOCGMEDIALIST:
            if (data == NULL)
                {
                error = EINVAL;
                break;
                }
            if (pDrvCtrl->cpswMediaList->endMediaListLen == 0)
                {
                error = ENOTSUP;
                break;
                }

            mediaList =(END_MEDIALIST *)data;
            if (mediaList->endMediaListLen <
                pDrvCtrl->cpswMediaList->endMediaListLen)
                {
                mediaList->endMediaListLen =
                    pDrvCtrl->cpswMediaList->endMediaListLen;
                error = ENOSPC;
                break;
                }

            bcopy ((char *)pDrvCtrl->cpswMediaList, (char *)mediaList,
                  sizeof (END_MEDIALIST) + (sizeof (UINT32) *
                  pDrvCtrl->cpswMediaList->endMediaListLen));
            break;

        case EIOCGIFMEDIA:
            if (data == NULL)
                error = EINVAL;
            else
                {
                pMedia =(END_MEDIA *)data;
                pMedia->endMediaActive = pDrvCtrl->cpswCurMedia;
                pMedia->endMediaStatus = pDrvCtrl->cpswCurStatus;
                }
            break;

        case EIOCSIFMEDIA:
            if (data == NULL)
                error = EINVAL;
            else
                {
                pMedia =(END_MEDIA *)data;
                (void)miiBusModeSet (pDrvCtrl->cpswMiiDev,
                                     pMedia->endMediaActive);
                (void)am65xCpswLinkUpdate (pDrvCtrl->pDev);
                error = OK;
                }
            break;

        case EIOCGIFCAP:
            hwCaps =(END_CAPABILITIES *)data;
            if (hwCaps == NULL)
                {
                error = EINVAL;
                break;
                }
            hwCaps->cap_available = pDrvCtrl->cpswCaps.cap_available;
            hwCaps->cap_enabled = pDrvCtrl->cpswCaps.cap_enabled;
            break;

        case EIOCSIFCAP:
            error = ENOTSUP;
            break;

        case EIOCGIFMTU:
            if (data == NULL)
                error = EINVAL;
            else
                *(INT32 *)data = pEnd->mib2Tbl.ifMtu;
            break;

        case EIOCSIFMTU:
            value =(INT32)data;
            if (value <= 0 || value > pDrvCtrl->mtu)
                {
                error = EINVAL;
                break;
                }
            pEnd->mib2Tbl.ifMtu = value;
            if (pEnd->pMib2Tbl != NULL)
                pEnd->pMib2Tbl->m2Data.mibIfTbl.ifMtu = value;
            break;

        case EIOCGRCVJOBQ:
            if (data == NULL)
                {
                error = EINVAL;
                break;
                }

            qinfo = (END_RCVJOBQ_INFO *)data;
            nQs = qinfo->numRcvJobQs;
            qinfo->numRcvJobQs = 1;
            if (nQs < 1)
                error = ENOSPC;
            else
                qinfo->qIds[0] = pDrvCtrl->cpswJobQueue;
            break;

        default:
            error = EINVAL;
            break;
        }

    if (cmd != EIOCPOLLSTART && cmd != EIOCPOLLSTOP)
        (void)semGive (pSwCtrl->cpswDevSem);

    return (error);
    }

/*******************************************************************************
*
* am65xCpswEndStart - start the device
*
* This function resets the device to put it into a known state and
* then configures it for RX and TX operation. The RX and TX configuration
* registers are initialized, and the address of the RX DMA window is
* loaded into the device. Interrupts are then enabled, and the initial
* link state is configured.
*
* Note that this routine also checks to see if an alternate jobQueue
* has been specified via the vxbParam subsystem. This allows the driver
* to divert its work to an alternate processing task, such as may be
* done with TIPC. This means that the jobQueue can be changed while
* the system is running, but the device must be stopped and restarted
* for the change to take effect.
*
* RETURNS: ERROR if device initialization failed, otherwise OK
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswEndStart
    (
    END_OBJ * pEnd
    )
    {
    K3CPSW_DRV_CTRL * pDrvCtrl = (K3CPSW_DRV_CTRL *)pEnd;
    K3CPSW_SW_CTRL  * pSwCtrl = pDrvCtrl->pSwCtrl;
    UINT32            val;
    STATUS            ret = OK;

    (void)semTake(pSwCtrl->cpswDevSem, WAIT_FOREVER);
    (void)END_TX_SEM_TAKE (pEnd, WAIT_FOREVER);

    if (pEnd->flags & IFF_UP)
        {
        ret = ERROR;
        DBG_MSG (DBG_ERR, "end already startup\n");

        goto out;
        }

    /* set the default job queue to netJobQueueId */

    pDrvCtrl->cpswJobQueue = netJobQueueId;

    /* setup the job queue fucntion */

    QJOB_SET_PRI(&pDrvCtrl->cpswTxQJob, NET_TASK_QJOB_PRI);
    pDrvCtrl->cpswTxQJob.func = (QJOB_FUNC)am65xCpswEndTxHandle;

    QJOB_SET_PRI(&pDrvCtrl->cpswRxQJob, NET_TASK_QJOB_PRI);
    pDrvCtrl->cpswRxQJob.func = (QJOB_FUNC)am65xCpswEndRxHandle;

    QJOB_SET_PRI(&pDrvCtrl->cpswMiscQJob, NET_TASK_QJOB_PRI);
    pDrvCtrl->cpswMiscQJob.func = (QJOB_FUNC)am65xCpswEndMiscHandle;


    /* enable the GMII */

    val = CSR_READ_4 (pSwCtrl, pDrvCtrl->portOffset + CPSW_SL_MAC_CTL);
    val |= CPSW_GMII_EN;
    CSR_WRITE_4 (pSwCtrl, pDrvCtrl->portOffset + CPSW_SL_MAC_CTL, val);

    /* tell the stack that we are on-line */

    pDrvCtrl->cpswCurMedia = IFM_ETHER | IFM_NONE;
    pDrvCtrl->cpswCurStatus = IFM_AVALID;

    (void)miiBusModeSet (pDrvCtrl->cpswMiiDev, IFM_ETHER | IFM_AUTO);

    END_FLAGS_SET (pEnd, (IFF_UP | IFF_RUNNING));

out:
    (void)END_TX_SEM_GIVE (pEnd);
    (void)semGive (pSwCtrl->cpswDevSem);

    DBG_MSG(DBG_INFO,"%s%d: cpswEndStart() ok\n",
            CPSW_PORT_NAME, pDrvCtrl->portIndex - 1);

    return ret;
    }

/*******************************************************************************
*
* am65xCpswEndStop - stop the device
*
* This function undoes the effects of am65xCpswEndStart(). The device is shut
* down and all resources are released. Note that the shutdown process
* pauses to wait for all pending RX, TX and link event jobs that may have
* been initiated by the interrupt handler to complete. This is done
* to prevent tNetTask from accessing any data that might be released by
* this routine.
*
* RETURNS: ERROR if device shutdown failed, otherwise OK
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswEndStop
    (
    END_OBJ * pEnd
    )
    {
    K3CPSW_DRV_CTRL * pDrvCtrl = (K3CPSW_DRV_CTRL *)pEnd;
    K3CPSW_SW_CTRL  * pSwCtrl = pDrvCtrl->pSwCtrl;
    UINT32            val;
    int               i;

    DBG_MSG(DBG_INFO, "%s%d: cpswEndStop is called\n",
            CPSW_PORT_NAME, pDrvCtrl->portIndex - 1);

    (void)semTake (pSwCtrl->cpswDevSem, WAIT_FOREVER);

    if ((pEnd->flags & IFF_UP) == 0)
        {
        (void)semGive (pSwCtrl->cpswDevSem);
        return OK;
        }

    /* clear IFF_UP | IFF_RUNNING */

    END_FLAGS_CLR (pEnd,(IFF_UP | IFF_RUNNING));

    /* disable the GMII */

    val = CSR_READ_4 (pSwCtrl, pDrvCtrl->portOffset + CPSW_SL_MAC_CTL);
    val &= (UINT32)(~CPSW_GMII_EN);
    CSR_WRITE_4 (pSwCtrl, pDrvCtrl->portOffset + CPSW_SL_MAC_CTL, val);

    /* release rx mblk resources */

    for (i = 0; i < CPSW_DESC_CNT; i++)
        {
        if (pDrvCtrl->cpswRxblk[i] != NULL)
            {
            endPoolTupleFree (pDrvCtrl->cpswRxblk[i]);
            pDrvCtrl->cpswRxblk[i] = NULL;
            }
        }

    /*
     * Flush the recycle cache to shake loose any of our mBlks that may be
     * stored there.
     */

    endMcacheFlush ();

    (void) END_TX_SEM_TAKE (pEnd, WAIT_FOREVER);

    for (i = 0; i < CPSW_DESC_CNT; i++)
        {
        if (pDrvCtrl->cpswTxblk [i] != NULL)
            {
            endPoolTupleFree (pDrvCtrl->cpswTxblk [i]);
            pDrvCtrl->cpswTxblk [i] = NULL;
            }
        }

    (void) END_TX_SEM_GIVE (pEnd);

    (void) semGive (pSwCtrl->cpswDevSem);

    DBG_MSG(DBG_INFO, "%s%d: cpswEndStop done\n",
            CPSW_PORT_NAME, pDrvCtrl->portIndex - 1);

    return OK;
    }

/*******************************************************************************
*
* am65xCpswEndRxHandle - process received frames
*
* This function is scheduled by the ISR to run in the context of tNetTask
* whenever an RX interrupt is received. It processes packets from the
* RX window and encapsulates them into mBlk tuples which are handed up
* to the MUX.
*
* There may be several packets waiting in the window to be processed.
* We take care not to process too many packets in a single run through
* this function so as not to monopolize tNetTask and starve out other
* jobs waiting in the jobQueue. If we detect that there's still more
* packets waiting to be processed, we queue ourselves up for another
* round of processing.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am65xCpswEndRxHandle
    (
    void * pArg
    )
    {
    QJOB          * job = pArg;
    K3CPSW_DRV_CTRL * pDrvCtrl = member_to_object (job,
                               K3CPSW_DRV_CTRL, cpswRxQJob);
    K3CPSW_SW_CTRL * pSwCtrl = pDrvCtrl->pSwCtrl;
    M_BLK_ID        pMblk;
    M_BLK_ID        pNextMblk;

    (void)semTake (pSwCtrl->cpswDevSem, WAIT_FOREVER);

    pMblk = pDrvCtrl->cpswRcvChain;
    pDrvCtrl->cpswRcvChain = NULL;
    pDrvCtrl->pCpswRcvChainTail = &pDrvCtrl->cpswRcvChain;

    (void) semGive (pSwCtrl->cpswDevSem);

    while (pMblk != NULL)
        {
        pNextMblk = TUP_NEXT_PK_GET (pMblk);
        TUP_NEXT_PK_SET (pMblk, NULL);
        END_RCV_RTN_CALL (&pDrvCtrl->cpswEndObj, pMblk);
        pMblk = pNextMblk;
        }

    }

/*******************************************************************************
*
* am65xCpswEndTxHandle - process TX completion events
*
* This function is scheduled by the ISR to run in the context of tNetTask
* whenever an TX interrupt is received. It runs through all of the
* TX register pairs and checks the TX status to see how many have
* completed. For each completed transmission, the associated TX mBlk
* is released, and the outbound packet stats are updated.
*
* If the transmitter has stalled, this routine will also call muxTxRestart()
* to drain any packets that may be waiting in the protocol send queues,
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am65xCpswEndTxHandle
    (
    void * pArg
    )
    {
    QJOB          *   job = pArg;
    K3CPSW_DRV_CTRL * pDrvCtrl = member_to_object (job,
                               K3CPSW_DRV_CTRL, cpswTxQJob);
    K3CPSW_SW_CTRL  * pSwCtrl= pDrvCtrl->pSwCtrl;
    BOOL              restart = FALSE;

    (void)semTake (pSwCtrl->cpswDevSem, WAIT_FOREVER);
    (void)END_TX_SEM_TAKE (&pDrvCtrl->cpswEndObj, WAIT_FOREVER);

    if (pDrvCtrl->cpswTxstall == TRUE)
        {
        pDrvCtrl->cpswTxstall = FALSE;
        restart = TRUE;
        }

    (void)END_TX_SEM_GIVE (&pDrvCtrl->cpswEndObj);
    (void)semGive (pSwCtrl->cpswDevSem);

    if (restart == TRUE)
        muxTxRestart (pDrvCtrl);

    (void)vxAtomicSet (&pDrvCtrl->cpswTxIntPend, FALSE);

    }

/*******************************************************************************
*
* am65xCpswEndMiscHandle - reset hardware when need
*
* This routine reset hardware when need.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am65xCpswEndMiscHandle
    (
    void * pArg
    )
    {
    QJOB          * job = pArg;
    K3CPSW_DRV_CTRL * pDrvCtrl = member_to_object (job,
                               K3CPSW_DRV_CTRL, cpswMiscQJob);

    /* reset hardware */

    if (muxDevStop (pDrvCtrl->cookie) != OK)
        {
        END_LOG2 ("%s%d muxDevStop return ERROR\n", CPSW_PORT_NAME,
                  (pDrvCtrl->portIndex - 1));
        return;
        }

    if (muxDevStart (pDrvCtrl->cookie) != OK)
        {
        END_LOG2 ("%s%d muxDevStart return ERROR\n", CPSW_PORT_NAME,
                  (pDrvCtrl->portIndex - 1));
        return;
        }
    }

/*******************************************************************************
*
* am65xCpswEndEncap - Encapsulate an outbound packet in the TX ring
*
* This function transmits the packet specified in <pMblk>.
*
* RETURNS: ERROR if failed to send, otherwise OK.
*
* ERRNO: N/A
*/

LOCAL INT32 am65xCpswEndEncap
    (
    K3CPSW_DRV_CTRL * pDrvCtrl,
    M_BLK_ID        pMblk
    )
    {
    TI_K3_NAVSS_UDMA_PKT_DATA pData;

    while (tiK3NavssUdmaSendTry() == ERROR) taskDelay (0);

    if (pDrvCtrl->pMblkToFree != NULL)
        {
        endPoolTupleFree (pDrvCtrl->pMblkToFree);
        pDrvCtrl->pMblkToFree = NULL;
        }

    pData.dstTag = pDrvCtrl->portIndex;
    pData.pktType = CPSW_CPPI_PKT_TYPE;

    if (tiK3NavssUdmaSend (mtod(pMblk, void *), (UINT32)pMblk->m_len, 
        (void *)&pData) == OK)
        {
        if (pMblk != pDrvCtrl->cpswPollbuf)
            {
            pDrvCtrl->pMblkToFree = pMblk;
            }
        return OK;
        }
    else
        {
        return ERROR;
        }
    }

/*******************************************************************************
*
* am65xCpswEndSend - transmit a packet
*
* This function transmits the packet specified in <pMblk>.
*
* RETURNS: OK, ERROR, or END_ERR_BLOCK.
*
* ERRNO: N/A
*/

LOCAL INT32 am65xCpswEndSend
    (
    END_OBJ *   pEnd,
    M_BLK_ID    pMblk
    )
    {
    K3CPSW_DRV_CTRL * pDrvCtrl;
    UINT32            rval;
    M_BLK_ID          pTmp;
    K3CPSW_SW_CTRL  * pSwCtrl;

    pDrvCtrl = (K3CPSW_DRV_CTRL *)pEnd;
    pSwCtrl = pDrvCtrl->pSwCtrl;

    (void)semTake (pSwCtrl->cpswDevSem, WAIT_FOREVER);
    (void)END_TX_SEM_TAKE (pEnd, WAIT_FOREVER);

    if (pDrvCtrl->cpswPolling == TRUE)
        {
        (void)END_TX_SEM_GIVE (pEnd);
        (void)semGive (pSwCtrl->cpswDevSem);

        endPoolTupleFree (pMblk);

        DBG_MSG (DBG_ERR,"%s%d cpswEndSend() is called in polling mode\n",
                 CPSW_PORT_NAME, pDrvCtrl->portIndex - 1);
        return ERROR;
        }

    if (pMblk->m_next != NULL)
        {

        if ((pTmp = endPoolTupleGet (pDrvCtrl->cpswEndObj.pNetPool)) == NULL)
            goto blocked;

        pTmp->m_len = pTmp->m_pkthdr.len =
            netMblkToBufCopy (pMblk, mtod(pTmp, char *), NULL);
        pTmp->m_flags = pMblk->m_flags;

        rval = (UINT32)am65xCpswEndEncap(pDrvCtrl, pTmp);
        if (rval == OK)
            endPoolTupleFree(pMblk);
        else
            endPoolTupleFree(pTmp);
        }
    else
        {
        rval = (UINT32)am65xCpswEndEncap (pDrvCtrl, pMblk);
        }

    if (rval != OK)
        {
        goto blocked;
        }

    (void)END_TX_SEM_GIVE(pEnd);
    (void)semGive (pSwCtrl->cpswDevSem);

    return (OK);

blocked:
    pDrvCtrl->cpswTxstall = TRUE;

    (void)END_TX_SEM_GIVE(pEnd);
    (void)semGive (pSwCtrl->cpswDevSem);

    return (END_ERR_BLOCK);
    }

/*******************************************************************************
*
* am65xCpswEndPollSend - polled mode transmit routine
*
* This function is similar to the am65xCpswEndSend() routine shown above, except
* it performs transmissions synchronously with interrupts disabled. After
* the transmission is initiated, the routine will poll the state of the
* TX status register associated with the current slot until transmission
* completed.
*
* RETURNS: OK, EAGAIN, or ERROR
*
* ERRNO: N/A
*/

LOCAL STATUS am65xCpswEndPollSend
    (
    END_OBJ * pEnd,
    M_BLK_ID  pMblk
    )
    {
    M_BLK_ID          pTemp;
    K3CPSW_DRV_CTRL * pDrvCtrl = (K3CPSW_DRV_CTRL *)pEnd;

    if (pDrvCtrl->cpswPolling == FALSE)
        return ERROR;

    while (tiK3NavssUdmaSendTry() == ERROR)
        ;

    pTemp = pDrvCtrl->cpswPollbuf;
    pTemp->m_len = pTemp->m_pkthdr.len =
         netMblkToBufCopy (pMblk, mtod(pTemp, char *), NULL);
    pTemp->m_pkthdr.csum_flags = pMblk->m_pkthdr.csum_flags;
    pTemp->m_pkthdr.csum_data  = pMblk->m_pkthdr.csum_data;
    pTemp->m_pkthdr.vlan       = pMblk->m_pkthdr.vlan;

    if (am65xCpswEndEncap (pDrvCtrl, pTemp) != OK)
        {
        return EAGAIN;
        }

    return OK;
    }

/*******************************************************************************
*
* am65xCpswEndPollReceive - polled mode receive routine
*
* This function receives a packet in polled mode, with interrupts disabled.
* It's similar in operation to the am65xCpswEndRxHandle() routine, except it
* doesn't process more than one packet at a time and does not load out
* buffers. Instead, the caller supplied an mBlk tuple into which this
* function will place the received packet.
*
* RETURNS: OK, or ERROR if operation failed.
*
* ERRNO: N/A
*/

LOCAL INT32 am65xCpswEndPollReceive
    (
    END_OBJ * pEnd,
    M_BLK_ID  pMblk
    )
    {
    K3CPSW_DRV_CTRL * pDrvCtrl = (K3CPSW_DRV_CTRL *)pEnd;
    int             rcvLen;

    if ((pEnd == NULL) || (pMblk == NULL))
        {
        return ERROR;
        }

    if (pDrvCtrl->cpswPolling == FALSE)
        {
        return ERROR;
        }

    if (!(pMblk->m_flags & M_EXT))
        {
        DBG_MSG (DBG_ERR, "%s%d cpswEndPollReceive pMblk has no M_EXT\n",
                 CPSW_PORT_NAME, pDrvCtrl->portIndex - 1);
        return ERROR;
        }

    rcvLen = tiK3NavssUdmaRecieve ((void *)(pMblk->m_data), NULL);
    if (rcvLen <= 0)
        {
        return EAGAIN;
        }

    pMblk->m_len = pMblk->m_pkthdr.len = rcvLen;
    pMblk->m_flags = M_PKTHDR | M_EXT;

    return OK;
    }

/*******************************************************************************
*
* am65xCpswRcvPollTask - receive poll task function
*
* This function loops on tiK3NavssUdmaRecieve(). Once the tiK3NavssUdmaRecieve()
* returns a valid packet, it calls jobQuequePost() to launch subsequent
* operations in network task.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am65xCpswRcvPollTask (K3CPSW_DRV_CTRL * pDrvCtrl)
    {
    M_BLK_ID         pMblk = NULL;
    int              rcvLen;
    K3CPSW_SW_CTRL * pSwCtrl = pDrvCtrl->pSwCtrl;

    while (1)
        {
        if (pDrvCtrl->cpswPolling)
            {
            taskDelay (1);
            continue;
            }

        if (pMblk == NULL)
            {
            pMblk = endPoolTupleGet (pDrvCtrl->cpswEndObj.pNetPool);
            }

        if (pMblk == NULL)
            {
            taskDelay (1);
            continue;
            }
        
        if ((rcvLen = tiK3NavssUdmaRecieve ((void *)(pMblk->m_data), NULL))
            > 0)
            {
            /*kprintf ("receive %d bytes\n", rcvLen);*/
            pMblk->m_len = pMblk->m_pkthdr.len = rcvLen;
            pMblk->m_flags = M_PKTHDR | M_EXT;
            (void)semTake (pSwCtrl->cpswDevSem, WAIT_FOREVER);
            *(pDrvCtrl->pCpswRcvChainTail) = pMblk;
            TUP_NEXT_PK_SET (pMblk, NULL);
            pDrvCtrl->pCpswRcvChainTail = &(pMblk->mBlkHdr.mNextPkt);
            (void)semGive (pSwCtrl->cpswDevSem);

            (void) jobQueuePost (pDrvCtrl->cpswJobQueue,
                                     &pDrvCtrl->cpswRxQJob);
            pMblk = NULL;
            }
        else
            {
            /* The pMblk is reused, nothing to be done */
            taskDelay (0);
            }
        }
    }
