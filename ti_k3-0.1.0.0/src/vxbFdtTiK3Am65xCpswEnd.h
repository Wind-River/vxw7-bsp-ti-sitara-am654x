/* vxbFdtTiK3Am65xCpswEnd.h - TI 3 port switch VxBus END driver header file */

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
11jun19,g_x  written based on vxbFdtTiCpswEnd.h (VXWPG-114)
*/

#ifndef __INCvxbFdtTiK2Am65xCpswEndh
#define __INCvxbFdtTiK2Am65xCpswEndh

#ifdef __cplusplus
extern "C" {
#endif

#define CPSW_PORT_NAME              "cpsw"

/* the offset bases for submodules */

#define MCU_CPSW0_NUSS_BASE          0
#define MCU_CPSW0_MDIO_BASE          0xf00
#define MCU_CPSW0_CPINT_BASE         0x1000
#define MCU_CPSW0_CONTROL_BASE       0x20000
#define MCU_CPSW0_P0_BASE            0x21000
#define MCU_CPSW0_P1_BASE            0x22000
#define MCU_CPSW0_STAT0_BASE         0x3a000
#define MCU_CPSW0_STAT1_BASE         0x3a200
#define MCU_CPSW0_ALE_BASE           0x3e000

/* global settings */

#define CPSW_DESC_CNT                (128)
#define NR_MAC_PORTS                 (2)
#define NR_DMA_CHANS                 (8)
#define NR_HDP                       (8)
#define CPSW_MTU                     (1536)
#define CPSW_TIMEOUT_VAL             (0x0fffffff)
#define CPSW_TXQ_INVALID             (0xffffffff)

/* BD descriptions */

#define CPSW_SOP                     (0x80000000)
#define CPSW_EOP                     (0x40000000)
#define CPSW_OWNERSHIP               (0x20000000)
#define CPSW_EOQ                     (0x10000000)
#define CPSW_TO_PORT_EN              (0x100000)
#define CPSW_TO_PORT_SHIFT           (16)
#define CPSW_DESC_ALIGNMENT          (4096)
#define CPSW_PASS_CRC                (0x4000000)
#define CPSW_PKT_LEN_MASK            (0x7ff)
#define OVERSIZE                     (0x01000000)
#define JABBER                       (0x02000000)
#define MAC_CTL                      (0x00800000)
#define OVERRUN                      (0x00400000)
#define PKT_ERROR                    (0x00300000)
#define CPSW_PKT_ERROR               (OVERSIZE | JABBER | MAC_CTL | OVERRUN | PKT_ERROR)
#define CPSW_MIN_PKT_PADDING         (60)

/* MDIO register offset and bit definitions */

#define CPSW_IN_CLK_FREQ             200000000U
#define CPSW_MDIO_CLK_FREQ           1000000U
#define CPSW_MDIO_CONTROL            (0x4)
#define CPSW_USERACCESSn(n)          (0x80 + (n) * 0x8)
#define CPSW_MDIO_CLK_DIV            (CPSW_IN_CLK_FREQ / CPSW_MDIO_CLK_FREQ - 1)
#define CPSW_MDIO_EN                 (0x40000000)
#define CPSW_MDIO_GO                 (0x80000000)
#define CPSW_MDIO_WRITE              (0x40000000)
#define CPSW_MDIO_ACK                (0x20000000)
#define CPSW_PHY_ADDR_SHIFT          (16)
#define CPSW_REG_ADDR_SHIFT          (21)

/* global register offset and bit definitions */

#define CPSW_ID_VER                  (0x0)
#define CPSW_CONTROL                 (0x4)
#define CPSW_SOFT_RESET              (0x8)
#define CPSW_STAT_PORT_EN            (0x14)
#define CPSW_PTYPE_REG               (0x18)

#define CPSW_CTL_P0_ENABLE           (1 << 2)
#define CPSW_CTL_REG_P0_TX_CRC_RM    (1 << 13)
#define CPSW_CTL_REG_P0_RX_PADBIT    (1 << 14)

/* host|gmac port*/

#define CPSW_MAX_BLKS                (0x8)  /* for PN only */
#define CPSW_BLK_CNT                 (0x10)
#define CPSW_FLOW_ID                 (0x8)  /* for P0 only */
#define CPSW_PORT_VLAN               (0x14)
#define CPSW_TX_PRI_MAP              (0x18)
#define CPSW_CPDMA_TX_PRI_MAP        (0x14)
#define CPDMA_RX_CH_MAP              (0x20)/* not match ?*/
#define CPSW_SL_SA_L0                (0x308)
#define CPSW_SL_SA_HI                (0x30c)

#define CPSW_GMII_EN                 (0x20)
#define GMAC_CTL_FULLDUPLEX          (0x1)
#define GAMC_CTL_GIG                 (0x80)
#define CPSW_TX_PRI_MAP_DFTL         (0x33221100)
#define CPDMA_TX_PRI_MAP_DFTL        (0x76543210)
#define CPSW_TX_IN_SEL_MSK           (0x3)
#define CPSW_TX_IN_SEL_SHIFT         (16)
#define CPSW_TX_FIFO_DUAL_EMAC       (0x1)
#define CPSW_EXT_EN                  (0x40000)

#define CPSW_DEFAULT_PORT_CFI        (0)
#define CPSW_DEFAULT_PORT_PRI        (0)
#define CPSW_PORT_VLAN_PROG(vlanId)  ((CPSW_DEFAULT_PORT_CFI << 12) | \
                                      (CPSW_DEFAULT_PORT_PRI << 13) | \
                                      (vlanId))

/* ALE register offset */

#define CPSW_ALE_TBLCTL              (0x20)
#define CPSW_ALE_WORD0               (0x3C)
#define CPSW_ALE_WORD1               (0x38)
#define CPSW_ALE_WORD2               (0x34)
#define CPSW_ALE_CONTROL             (0x08)
#define CPSW_ALE_UNKNOWN_VLAN        (0x18)
#define CPSW_ALE_PORTCTL(n)          (0x40 + (n) * 0x4)
#define CPSW_ALE_ENTRY_IDX_MASK      (0x3ff)
#define CPSW_ALE_ENTRY_NR            (1024)
#define CPSW_ALE_PORT_FW             (0x3)
#define CPSW_ALE_MULTICAST           (0x10000000)
#define CPSW_ALE_MULTICAST_FW        (0x40000000)
#define CPSW_ALE_SUPER               (0x2)
#define CPSW_ALE_ENTRY_MASK          (0x30000000)
#define CPSW_ALE_BLOCK               (0x2)
#define CPSW_ALE_SECURE              (0x1)
#define CPSW_ALE_UNICAST             (0x10000000)
#define CPSW_ALE_VLAN                (0x20000000)
#define CPSW_ALE_UNICAST_AGEABLE_NOT (0x0)
#define CPSW_ALE_UNICAST_AGEABLE     (0x4)
#define CPSW_ALE_BYPASS              (0x10)
#define CPSW_ALE_CTL_NO_LEARN        (0x10)
#define CPSW_ALE_CLR_TABLE           (0x40000000)
#define CPSW_ALE_EN_TABLE            (0x80000000)
#define CPSW_ALE_VLAN_AWARE          (0x4)
#define CPSW_ALE_WRITE               (0x80000000)
#define CPSW_ALE_ENRY_MASK           (0x3ff)

/* GMAC register offset and bit definitions */

#define CPSW_SL_MAC_CTL              (0x330)
#define CPSW_SL_MAC_SOFT_RESET       (0x338)
#define CPSW_SL_RX_MAXLEN            (0x024)
#define CPSW_SL_RX_PRI_MAP           (0x20)
#define CPSW_SL_RX_PRI_MAP_VAL       (0x12345678)

/* CPSW ALE register definitions */

#define CPSW_ALE_MCAST_FWD                (0)
#define CPSW_ALE_MCAST_BLOCK_LEARN_FWD    (1)
#define CPSW_ALE_MCAST_FWD_LEARN          (2)
#define CPSW_ALE_MCAST_FWD_2              (3)
#define CPSW_ALE_MCAST_SUPER              (1)
#define CPSW_ALE_MCAST_NOT_SUPER          (0)
#define CPSW_ALE_TYPE_FREE                (0)
#define CPSW_ALE_TYPE_ADDR                (1)
#define CPSW_ALE_TYPE_VLAN                (2)
#define CPSW_ALE_TYPE_VLAN_ADDR           (3)
#define CPSW_ALE_CTL_ENABLE               (1 << 31)
#define CPSW_ALE_CTL_RESET                (1 << 30)
#define CPSW_ALE_CTL_BYPASS               (1 << 4)

/* MACCONTROL register definitions */

#define CPSW_SL_MAC_CTL_TX_FLOW_EN   (0x1 << 4)
#define CPSW_SL_MAC_CTL_RX_FLOW_EN   (0x1 << 3)

#define CPSW_SL_MAC_CTL_DEF          (CPSW_GMII_EN |                 \
                                      CPSW_SL_MAC_CTL_TX_FLOW_EN |   \
                                      CPSW_SL_MAC_CTL_RX_FLOW_EN)

#define CPSW_CPPI_PKT_TYPE			 0x7

#undef BIT
#define BIT(n)                       ((UINT32)0x1 << (n))                                     

#define CPSW_MODE_SWITCH                    0    
#define CPSW_MODE_INDEPENDENT_PORT          1

#define CPDMA_TX_HOST_ERR_CODE(n) (((n) & 0xf00000) >> 20)
#define CPDMA_RX_HOST_ERR_CODE(n) (((n) & 0xf000)   >> 12)
#define CPDMA_TX_HOST_ERR_CHAN(n) (((n) & 0xf0000)  >> 16)
#define CPDMA_RX_HOST_ERR_CHAN(n) (((n) & 0xf00)    >> 8)

#define CPSW_DESC_INC(index, cnt) (index) = (((index) + 1) % (cnt))                                  
                                     
typedef struct cpsw_drv_ctrl    K3CPSW_DRV_CTRL;
typedef struct cpsw_host_ctrl   K3CPSW_SW_CTRL;

typedef struct
    {
    UINT32              word0;
    UINT32              word1;
    UINT8               word2;
    } CPSW_ALE_TBL;

typedef struct
    {
    /* rx hw statitics */

    UINT32              rxgood;
    UINT32              rxbroadcast;
    UINT32              rxmulticast;
    UINT32              rxpause;
    UINT32              rxcrcerros;
    UINT32              rxalignmenterrors;
    UINT32              rxoversized;
    UINT32              rxjabber;
    UINT32              rxundersized;
    UINT32              rxfrags; 
    UINT32              unused0;
    UINT32              unused1;    
    UINT32              rxoctets;

    /* tx hw statitics */

    UINT32              txgood;
    UINT32              txbroadcast;
    UINT32              txmulticast;
    UINT32              txpause;
    UINT32              txdefered;
    UINT32              txcollision;
    UINT32              txsinglecol;
    UINT32              txmulticol;
    UINT32              txexceesive;
    UINT32              txlatecol;
    UINT32              txunderrun;
    UINT32              txcariersense;
    UINT32              txoctets;

    UINT32              sz64octets;
    UINT32              sz65_127octets;
    UINT32              sz128_255octets;
    UINT32              sz256_511octets;
    UINT32              sz512_1023octets;
    UINT32              sz1024octets;

    UINT32              netoctets;
    UINT32              rxfifooverrun[3];
    } CPSW_STAT;
            
struct cpsw_drv_ctrl
    {
    END_OBJ             cpswEndObj;
    UINT8               mac[ETHER_ADDR_LEN];
    int                 mtu;
    void              * cookie;
    
    K3CPSW_SW_CTRL    * pSwCtrl;

    VXB_DEV_ID          pDev;
    VXB_RESOURCE      * intRes[3];
    
    UINT32              portOffset;
    UINT32              macSaveWay;

    UINT32              portIndex;
    UINT32              portVlan;

    M_BLK_ID            cpswRxblk[CPSW_DESC_CNT];
    M_BLK_ID            cpswTxblk[CPSW_DESC_CNT];

    VXB_DEV_ID          cpswMiiDev;
    END_MEDIALIST     * cpswMediaList;
    UINT32              cpswCurMedia;
    UINT32              cpswCurStatus;
    END_CAPABILITIES    cpswCaps;
   
    BOOL                cpswTxstall;
    BOOL                cpswPolling;
    M_BLK_ID            cpswPollbuf;

    M_BLK_ID            cpswRcvChain;
    M_BLK_ID          * pCpswRcvChainTail;
    M_BLK_ID            pMblkToFree;
 
    JOB_QUEUE_ID        cpswJobQueue;
    QJOB                cpswTxQJob;
    atomic_t            cpswTxIntPend;
    QJOB                cpswRxQJob;
    atomic_t            cpswRxIntPend;
    QJOB                cpswMiscQJob;
    atomic_t            cpswMiscIntPend;   

    END_ERR             cpswLastError;
    END_IFDRVCONF       cpswStatsConf;
    END_IFCOUNTERS      cpswStatsCounters;

    endCounter          cpswInDropped;
    };

struct cpsw_host_ctrl  
    {
    VXB_DEV_ID          pDev;
    void              * handle;    
    void              * regBase;
    UINT32              portOffset;
    UINT32              cpswOffset;
    UINT32              cpdmaHdpOffset;
    UINT32              statsOffset;
    UINT32              aleOffset;
    UINT32              mdioOffset;

    BOOL                workMode;
    int                 hostPortIndex;

    SEM_ID              cpswDevSem;
    K3CPSW_DRV_CTRL   * port[NR_MAC_PORTS];    

    CPSW_STAT           cpswStat; 
    };

/* accessor definitions */

#define CPSW_BAR(p)      p->regBase
#define CPSW_HANDLE(p)   p->handle

#define CSR_READ_4(pDrv, addr)          \
    vxbRead32(CPSW_HANDLE(pDrv),        \
            (UINT32 *)((char *)CPSW_BAR(pDrv) + (addr)))

#define CSR_WRITE_4(pDrv, addr, data)   \
    vxbWrite32(CPSW_HANDLE(pDrv),       \
            (UINT32 *)((char *)CPSW_BAR(pDrv) + (addr)), (data))

#define CSR_SET_BIT(pDrv, offset, val)  \
    CSR_WRITE_4(pDrv, offset, CSR_READ_4(pDrv, offset) | (val))

#define CSR_CLR_BIT(pDrv, offset, val)  \
    CSR_WRITE_4(pDrv, offset, CSR_READ_4(pDrv, offset) & ~(val))

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCvxbFdtTiK2Am65xCpswEndh */
