/* am6xxx.c - TI AM6xxx Processor Support Library */

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
19feb19,whe  created (VXWPG-114)
*/

/*
DESCRIPTION
This library provides processor-specific routines for the ti am6xxx series SOC.

INCLUDE FILES: sysLib.h string.h intLib.h taskLib.h vxLib.h muxLib.h

SEE ALSO:
\tb VxWorks Programmer's Guide: Configuration
\tb "AM65x/DRA80xM Processors Technical Reference Manual"
*/

/* includes */

#include <vxWorks.h>
#include <stdio.h>
#include <boardLib.h>
#include <sysLib.h>
#include <string.h>
#include <vxLib.h>
#include <cacheLib.h>
#include <pmapLib.h>
#include <vmLib.h>
#include <intLib.h>
#include <vxFdtLib.h>
#include <vxFdtCpu.h>
#include <arch/arm/mmuArmLib.h>
#include <private/vmLibP.h>
#include <hwif/vxBus.h>
#include <subsys/timer/vxbTimerLib.h>
#include <private/adrSpaceLibP.h>
#include <arch/arm/vxbFdtArmGicv3.h>
#include <am6xxx.h>
#ifdef _WRS_CONFIG_SMP
#   include <arch/arm/vxAtomicArchLib.h>
#   include <cpuset.h>
#   include <private/cpcLibP.h>
#endif /* _WRS_CONFIG_SMP */

/* defines */

#undef DEBUG_AM6xxx_ENABLE
#ifdef DEBUG_AM6xxx_ENABLE
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
#endif  /* DEBUG_AM6xxx_ENABLE */

/* UART */

#define REG_THR *(volatile unsigned char *)(debugSioBase + 0x00)
#define REG_LSR *(volatile unsigned char *)(debugSioBase + 0x14)
#define REG_MDR1 *(volatile unsigned char *)(debugSioBase + 0x20)

/* define for early debug out */

#define DEBUG_EARLY_PRINT

#define AM6XXX_SIO_COMPATIBLE_STR             "ti,am5-uart"

/* define for reboot */

#define SMC_CPU_RST                         0x84000009

#ifdef _WRS_CONFIG_SMP
#define SMC_CPU_ON                          0xC4000003
#endif /* _WRS_CONFIG_SMP */

/* imports */

IMPORT STATUS k3SecProxyInit ();
IMPORT void  k3EnableClk (UINT32, UINT8);
IMPORT void  armSysToMonitor(FUNCPTR core0ExitFunc, FUNCPTR pspExitFunc,
                             int startType);
IMPORT VIRT_ADDR mmuPtMemBase;

#ifdef _WRS_CONFIG_SMP
IMPORT void sysInit (void);
IMPORT STATUS armMonitorSpinRelease (UINT32);
IMPORT STATUS armMpCoreInit (UINT32, WIND_CPU_STATE *, FUNCPTR);
IMPORT UINT32 cpuIndexMap[];
#endif /* _WRS_CONFIG_SMP */

/* locals */

LOCAL VIRT_ADDR debugSioBase    = 0;
LOCAL VIRT_ADDR resetVirtAddr   = 0;

/* ARM core generic timer frequency (in Hz) */

LOCAL UINT32 am6xxxGenTimerFreq;

LOCAL void      am6xxxResetInit (void);
#ifdef DEBUG_EARLY_PRINT
LOCAL void      am6xxxEarlyDebugInit (void);
#endif /* DEBUG_EARLY_PRINT */

LOCAL void      am6xxxDebugInit (void);
LOCAL STATUS    am6xxxDbg (char * buffer, size_t len);
LOCAL void      am6xxxCore0Exit(UINT32 cpuId);
LOCAL UINT32    am6xxxCounterFreqGet  (void);

#ifdef _WRS_CONFIG_SMP
LOCAL void      am6xxxPspEntry (void);
LOCAL void      am6xxxPspExit(UINT32 cpuId);

/*******************************************************************************
*
* am65xSmc - perform SMC call
*
* This routine performs SMC call
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

_WRS_INLINE void am6xxxSmc
    (
    UINT64 * req
    )
    {
    __asm__ __volatile__ (
        "LDR    X0, %0\n"
        "LDR    X1, %1\n"
        "LDR    X2, %2\n"
        "LDR    X3, %3\n"
        "LDR    X4, %4\n"
        "LDR    X5, %5\n"
        "LDR    X6, %6\n"
        "SMC    #0\n"
        "STR    X0, %0\n"
        "STR    X1, %1\n"
        "STR    X2, %2\n"
        "STR    X3, %3\n"
        : "+m"(req[0]), "+m"(req[1]), "+m"(req[2]), "+m"(req[3])
        : "m"(req[4]), "m"(req[5]), "m"(req[6])
        : "x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7",
          "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15",
          "x16", "x17");
    }
#endif /* _WRS_CONFIG_SMP */

/*******************************************************************************
*
* am6xxxProbe - probe the board
*
* This routine probes the board with the DTB.
*
* RETURNS: TRUE when probed, otherwise FALSE
*
*\NOMANUAL
*
* ERRNO: N/A
*/

BOOL am6xxxProbe
    (
    char * boardCompatStr
    )
    {
    int    offset;
    
    /* The board compatibility string is in the root node */
    
    offset = vxFdtPathOffset ("/");
    
    if (offset < 0)
        {
        /* badly formed device tree - can't find root node */
    
        return FALSE;
        }
    
    /* Determine if the PSL supports this board */
    
    return (vxFdtNodeCheckCompatible (offset, boardCompatStr) == 0);
    }

/*******************************************************************************
*
* am6xxxConsoleGet - get the configuration for the debug console
*
* This routine searches the debug console, and finds out the register address
* and length.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL STATUS am6xxxConsoleGet
    (
    PHYS_ADDR * pPhyAddr,
    size_t *    pLen
    )
    {
    int offset;

    /* find the stdout in chosen node */

    offset = vxFdtStdoutGet ();

    /* try to use i.MX UART tty if failed */

    if (offset < 0)
        {
        offset = vxFdtNodeOffsetByCompatible (0, AM6XXX_SIO_COMPATIBLE_STR);
        if (offset < 0)
            return ERROR;
        }

    return vxFdtDefRegGet (offset, 0, pPhyAddr, pLen);
    }

#ifdef DEBUG_EARLY_PRINT
/*******************************************************************************
*
* am6xxxEarlyDebugInit - initialize debug console early
*
* This routine searches the debug console, and set up mmu entries for  the
* register space, then initializes the kprintf and kwrite hooks.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am6xxxEarlyDebugInit (void)
    {
    PHYS_ADDR       phyAddr;
    size_t          len;

    if (am6xxxConsoleGet (&phyAddr, &len) != OK)
        {
        DEBUG_MSG ("%s: get ti,am5-uart BAR failed.\n", __FUNCTION__);
        return;
        }

    debugSioBase = vxMmuEarlyRegMap (phyAddr, len);
    _func_kwrite = am6xxxDbg;
    if (REG_MDR1 != 0)
        {
        REG_MDR1 = 0;
        }

    DEBUG_MSG ("early debug init successfully.\n");
    }
#endif /* DEBUG_EARLY_PRINT */

/*******************************************************************************
*
* am6xxxEarlyInit - early init the board
*
* This routine does early init.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void am6xxxEarlyInit (void)
    {
    UINT    uState = MMU_ATTR_VALID | MMU_ATTR_SUP_RWX |
                     MMU_ATTR_CACHE_COPYBACK
#ifdef _WRS_CONFIG_SMP
                     | MMU_ATTR_CACHE_COHERENCY
#endif /* _WRS_CONFIG_SMP */
                     ;

#ifdef DEBUG_EARLY_PRINT
    if (boardFlagCheck (BOARD_DESC_FLAG_DBG, FALSE))
        {
        am6xxxEarlyDebugInit ();
        }
#endif /* DEBUG_EARLY_PRINT */

    /* Initialise the generic timer frequency */

    am6xxxGenTimerFreq = am6xxxCounterFreqGet();

    sysPhysMemDescNumEnt = vxFdtPhysMemInfoGet (sysPhysMemDesc,
                                                sysPhysMemDescNumEnt,
                                                uState);

    if (sysPhysMemDescNumEnt == 0)
        {
        DEBUG_MSG ("sysPhysMemDescNumEnt == 0\n");
        }

    mmuPtMemBase = KERNEL_SYS_MEM_RGN_BASE - sysPhysMemDesc[0].physicalAddr;
    }

/*******************************************************************************
*
* am6xxxModel - return the model name of the CPU board
*
* This routine returns the model name of the CPU board.
*
* RETURNS: A pointer to a string identifying the board and CPU.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

char * am6xxxModel (void)
    {
    int offset;
    void *  pValue;

    if ((offset = vxFdtPathOffset ("/")) >= 0)
        {
        pValue = (void *)vxFdtPropGet (offset, "model", NULL);
        if (pValue != NULL)
            {
            return (char *)pValue;
            }
        }

    return("unknown TI AM6 board");
    }

/*******************************************************************************
*
* k3ClksInit - initialize power and clocks
*
* This routine initializes the am65xx powers and clocks
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void k3ClksInit ()
    {
    k3EnableClk (137, 1); /* AM6_DEV_MCSPI0 DEV_MCSPI0_BUS_CLKSPIREF_CLK */
    }

/*******************************************************************************
*
* am6xxxDevInit - init device
*
* This routine initializes various feature of the am6xxx soc.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am6xxxDevInit (void)
    {
    STATUS ret;
    am6xxxResetInit ();
    ret = k3SecProxyInit ();
    if (ret == ERROR)
        {
        DEBUG_MSG ("k3SecProxyInit fail!\n");
        return ;
        }
    k3ClksInit ();
    }

/*******************************************************************************
*
* am6xxxInit - initialize the system hardware
*
* This routine initializes various feature of the am6xxx soc. It sets up
* the control registers, initializes various devices if they are present.
*
* NOTE: This routine should not be called directly by the user.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

void am6xxxInit (void)
    {
    if (boardFlagCheck (BOARD_DESC_FLAG_DBG, FALSE))
        {
        am6xxxDebugInit ();

        DEBUG_MSG ("installed kprintf\n");
        }

    am6xxxDevInit ();

    DEBUG_MSG ("end of bspInit\n");
    }

/*******************************************************************************
*
* am6xxxResetInit - initialize the SOC reset Module
*
* NOTE: This routine will read the physical address & reset control value of
* PPRCM_PRM_RSTCTRL Registers from DTB, then map the registers into virtual
* space, the virtual address of this register will be saved in resetVirtAddr,
* am6xxxReset() will use these information to reset the whole SOC.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am6xxxResetInit (void)
    {
    int         offset, len;
    UINT32 *    pProp;
    UINT32      resetPhyAddr;
    UINT32      mmuAttr;
    void *      virtAddr;

    offset = vxFdtPathOffset("/soc");
    pProp = (UINT32*)vxFdtPropGet (offset, "reset_control", (int*)(&len));
    if (len == 0 || pProp == NULL)
        {
        DEBUG_MSG ("Did not find reset_control from DTB\n");
        return;
        }

    resetPhyAddr = vxFdt32ToCpu (pProp[0]);

    mmuAttr = MMU_ATTR_SUP_RW | MMU_ATTR_CACHE_OFF | MMU_ATTR_CACHE_GUARDED;
    virtAddr = pmapGlobalMap ((PHYS_ADDR) resetPhyAddr,
                              (size_t) PAGE_SIZE, mmuAttr);
    if (virtAddr == PMAP_FAILED)
        {
        resetVirtAddr = 0;
        DEBUG_MSG ("pmapGlobalMap resetPhyAddr error\n");
        }
    else
        {
        resetVirtAddr = (UINT64)virtAddr;
        }

    return;
    }

/*******************************************************************************
*
* __inline__am6xxxGetCntFreq - get the ARM core counter frequency
*
* This routine gets the ARM core counter frequency from the CNTFRQ system
* register.
*
* RETURNS: the ARM core counter frequency
*/


UINT32 __inline__am6xxxGetCntFreq (void)
    {
    UINT64  tempVal;
    __asm__ __volatile__ ("MRS %0, CNTFRQ_EL0"
                          : "=r" (tempVal)
                          : );
    return (UINT32)tempVal;
    }

/*******************************************************************************
*
* am6xxxCounterFreqGet - get free-running counter frequency
*
* This function gets the ARM core generic timer counter frequency.
*
* RETURNS: counter frequency (in Hz)
*/

LOCAL UINT32 am6xxxCounterFreqGet  (void)
    {
    return __inline__am6xxxGetCntFreq ();
    }

/*******************************************************************************
*
* __inline__am6xxxGetVirtTimerCnt - get the ARM core counter value
*
* This routine gets the ARM core virtual counter frequency from the
* CNTVCT system register.
*
* RETURNS: the ARM core counter frequency
*/

UINT64 __inline__am6xxxGetVirtTimerCnt (void)
    {
    UINT64  tempVal;
    __asm__ __volatile__ ("MRS %0, CNTVCT_EL0"
                          : "=r" (tempVal)
                          : );
    return tempVal;
    }

/*******************************************************************************
*
* am6xxxCounterValueGet - get free-running counter value
*
* This function gets the ARM core generic timer virtual counter value.
*
* The virtual counter rather than the physical counter is used because the
* the virtual counter is always accessible to software running at non-secure
* EL0. Access to the physical counter depends on the configuration of
* system register CNTHCTL so access is not guaranteed.
*
* RETURNS: Counter value
*/

LOCAL UINT64 am6xxxCounterValueGet (void)
    {
    /*
     * Reads from CNTVCT can occur speculatively and out of order relative
     * to other instructions. The ISB ensures that the CNTVCT is read after
     * preceding instructions.
     */

    WRS_ASM ("ISB");

    return __inline__am6xxxGetVirtTimerCnt();
    }

/*******************************************************************************
*
* am6xxxUsDelay - delay the specified amount of time (in microseconds)
*
* This routine delays for approximately the requested microseconds. The accuracy
* of the delay increases as the requested delay increases due to a certain
* amount of overhead.
*
* NOTE:  This routine will not relinquish the CPU, as it is meant to perform a
* busy loop delay
*
* RETURNS: N/A
*
* ERRNO: N/A
*
*\NOMANUAL
*/

void am6xxxUsDelay
    (
    int delayUs   /* microseconds */
    )
    {
    volatile UINT64 oldTicks;
    volatile UINT64 newTicks;
    volatile UINT64 delayTicks;
    volatile UINT64 elapsedTicks = 0;

    /* Confirm delay is non-zero number of microseconds */

    if (delayUs <= 0)
        {
        return;
        }

    /* Convert delay period to counter ticks */

    delayTicks = ((am6xxxGenTimerFreq * (UINT64)delayUs) + (1000000 - 1)) /
                 1000000;

    /*
     * Repeatedly read the counter until the elapsed number
     * of ticks is greater than the delay period.
     */

    oldTicks = am6xxxCounterValueGet ();

    while (elapsedTicks <= delayTicks)
        {
        newTicks = am6xxxCounterValueGet ();

        if (newTicks >= oldTicks)
            {
            elapsedTicks += newTicks - oldTicks;
            }
        else
            {
            elapsedTicks += 0xffffffffffffffffUL -
                            oldTicks + newTicks + 1;
            }

        oldTicks = newTicks;
        }
    }

/*******************************************************************************
*
* am6xxxCore0Exit - shutdown routine
*
* This routine disables the external L2 and GIC before system shutdown.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am6xxxCore0Exit
    (
    UINT32  apCore
    )
    {
    VXB_DEV_ID  gicDev = NULL;

    (void) intCpuLock ();
    gicDev = vxbDevAcquireByName("armgic-v3",0);
    if (gicDev == NULL)
        return;
    vxbDevRelease (gicDev);

    (void) vxbArmGicDistShutdown(gicDev);
    (void) vxbArmGicRdistCpuShutdown(gicDev);
    }

/*******************************************************************************
*
* am6xxxReset - reset the board
*
* This routine resets the board.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
*\NOMANUAL
*/

void am6xxxReset
    (
    int startType
    )
    {
    UINT64      smcReq [7] = {0};
    smcReq[0] = (UINT64) SMC_CPU_RST;

    if ((sysWarmBootFunc == NULL) || (startType == BOOT_CLEAR))
        goto cold;

#ifdef _WRS_CONFIG_SMP
    armSysToMonitor ((FUNCPTR)am6xxxCore0Exit, (FUNCPTR)am6xxxPspExit,
                     startType);
#else
    armSysToMonitor ((FUNCPTR)am6xxxCore0Exit, NULL, startType);
#endif

cold:

    DEBUG_MSG ("Try cold boot\n");

    /* now reset */

    if (resetVirtAddr != 0)
        {
        /*
         * Asserts a global COLD software reset through PRM_RSTCTRL
         * Register.
         */

        sysWrite32 (resetVirtAddr, 0x2);
        }

    /* we should never fall through here */

    DEBUG_MSG ("am6xxx failed, startType=0x%x.\n", startType);
    }

/*******************************************************************************
*
* am6xxxDebugInit - initialize debug console
*
* This routine searches the debug console, and sets up MMU entries for the
* register space, then initializes the krpintf and kwrite hooks.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am6xxxDebugInit (void)
    {
    void *      virtAddr = 0;
    PHYS_ADDR   phyAddr;
    size_t      len;

    if (am6xxxConsoleGet (&phyAddr, &len) != OK)
        {
        DEBUG_MSG ("%s: get ti,am5-uart BAR failed.\n", __FUNCTION__);
        return;
        }

    virtAddr = pmapGlobalMap (phyAddr, len, VXB_REG_MAP_MMU_ATTR);
    if (virtAddr == PMAP_FAILED)
        {
        return;
        }    
    debugSioBase  =  (VIRT_ADDR)virtAddr;
    _func_kwrite = am6xxxDbg;
    
    if (REG_MDR1 != 0)
        {
        REG_MDR1 = 0;
        }

    DEBUG_MSG ("Debug init successfully\n");
    }

/*******************************************************************************
*
* am6xxxDbg - ouput buffer on debug console
*
* This routine ouputs the content of the buffer on the debug console.
*
* RETURNS: OK always
*
* ERRNO: N/A
*/

LOCAL STATUS am6xxxDbg
    (
    char * buffer,
    size_t len
    )
    {
    /* if no one set the value means we are in no mmu env. */

    if ((buffer == NULL) || (debugSioBase == 0))
        return ERROR;

    while (len--)
        {
        while ((REG_LSR & 0x20) == 0);
        REG_THR = *buffer;
        if (*buffer == '\n')
            {
            while ((REG_LSR & 0x20) == 0);
            REG_THR = '\r';
            }
        buffer++;
        }

    return OK;
    }

#ifdef _WRS_CONFIG_SMP

/*******************************************************************************
*
* am6xxxPspEntry - secondary core entry callback function
*
* This routine is the callback function called before secondary cores enter
* kernel. Here is a good place to set the interrupt control to enable secondary
* cores to receive IPI from core 0.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am6xxxPspEntry (void)
    {
    VXB_DEV_ID  gicDev = NULL;

    gicDev = vxbDevAcquireByName("armgic-v3",0);
    if (gicDev == NULL)
        return;
    vxbDevRelease (gicDev);

    (void) vxbArmGicRedistInit (gicDev);
    (void) vxbArmGicCpuInit (gicDev);
    }

/*******************************************************************************
*
* am6xxxPspExit - secondary cores exit function
*
* This routine is the exit function for secondary cores. It deactivates the
* active interrupt, clears all remaining pending interrupt and then disables the
* interrupt controller.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL void am6xxxPspExit
    (
    UINT32 cpuId
    )
    {
    VXB_DEV_ID  gicDev = NULL;

    (void) intCpuLock ();
    gicDev = vxbDevAcquireByName("armgic-v3",0);
    if (gicDev == NULL)
        return;
    vxbDevRelease (gicDev);

    (void) vxbArmGicRdistCpuShutdown (gicDev);
    }

/*******************************************************************************
*
* am6xxxCpuEnable - enable a multi core CPU
*
* This routine brings a multi core CPU out of reset
*
* RETURNS: OK or ERROR
*
* ERRNO: N/A
*
* \NOMANUAL
*/

STATUS am6xxxCpuEnable
    (
    UINT32           cpuId,
    WIND_CPU_STATE * cpuState
    )
    {
    UINT64      smcReq [7] = {0};

    if (cpuId >= vxFdtCpuAvail ())
        return ERROR;

    if (ERROR == armMpCoreInit (cpuId, cpuState, (FUNCPTR) am6xxxPspEntry))
        {
        return ERROR;
        }

    if (sysStartType == BOOT_COLD)
        {
        PHYS_ADDR        physAddr;
        STATUS           stat;

        stat = vmTranslate (NULL, (VIRT_ADDR)sysInit, &physAddr);
        if (stat == ERROR)
            {
            DEBUG_MSG ("am6xxxCpuEnable: vmTranslate failed for cpuId:%d\n",
                       cpuId);
            return ERROR;
            }

        smcReq[0] = (UINT64) SMC_CPU_ON;
        smcReq[1] = (UINT64) cpuIndexMap[cpuId];
        smcReq[2] = (UINT64) physAddr;
        smcReq[3] = 0;
        am6xxxSmc (smcReq);

        return OK;
        }
    else
        {
        return armMonitorSpinRelease (cpuId);
        }
    }
#endif /* _WRS_CONFIG_SMP */
