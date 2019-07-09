/* sysLib.c - TI K3 AM65x system-dependent library */

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
This library provides board-specific routines for TI AM65x 
with cortex-A53 processor.

INCLUDE FILES: vxFdtLib.h

SEE ALSO:
\tb VxWorks Programmer's Guide: Configuration
*/

/* includes */

#include <vxWorks.h>
#include <boardLib.h>
#include <prjParams.h>
#include <am6xxx.h>
#include <vxFdtCpu.h>

#ifdef INCLUDE_SHOW_ROUTINES
#include <stdio.h>
#include <stdlib.h>
#include <vxFdtLib.h>

/* defines */

LOCAL void boardInfo (void);
IMPORT void cpuArmVerShow (void);

#endif /* INCLUDE_SHOW_ROUTINES */

LOCAL BOARD_FUNC_TBL sitaraFuncTbl = {
    /* .earlyInit  = */ am6xxxEarlyInit,
    /* .init       = */ am6xxxInit,
    /* .reset      = */ am6xxxReset,
    /* .model      = */ am6xxxModel,
    /* .usDelay    = */ am6xxxUsDelay,
#ifdef _WRS_CONFIG_SMP
    /* .cpuEn      = */ am6xxxCpuEnable,
    /* .cpuAvail   = */ vxFdtCpuAvail,
    /* .cpuDis     = */ NULL,
#endif /*_WRS_CONFIG_SMP*/ 
#ifdef INCLUDE_SHOW_ROUTINES
    /* .infoShow   = */ boardInfo,
#else
    /* .infoShow   = */ NULL,
#endif /* INCLUDE_SHOW_ROUTINES */
    /* .endMacGet  = */ NULL
};

LOCAL BOARD_DESC ti_am65x =
    {
    /* .uVer     = */ BOARD_DESC_VER_3_0,
    /* .pCompat  = */ "ti,am65x",
#ifdef INCLUDE_DEBUG_KPRINTF
    /* .uFlag    = */ BOARD_DESC_FLAG (BOARD_DESC_FLAG_DBG, 0),
#else
    /* .uFlag    = */ BOARD_DESC_FLAG (0, 0),
#endif
    /* .probe    = */ am6xxxProbe,
    /* .pFuncTbl = */ &sitaraFuncTbl
    };

BOARD_DEF (ti_am65x)

#ifdef INCLUDE_SHOW_ROUTINES

/*******************************************************************************
*
* socInfoShow - print SOC information
*
* This routine prints SOC information.
*
* RETURNS: N/A
* ERRNO: N/A
*/

LOCAL void socInfoShow(void)
    {
    }

/*******************************************************************************
*
* boardInfo - print board information
*
* This routine prints board information.
*
* RETURNS: N/A
* 
* ERRNO: N/A
*/

LOCAL void boardInfo (void)
    {
    printf ("%s, \n", am6xxxModel ());
    printf ("CPU: ");	
    socInfoShow();
    printf ("\n");
    cpuArmVerShow();
    printf ("\n");

    return;
    }
#endif /* INCLUDE_SHOW_ROUTINES */
