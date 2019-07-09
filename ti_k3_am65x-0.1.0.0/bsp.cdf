/* bsp.cdf - Board definition description file */

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

Bsp ti_k3_am65x {
    NAME                am65x board support package
    CPU                 CORTEX_A53 ARMARCH8A 
    ENDIAN              little
    MP_OPTIONS          UP SMP
    DATA_MODEL          LP64
    BUILD_SPECS         default
    BUILD_TARGETS       uVxWorks vxWorks vxWorks.bin
    FP                  hard soft
    REQUIRES            INCLUDE_KERNEL                  \
                        INCLUDE_CPU_INIT                \
                        INCLUDE_MEM_DESC                \
                        INCLUDE_BOARD_LIB_INIT          \
                        DRV_BUS_FDT_SIMPLE              \
                        INCLUDE_EARLY_MMU               \
                        DRV_INTCTLR_FDT_ARM_GIC_V3      \
                        DRV_ARM_GEN_TIMER               \
                        INCLUDE_TIMER_SYS               \
                        BOARD_SELECTION                 \
                        DRV_SIO_AM3_TI_K3               \
                        INCLUDE_CACHE_SUPPORT
}
