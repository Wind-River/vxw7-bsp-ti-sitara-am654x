/* am6xxx.h - header files for TI AM6xxx processor */

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
15feb19,whe  created (VXWPG-114)
*/

#ifndef __INCam6xxxh
#define __INCam6xxxh

#ifdef __cplusplus
extern "C" {
#endif

/* Handy sizes */

#define SZ_4K                       (0x00001000)
#define SZ_8K                       (0x00002000)
#define SZ_16K                      (0x00004000)
#define SZ_32K                      (0x00008000)
#define SZ_64K                      (0x00010000)
#define SZ_128K                     (0x00020000)
#define SZ_256K                     (0x00040000)
#define SZ_512K                     (0x00080000)

#define SZ_1M                       (0x00100000)
#define SZ_2M                       (0x00200000)
#define SZ_4M                       (0x00400000)
#define SZ_8M                       (0x00800000)
#define SZ_16M                      (0x01000000)
#define SZ_32M                      (0x02000000)
#define SZ_64M                      (0x04000000)
#define SZ_128M                     (0x08000000)
#define SZ_256M                     (0x10000000)
#define SZ_512M                     (0x20000000)

#define SZ_1G                       (0x40000000)
#define SZ_2G                       (0x80000000)

#ifndef BIT
#define BIT(x)                      (1 << (x))
#endif

/* GIC */

#define GICD_CTRL                   (0x0)
#define GICD_ISENABLER0             (0x100)
#define GICD_ICENABLER0             (0x180)
#define GICD_ISPENDR0               (0x200)
#define GICD_ICPENDR0               (0x280)
#define GICD_ISACTIVER0             (0x300)
#define GICD_ICACTIVER0             (0x380)

#define GICC_CTRL                   (0x0)
#define GICC_PMR                    (0x4)
#define GICC_EOIR                   (0x10)

#define INT_LVL_MPCORE_CPC          (0)

#define SGI_INT_MAX                 (16)
#define ARM_GIC_IPI_COUNT           (16)    /* MPCore IPI count         */
#define SPI_START_INT_NUM           (32)    /* SPI start at ID32        */
#define PPI_START_INT_NUM           (16)    /* PPI start at ID16        */
#define GIC_INT_MAX_NUM             (1020)  /* GIC max interrupts count */

/* accessors */

#ifndef sysRead32
#define sysRead32(a)             (*(volatile unsigned int *)(a))
#endif /* sysRead32 */
#ifndef sysWrite32
#define sysWrite32(a, v)         (*(volatile unsigned int *)(a) = (v))
#endif /* sysWrite32 */
#ifndef sysRead16
#define sysRead16(a)             (*(volatile unsigned short *)(a))
#endif /* sysRead16 */
#ifndef sysWrite16
#define sysWrite16(a, v)         (*(volatile unsigned short *)(a) = (v))
#endif /* sysWrite16 */
#ifndef sysRead8
#define sysRead8(a)              (*(volatile unsigned char *)(a))
#endif /* sysRead8 */
#ifndef sysWrite8
#define sysWrite8(a, v)          (*(volatile unsigned char *)(a) = (v))
#endif /* sysWrite8 */

/* function declarations */

IMPORT BOOL   am6xxxProbe     (char * compat);
IMPORT void   am6xxxInit      (void);
IMPORT void   am6xxxEarlyInit (void);
IMPORT char * am6xxxModel     (void);
IMPORT void   am6xxxUsDelay   (int us);
IMPORT void   am6xxxReset     (int startType);
#ifdef _WRS_CONFIG_SMP
IMPORT STATUS  am6xxxCpuEnable(UINT32 cpuId, WIND_CPU_STATE * cpuState);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCam6xxxh */
