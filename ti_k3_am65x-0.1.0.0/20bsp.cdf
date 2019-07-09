/* 20bsp.cdf - BSP component description file */
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

Component DRV_END_FDT_TI_K3CPSW {
    REQUIRES += INCLUDE_TI_DP_PHY
    INCLUDE_WHEN  INCLUDE_END
}

Parameter VX_SMP_NUM_CPUS {
    NAME        Number of CPUS to be enabled for SMP
    TYPE        UINT
    DEFAULT     4
}

Parameter LOCAL_MEM_PHYS_ADRS {
    NAME        local memory physical base address
    SYNOPSIS    Base physical address. For LPAE, alignment of address must be 2MB.
    DEFAULT     0x80000000
}

Parameter LOCAL_MEM_SIZE {
    NAME        local memory size
    SYNOPSIS    Fixed (static) memory size
    DEFAULT     0x80000000
}

Parameter DTS_FILE {
    NAME        dts file name to be used
    SYNOPSIS    This parameter specifies the DTS file to be used.
    DEFAULT     am65x.dts
}

Parameter RAM_LOW_ADRS {
    NAME        Runtime kernel entry address
    DEFAULT     0xffffffff80100000
}

Parameter RAM_HIGH_ADRS {
    NAME        Runtime kernel high address
    DEFAULT     0xffffffff81000000
}

Parameter LOCAL_MEM_LOCAL_ADRS {
    NAME        System memory start address
    SYNOPSIS    This parameter specifies the virtual base address for memory.
    DEFAULT     0xffffffff80000000
}

Parameter DTB_RELOC_ADDR {
    NAME        Device tree blob (DTB) relocation address
    SYNOPSIS    The DTB must be relocated to a safe address so that it is not \
                overwritten. This address should be below RAM_LOW_ADRS and the \
                reserved start region. This address must allow sufficient space\
                for the entire DTB.
    TYPE        void *
    DEFAULT     (LOCAL_MEM_LOCAL_ADRS + 0x10000)
}

Parameter DTB_MAX_LEN {
    NAME        DTB maximum length
    SYNOPSIS    DTB(Device Tree Blob) need to be relocated to one safe \
                address to avoid be overwritten, so it should be below \
                RAM_LOW_ADRS and the reserved start region, and enough \
                for the entire DTB.
    TYPE        int
    DEFAULT     0x20000
}

Parameter KERNEL_LOAD_ADRS {
    NAME        Runtime kernel load address
    DEFAULT     0x80100000
}

Parameter IMA_SIZE {
    NAME        Initial mapped area (IMA) size
    SYNOPSIS    IMA is mapped to the MMU during the early initialization phase \
                (before usrMmuInit()). Therefore, the size must be large enough\
                to accommodate the entire VxWorks kernel image.
    DEFAULT     0x10000000
}

Parameter ISR_STACK_SIZE {
    NAME        ISR stack size
    SYNOPSIS    This parameter defines the stack size in bytes for the \
                interrupt service routine (ISR).
    DEFAULT     0x2000
}


Parameter DEFAULT_BOOT_LINE {
    NAME        default boot line
    SYNOPSIS    Default boot line string
    TYPE        string
    DEFAULT     "cpsw(0,0)host:vxWorks h=192.168.1.1 e=192.168.1.100 u=target pw=vxTarget f=0x0"
}

Parameter CONSOLE_TTY {
    NAME        console serial port
    DEFAULT     0
}

Parameter NUM_TTY {
     NAME       number of serial ports
     DEFAULT    1
}

Parameter CONSOLE_BAUD_RATE {
    NAME        baud rate of console port
    DEFAULT     115200
}

Profile BSP_DEFAULT {
    PROFILES    PROFILE_OS_DEFAULT
}