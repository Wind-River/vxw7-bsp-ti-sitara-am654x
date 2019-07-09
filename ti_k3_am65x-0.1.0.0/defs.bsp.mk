# defs.bsp.mk - make variable definitions
#
# Copyright 2019 Wind River Systems, Inc.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met: 
# 
# 1) Redistributions of source code must retain the above copyright notice, 
# this list of conditions and the following disclaimer. 
# 
# 2) Redistributions in binary form must reproduce the above copyright notice, 
# this list of conditions and the following disclaimer in the documentation and/or 
# other materials provided with the distribution. 
# 
# 3) Neither the name of Wind River Systems nor the names of its contributors may be 
# used to endorse or promote products derived from this software without specific 
# prior written permission. 
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
#
# modification history
# --------------------
# 19feb19,whe  created (VXWPG-114)

# DESCRIPTION
# This file contains definitions for building VxWorks for the TI K3 AM65x
# boards.

# INTERNAL
# This file should only contain definitions specific to the BSP.  Any rules
# specific to this BSP should be placed in the rules file (rules.bsp.mk)
#

LOAD_ADDR = 0x$(LOCAL_MEM_PHYS_ADRS)

# build binary target

vxWorks.bin:    vxWorks
		- @ $(RM) $@
		$(EXTRACT_BIN) vxWorks $@
