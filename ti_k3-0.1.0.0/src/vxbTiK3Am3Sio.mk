# Makefile - makefile for vxbTiK3Am3Sio.c

# Copyright (c) 2019 Wind River Systems, Inc.  
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
# 28apr19,mpc  added CORTEX series CPU type definitions (F11131)
# 21feb19,whe  created (VXWPG-114)
#
# DESCRIPTION
# This file contains the makefile macro values for the vxbTiK3Am3Sio driver
#
#

ifdef _WRS_CONFIG_FDT
OBJS_COMMON += vxbTiK3Am3Sio.o
endif
