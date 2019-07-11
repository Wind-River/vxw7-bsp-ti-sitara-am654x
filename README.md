VxWorks® 7 TI Sitara AM65x unsupported BSP
===
---

# Overview

This document describes the features of the ti_k3_am65x BSP/PSL, which is designed
to run on the TI AM65x EVM/IDK board. This is an ARM Cortex-A53 processor-based 
platform.

# Project License

Copyright (c) 2019 Wind River Systems, Inc.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1) Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3) Neither the name of Wind River Systems nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

# Prerequisite(s)

* You must have Wind River® VxWorks® 7 SR0620 released source code and
  development environment to support "TI Sitara AM65x unsupported BSP".

# Building and Using

### Preparation

IMPORTANT: Your existing VxWorks 7 installation may already contains the ti_k3_am65x 
BSP and ti_k3 PSL, please check the following installation location to see whether the
BSP and PSL existed already.

The ti_k3_am65x BSP is installed at:
```Bash
***installDir***/vxworks-7/pkgs_v2/unsupported/ti_k3_am65x/ti_k3_am65x-W.X.Y.Z
```
The ti_k3 PSL is installed at:
```Bash
***installDir***/vxworks-7/pkgs_v2/unsupported/ti_k3_am65x/ti_k3-W.X.Y.Z
```
If the installed version of BSP/PSL is the same or newer than the Open Source BSP published 
here, there's no need to download and install the Open Source BSP from GitHub again. 

If your existing VxWorks 7 installation contains older version of ti_k3_am65x BSP and ti_k3
PSL, it's recommended to download the latest code from GitHub and then install.

### Download

Download all layers from Github.
```Bash
git clone https://github.com/Wind-River/vxw7-bsp-ti-sitara-am654x.git
cd vxw7-bsp-ti-sitara-am654x
```

### Installation

There are two ways to install this BSP: inside existing VxWorks 7 installation or outside
existing VxWorks 7 installation.

#### Install into the source tree

All layers in this BSP go to their respective destination among the existing installation. 
The advantage is the BSP will always be accessible once you complete the installation. The 
disadvantage is you can't shut down this BSP unless you manually delete all the installed 
layers among the source tree.

Here's how it’s done:

```Bash
cp -r ti_k3_am65x-W.X.Y.Z ***installDir***/vxworks-7/pkgs_v2/unsupported/ti_k3_am65x/
cp -r ti_k3-W.X.Y.Z ***installDir***/vxworks-7/pkgs_v2/unsupported/ti_k3_am65x/
```

#### Install beside the source tree

All layers are copied in a place that's outside the default scanning folder, i.e., 
vxworks-7/pkgs_v2, and when launching the Development Shell or Workbench, the path containing 
this BSP is provided to the development environment. The advantage of this method is obvious, 
the package can be easily turn on or off, and the source code stays in one unified location 
external to the default installation, which makes it easier to manage.

Suppose all packages are in /home/ti_k3_am65x/, then do the following when launching wrenv
or Workbench:

```Bash
export WIND_LAYER_PATHS=/home/ti_k3_am65x
export WIND_BSP_PATHS=/home/ti_k3_am65x
```
then enter into the existing VxWorks 7 installation directory
```Bash
cd ***installDir***
```
if use the Development Shell
```Bash
./wrenv.sh -p vxworks-7
```
or Workbench
```Bash
./workbench-4/startWorkbench.sh
```

### Drivers

By now, the support of drivers in this Open Source BSP is as below:

| Hardware Interface | Controller | Driver/Component | Status |
| ------ | ------ | ------ | ------ |
GIC                 | on-chip     | DRV_INTCTLR_FDT_ARM_GIC        | SUPPORTED
UART                | on-chip     | DRV_SIO_AM3_TI_K3              | SUPPORTED
TIMER               | on-chip     | DRV_TIMER_FDT_OMAP3_TI_K3      | SUPPORTED
TIMER               | on-chip     | DRV_ARM_GEN_TIMER              | SUPPORTED
GPIO                | on-chip     | N/A                            | UNSUPPORTED
I2C                 | on-chip     | DRV_AM38XX_I2C_TI_K3           | SUPPORTED
EEPROM              | on-chip     | DRV_I2C_EEPROM                 | SUPPORTED
1000MB-ETHERNET     | on-chip     | DRV_END_FDT_TI_K3CPSW          | SUPPORTED
RTC                 | on-chip     | DRV_I2C_RTC_TI_K3              | SUPPORTED
CLOCK               | on-chip     | N/A                            | UNSUPPORTED
PINMUX              | on-chip     | N/A                            | UNSUPPORTED
PCIE                | on-chip     | N/A                            | UNSUPPORTED
SATA                | on-chip     | N/A                            | UNSUPPORTED
EDMA                | on-chip     | N/A                            | UNSUPPORTED
SPI:0               | on-chip     | DRV_AM35XX_SPI_TI_K3           | SUPPORTED
SPI FLASH           | on-board    | DRV_SPI_FLASH                  | SUPPORTED
WDT                 | on-chip     | N/A                            | UNSUPPORTED
USB                 | on-chip     | N/A                            | UNSUPPORTED
NAND FLASH          | on-chip     | N/A                            | UNSUPPORTED
SD/MMC              | on-chip     | N/A                            | UNSUPPORTED
DCAN                | on-chip     | N/A                            | UNSUPPORTED
LCD                 | on-chip     | N/A                            | UNSUPPORTED
MAILBOX             | on-chip     | N/A                            | UNSUPPORTED
QSPI                | on-chip     | N/A                            | UNSUPPORTED
SPINLOCK            | on-chip     | N/A                            | UNSUPPORTED
SGX                 | on-chip     | N/A                            | UNSUPPORTED
HDMI                | on-chip     | N/A                            | UNSUPPORTED

The detailed introduction of these drivers and usage can also be found in target.txt.

# Legal Notices

All product names, logos, and brands are property of their respective owners. All company, product 
and service names used in this software are for identification purposes only. Wind River and VxWorks 
are registered trademarks of Wind River Systems. 

Disclaimer of Warranty / No Support: Wind River does not provide support and maintenance services 
for this software, under Wind River’s standard Software Support and Maintenance Agreement or otherwise. 
Unless required by applicable law, Wind River provides the software (and each contributor provides its 
contribution) on an “AS IS” BASIS, WITHOUT WARRANTIES OF ANY KIND, either express or implied, including, 
without limitation, any warranties of TITLE, NONINFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A PARTICULAR 
PURPOSE. You are solely responsible for determining the appropriateness of using or redistributing the 
software and assume ay risks associated with your exercise of permissions under the license.

