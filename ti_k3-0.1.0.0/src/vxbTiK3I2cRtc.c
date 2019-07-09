/* vxbTiK3I2cRtc.c - VxBus I2C RTC driver */

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
02jul19,jff  created (VXWPG-114)
*/

/*
DESCRIPTION
This is the driver for the I2C serial Real-Time Clock.

SUPPORT
This driver can support all Maxim/Dallas I2C RTC device. Other I2C real-time
clock devices with identical registers should also be able to use this driver.

Most RTC devices use 7 distinct registers to define a date and time, addressable
and address map table are as follows:

\cs
REGISTERS                    ADDRESS   FUNCTION   RANGE
#define RTC_SEC_REG             0x0    Seconds    00-59
#define RTC_MIN_REG             0x1    Minutes    00-59
#define RTC_HOUR_REG            0x2    Hours      00-23
#define RTC_DAY_REG             0x3    Day        01-07
#define RTC_DATE_REG            0x4    Date       01-31
#define RTC_MONTH_REG           0x5    Month      01-12
#define RTC_YEAR_REG            0x6    Year       00-99
\ce

\ts
Address Map
----------------------------------
Address  | FUNCTION        | RANGE
---------|-----------------|------
00h      | Seconds         | 00-59
01h      | Minutes         | 00-59
02h      | Hours           | 00-23
03h      | Day             | 1-7
04h      | Date            | 1-31
05h      | Month/Century   | 01-12
06h      | Year            | 00-99
07h      | Alarm 1 Seconds | 00-59
08h      | Alarm 1 Minutes | 00-59
09h      | Alarm 1 Hours   | 00-23
0Ah      | Alarm 1 Date    | 1-31
0Bh      | Alarm 2 Minutes | 00-59
0Ch      | Alarm 2 Hours   | 00-23
0Dh      | Alarm 2 Date    | 1-31
0Eh      | Control         | -
0Fh      | Status          | -
10h      | Aging Offset    | -
11h      | MSB of Temp     | -
12h      | LSB of Temp     | -
13h      | Not used        | Reserved for test
14h-0FFh | SRAM            | 00h-0FFh
\te

To get or set time, one independently accesses the appropriate register to get
or set second, minute, hour, day, date, month and year.

In contrast, some RTC devices, use 4 cascaded registers to define date and time,
addressable and address map tables are as follows:

\cs
#define RTC_CNT_BYTE0_ADDR      0x00
#define RTC_CNT_BYTE1_ADDR      0x01
#define RTC_CNT_BYTE2_ADDR      0x02
#define RTC_CNT_BYTE3_ADDR      0x03
\ce

\ts
Address Map
--------- ------------------------|-----------------------
Address  | FUNCTION               | BIT7 - BIT0
---------|------------------------|-----------------------
00h      | Time-of-Day Counter    | TOD Counter Byte 0
01h      | Time-of-Day Counter    | TOD Counter Byte 1
02h      | Time-of-Day Counter    | TOD Counter Byte 2
03h      | Time-of-Day Counter    | TOD Counter Byte 3
04h      | Watchdog/Alarm Counter | WD/ALM Counter Byte 0
05h      | Watchdog/Alarm Counter | WD/ALM Counter Byte 1
06h      | Watchdog/Alarm Counter | WD/ALM Counter Byte 2
07h      | Control                | -
08h      | Status                 | -
09h      | Trickle Charger        | -
\te

The programmer must manually convert the count of whole seconds into a calendar
time.

The driver will automatically use the appropriate register model based on the
device name when adding device resource in the BSP, and this driver provides
vxbI2cRtcTimeGet()/vxbRtcTimeSet() to access the RTC chip.

To add the driver to your vxWorks image, add the following component to the
kernel configuration.

\cs
prj vip component add DRV_I2C_RTC_TI_K3
\ce

\cs
#define DRV_I2C_RTC
\ce

If the RTC type is not supported in rtcTable, but is similar in procedures with
one item in rtcTable, the driver also can be used.

RTC DATE AND TIME

This driver maintains the date and time in an internal structure(struct tm).

struct tm (in time.h - POSIX time header)
    {
    int tm_sec;     seconds after the minute     - [0, 59]
    int tm_min;     minutes after the hour       - [0, 59]
    int tm_hour;    hours after midnight         - [0, 23]
    int tm_mday;    day of the month             - [1, 31]
    int tm_mon;     months since January         - [0, 11]
    int tm_year;    years since 1900               
    int tm_wday;    days since Sunday            - [0, 6]
    int tm_yday;    days since January 1         - [0, 365]
    int tm_isdst;   Daylight Saving Time flag
    };

Below is an example shows how to set and get RTC: (JAN 01 23:59:59 2013)

\cs
   struct tm time_spec;
   memset(&time_spec, 0, sizeof(time_spec));

   time_spec.tm_year = 113;
   time_spec.tm_mon  = 0;
   time_spec.tm_mday = 1;
   time_spec.tm_hour = 23;
   time_spec.tm_min  = 59;
   time_spec.tm_sec  = 59;

   if (vxbI2cRtcTimeSet(pDev, &time_spec) != OK)
       {
       return ERROR;
       }

    vxbI2cRtcShow(pDev, 100);
\ce

Note: 
This implementation supports to set the range of years from 2000 to 2099.
The interrupt controller vector assigned to RTC alarm interrupt pin should be
configured as edge sensitive, level sensitive is not recommend.

This driver is bound to device tree, and the device tree node must specify
below properties:

\is

\i <compatible>
This property specifies the name of the clock controller driver. It must be
the type string in i2cEepromMatch[].

\i <reg>
This property specifies the I2C address of the device.

\i <data-scl-frequency>
This property specifies the SCL clock frequency of the data access in Hz.
Generally, it should be 100000, known as standard mode, or 400000, know as fast
mode. If the parameter is set as 0 or doesn't exist, the fast mode will be used.

\i <interrupts>
This property specifies interrupt vector of the RTC alarm interrupt that are 
generated by this device.

\i <interrupt-parent>
This property is available to define an interrupt parent. if it is missing from
a device, it's interrupt parent is assumed to be its device tree parent.

\i <int-gpio-pin>
This property specifies the GPIO pin which is connected to RTC alarm interrupt. 
This field cannot be set when <interrupts> is set.

\i <int-gpio-polarity>
This property specifies the GPIO interrupt polarity. 1 stands for high level, 
2 stands for low level.

\ie

An example of device node is shown below:

\cs
    rtc@68 {
        compatible = "dallas,ds3232";
        reg = <0x68>;
        data-scl-frequency = <100000>;
        interrupts = <38 0 4>;
        interrupt-parent = <&intc>;
        };
    
    rtc@69 {
        compatible = "dallas,ds3232";
        reg = <0x69>;
        int-gpio-pin = <15>;
        int-gpio-polarity = <2>;
        };
\ce

INCLUDE FILES: vxBus.h vxbI2cLib.h vxbRtcLib.h
*/

/* includes */

#include <vxWorks.h>
#include <semLib.h>
#include <stdlib.h>
#include <string.h>
#include <logLib.h>
#include <stdio.h>
#include <time.h>
#include <intLib.h>
#include <taskLib.h>
#include <private/timeP.h>
#include <hwif/vxBus.h>
#include <subsys/int/vxbIntLib.h>
#include <subsys/gpio/vxbGpioLib.h>
#include <hwif/buslib/vxbI2cLib.h>
#include "vxbTiK3I2cRtc.h"
#include <hwif/drv/resource/vxbRtcLib.h>

/* define */

/*
 * The definition of the year and month are different from struct tm and RTC,
 * so here add two "bias" values are _added_ to the struct tm fields
 * (found in time.h) when converting to the values that the timekeeper requires.
 * These values are correspondingly _subtracted_ from the timekeeper native 
 * values when loading into a struct tm field on a read.
 */

#define BIAS_YEAR                 (CENTURY)
#define BIAS_MONTH                1
#define BIAS_WDAY                 1

#define BIASED_YEAR(yr)           ((yr) - BIAS_YEAR)
#define UNBIASED_YEAR(yr)         ((yr) + BIAS_YEAR)

/* debug macro */
#undef  I2C_RTC_CTRL_DEBUG
#ifdef  I2C_RTC_CTRL_DEBUG

IMPORT FUNCPTR _func_kprintf;
#define I2CRTC_MSG(X0, X1, X2, X3, X4, X5, X6)         \
    if (_func_kprintf != NULL) \
        _func_kprintf (X0, (int)X1, (int)X2, (int)X3, (int)X4, (int)X5, (int)X6)
#else
#define I2CRTC_MSG(fmt,a,b,c,d,e,f)
#endif  /* I2C_RTC_CTRL_DEBUG */

/* forward declarations */

LOCAL STATUS vxbI2cRtcTimeGet (VXB_DEV_ID pDev, struct tm * rtcTime);
LOCAL STATUS vxbI2cRtcTimeSet (VXB_DEV_ID pDev, struct tm * rtcTime);
LOCAL STATUS vxbI2cRtcAlarmSet
    (
    VXB_DEV_ID     pDev,
    UINT8          unit,
    struct tm *    alarmTime,
    RTC_ALARM_FUNC rtcAlarmFunc,
    void *         prtcAlarmArg
    );
LOCAL STATUS vxbI2cRtcAlarmGet
    (
    VXB_DEV_ID    pDev,
    UINT8         unit,
    struct tm *   alarmTime
    );
LOCAL void vxbI2cRtcAlarmMon
    (
    VXB_DEV_ID    pDev
    );
LOCAL void vxbI2cRtcAlarmHandler
    (
    VXB_DEV_ID    pDev
    );
LOCAL STATUS vxbI2cRtcAlarmIsr
    (
    VXB_DEV_ID pDev
    );


/* I2C_RTC_CTRL device struct */

typedef struct i2cRtcCtrlInfo {
    UINT8 rtcReg;
    UINT8 rtcRegNum; /* Register model */
    UINT8 alarm1;    /* Alarm 1 */
    UINT8 alarm2;    /* Alarm 2 */
    UINT8 temp;      /* Temperature */
    UINT8 sram;      /* Sram Base */
    UINT8 sramSize;  /* Sram size */
}I2C_RTC_CTRL_INFO;

typedef struct i2c_rtc_ctrl{
    VXB_DEV_ID          pDev;
    I2C_RTC_CTRL_INFO   i2cRtcInfo;
    UINT16              i2cAddr;
    VXB_RESOURCE *      intRes;
    UINT8 *             writebuf;
    UINT32              dataScl;        /* SCL clock frequency for data access in Hz */
    RTC_ALARM_FUNC      rtcAlarm1Func;  /* alarm1 callback routine */
    RTC_ALARM_FUNC      rtcAlarm2Func;  /* alarm2 callback routine */
    void *              rtcAlarm1Arg;   /* parameter of alarm1 callback */
    void *              rtcAlarm2Arg;   /* parameter of alarm2 callback */
    UINT32              intPin;
    SEM_ID              irqSem;
    BOOL                gpioInt;
}I2C_RTC_CTRL;

#define INFO(_rtcReg, _rtcRegNum, _alarm1, _alarm2, _temp, _sram, _sramSize) \
    (&(I2C_RTC_CTRL_INFO) { \
        .rtcReg = (_rtcReg),       \
        .rtcRegNum = (_rtcRegNum), \
        .alarm1 = (_alarm1),       \
        .alarm2 = (_alarm2),       \
        .temp = (_temp),           \
        .sram = (_sram),           \
        .sramSize = (_sramSize),   \
    })

LOCAL const VXB_FDT_DEV_MATCH_ENTRY i2cRtcMatch[] =
    {
        /*
         * For the compatibility issue, here we added old compatible
         * name, but it is recommended and guaranteed that use the
         * new compatible name in DTS file in the future.
         */

        {"dallas, ds1307",     INFO(0, 0x7, 0,   0,   0,    0x08, 55 )},
        {"dallas, ds1337",     INFO(0, 0x7, 0x7, 0xB, 0,    0,    0 )},
        {"dallas, ds1338",     INFO(0, 0x7, 0,   0,   0,    0x08, 55 )},
        {"dallas, ds1339",     INFO(0, 0x7, 0x7, 0xB, 0,    0,    0 )},
        {"dallas, ds1340",     INFO(0, 0x7, 0,   0,   0,    0,    0 )},
        {"dallas, ds1341",     INFO(0, 0x7, 0x7, 0xB, 0,    0,    0 )},
        {"dallas, ds1342",     INFO(0, 0x7, 0x7, 0xB, 0,    0,    0 )},
        {"dallas, ds1371",     INFO(0, 0x4, 0x4, 0,   0,    0,    0 )},
        {"dallas, ds1372",     INFO(0, 0x4, 0x4, 0,   0,    0,    0 )},
        {"dallas, ds1374",     INFO(0, 0x4, 0x4, 0,   0,    0,    0 )},
        {"dallas, ds1375",     INFO(0, 0x7, 0x7, 0xB, 0,    0x10, 15)},
        {"dallas, ds1672",     INFO(0, 0x4, 0,   0,   0,    0,    0)},
        {"dallas, ds1678",     INFO(0, 0x7, 0,   0,   0,    0,    0)},
        {"dallas, ds3231",     INFO(0, 0x7, 0x7, 0xB, 0x11, 0x14, 235)},
        {"dallas, ds3232",     INFO(0, 0x7, 0x7, 0xB, 0x11, 0x14, 235)},
        {"dallas, ds32b35",    INFO(0, 0x7, 0x7, 0xB, 0x11, 0,    0  )},
        {"pericom, pt7c4338",  INFO(0, 0x7, 0,   0,   0,    0x08, 55 )},
        {"microchip, mcp79410",INFO(0, 0x7, 0xA, 0x11,0,    0x20, 64 )},
        
        {"dallas,ds1307",      INFO(0, 0x7, 0,   0,   0,    0x08, 55 )},
        {"dallas,ds1337",      INFO(0, 0x7, 0x7, 0xB, 0,    0,    0 )},
        {"dallas,ds1338",      INFO(0, 0x7, 0,   0,   0,    0x08, 55 )},
        {"dallas,ds1339",      INFO(0, 0x7, 0x7, 0xB, 0,    0,    0 )},
        {"dallas,ds1340",      INFO(0, 0x7, 0,   0,   0,    0,    0 )},
        {"dallas,ds1341",      INFO(0, 0x7, 0x7, 0xB, 0,    0,    0 )},
        {"dallas,ds1342",      INFO(0, 0x7, 0x7, 0xB, 0,    0,    0 )},
        {"dallas,ds1371",      INFO(0, 0x4, 0x4, 0,   0,    0,    0 )},
        {"dallas,ds1372",      INFO(0, 0x4, 0x4, 0,   0,    0,    0 )},
        {"dallas,ds1374",      INFO(0, 0x4, 0x4, 0,   0,    0,    0 )},
        {"dallas,ds1375",      INFO(0, 0x7, 0x7, 0xB, 0,    0x10, 15)},
        {"dallas,ds1672",      INFO(0, 0x4, 0,   0,   0,    0,    0)},
        {"dallas,ds1678",      INFO(0, 0x7, 0,   0,   0,    0,    0)},
        {"dallas,ds3231",      INFO(0, 0x7, 0x7, 0xB, 0x11, 0x14, 235)},
        {"dallas,ds3232",      INFO(0, 0x7, 0x7, 0xB, 0x11, 0x14, 235)},
        {"dallas,ds32b35",     INFO(0, 0x7, 0x7, 0xB, 0x11, 0,    0  )},
        {"pericom,pt7c4338",   INFO(0, 0x7, 0,   0,   0,    0x08, 55 )},
        {"microchip,mcp79410", INFO(0, 0x7, 0xA, 0x11,0,    0x20, 64 )},
        {} /* Empty terminated list */
    };

/* VxBus methods */

STATUS vxbI2cRtcInstUnlink (VXB_DEV_ID pDev, void * unused);

#ifdef  I2C_RTC_CTRL_DEBUG
LOCAL STATUS vxbI2cRtcShow (VXB_DEV_ID pDev, int verbose);
#endif

/*
 * If the RTC works at byte write mode, tBUF time is needed during each byte
 * write, if works at subsequential write mode, this delay is not needed.
 * ------------------------------------------------------------------
 * |Bus Free Time Between         | tBUF | Fast mode      |   1.3us |
 * |STOP and START Conditions     |      | Standard mode  |   4.7us |
 * ------------------------------------------------------------------
 */

LOCAL STATUS vxbI2cRtcProbe(struct vxbDev * pDev);
LOCAL STATUS vxbI2cRtcAttach(struct vxbDev * pDev);

LOCAL VXB_DRV_METHOD vxbI2cRtc_methods[] = {
    { VXB_DEVMETHOD_CALL(vxbDevProbe), vxbI2cRtcProbe},
    { VXB_DEVMETHOD_CALL(vxbDevAttach), vxbI2cRtcAttach},
    VXB_DEVMETHOD_END
};

VXB_DRV vxbOfTiK3I2cRtcDrv =
    {
    { NULL },
    "rtc",              /* Name */
    "Dallas rtc",       /* Description */
    VXB_BUSID_FDT,      /* Class */
    0,                  /* Flags */
    0,                  /* Reference count */
    vxbI2cRtc_methods   /* Method table */
    };

VXB_DRV_DEF(vxbOfTiK3I2cRtcDrv)

/******************************************************************************
*
* vxbI2cRtcProbe - probe for device presence at specific address
*
* Check for i2c rtc controller (or compatible) device at the specified
* base address. We assume one is present at that address, but we need to verify.
*
* RETURNS: OK or ERROR.
*
*/

LOCAL STATUS vxbI2cRtcProbe
    (
    VXB_DEV_ID pDev /* Device information */
    )
    {
    VXB_FDT_DEV_MATCH_ENTRY *pMatch;
    I2C_RTC_CTRL_INFO * pI2cRtcCtrlInfo;

    if (vxbFdtDevMatch (pDev, i2cRtcMatch, &pMatch) == ERROR)
        return ERROR;

    pI2cRtcCtrlInfo = ( I2C_RTC_CTRL_INFO *)pMatch->data;

    vxbDevDrvDataSet(pDev,(void *)pI2cRtcCtrlInfo);

    return OK;
    }

/******************************************************************************
*
* vxbI2cRtcAttach - attach rtc device
*
* This routine attaches i2c eeprom device with vxbOfI2cRtcDrv,it creates child
* device and assigns resource for it.
*
* RETURNS: OK or ERROR
*
* ERRNO
*/

LOCAL STATUS vxbI2cRtcAttach
    (
    VXB_DEV_ID pDev
    )
    {
    I2C_RTC_CTRL *      pDrvCtrl;
    VXB_RESOURCE_ADR *  pResAdr = NULL;
    VXB_RESOURCE *      pRes = NULL;
    VXB_I2C_RTC_FUNC *  i2cRtc = NULL;
    I2C_RTC_CTRL_INFO * pI2cRtcCtrlInfo;
    VXB_FDT_DEV *       pFdtDev;
    const VOID *        pVal;
    UINT32              intPol;
    int                 len;
    TASK_ID             rc = TASK_ID_NULL;

    if ((pFdtDev = (VXB_FDT_DEV *) (vxbFdtDevGet (pDev))) == NULL)
        return ERROR;

    pDrvCtrl = (I2C_RTC_CTRL *) vxbMemAlloc (sizeof (I2C_RTC_CTRL));

    if (pDrvCtrl == NULL)
        return ERROR;

   /* save instance ID */

    pDrvCtrl->pDev = pDev;

    /* save pDrvCtrl in VXB_DEVICE structure */

    vxbDevSoftcSet(pDev, pDrvCtrl);

    pI2cRtcCtrlInfo = (I2C_RTC_CTRL_INFO *)vxbDevDrvDataGet(pDev);

    pDrvCtrl->i2cRtcInfo.rtcReg = pI2cRtcCtrlInfo->rtcReg;
    pDrvCtrl->i2cRtcInfo.rtcRegNum =pI2cRtcCtrlInfo->rtcRegNum;
    pDrvCtrl->i2cRtcInfo.alarm1 = pI2cRtcCtrlInfo->alarm1;

    pDrvCtrl->i2cRtcInfo.alarm2 = pI2cRtcCtrlInfo->alarm2;
    pDrvCtrl->i2cRtcInfo.temp = pI2cRtcCtrlInfo->temp;
    pDrvCtrl->i2cRtcInfo.sram =pI2cRtcCtrlInfo->sram;
    pDrvCtrl->i2cRtcInfo.sramSize = pI2cRtcCtrlInfo->sramSize;

    /* retrieve SCL clock frequency of data access */

    pVal = vxFdtPropGet (pFdtDev->offset, "data-scl-frequency", NULL);
    if ((pVal == NULL) || ((*(UINT32 *) pVal) == 0))
        pDrvCtrl->dataScl = FAST_MODE;
    else
        {
        pDrvCtrl->dataScl = *(UINT32 *) pVal;
        pDrvCtrl->dataScl = vxFdt32ToCpu (pDrvCtrl->dataScl);
        }

    /* buffer (data + address at the beginning) */

    pDrvCtrl->writebuf = vxbMemAlloc ((size_t)(pDrvCtrl->i2cRtcInfo.rtcRegNum + 1));

    if (pDrvCtrl->writebuf == NULL)
        {
        vxbMemFree(pDrvCtrl);
        return ERROR;
        }

    /* retrieve the i2cAddress */

    pRes = vxbResourceAlloc (pDev, VXB_RES_MEMORY, 0);

    if (pRes == NULL)
        goto Exit;

    pResAdr = (VXB_RESOURCE_ADR *) pRes->pRes;

    if (pResAdr == NULL)
         goto Exit;

    pDrvCtrl->i2cAddr = (UINT16)pResAdr->virtual;
    
    /* initialize semaphore which is used to active monitor routine */
    
    pDrvCtrl->irqSem = semBCreate (SEM_Q_PRIORITY, SEM_EMPTY);
    if (pDrvCtrl->irqSem == SEM_ID_NULL)
        {
        I2CRTC_MSG ("irqSem create failed\n", 1, 2, 3, 4, 5, 6);
        goto Exit;
        }
    
    i2cRtc = (VXB_I2C_RTC_FUNC *) vxbMemAlloc(sizeof(VXB_I2C_RTC_FUNC));
    if (i2cRtc == NULL)
        goto Exit;

    i2cRtc->rtcGet = vxbI2cRtcTimeGet;
    i2cRtc->rtcSet = vxbI2cRtcTimeSet;
    i2cRtc->alarmSet = vxbI2cRtcAlarmSet;
    i2cRtc->alarmGet = vxbI2cRtcAlarmGet;

    /* Calculate how many alarms in RTC chip */

    if (pDrvCtrl->i2cRtcInfo.alarm1 != 0)
        {
        i2cRtc->alarmCap[0] = RTC_ALARM_SEC_SUPPORT;
        
        if (pDrvCtrl->i2cRtcInfo.alarm2 != 0)
            {
            i2cRtc->alarmNum = 2;
            i2cRtc->alarmCap[1] = RTC_ALARM_SEC_SUPPORT;
            }
        else
            i2cRtc->alarmNum = 1;
        }
    else
        i2cRtc->alarmNum = 0;

    /* retrieve GPIO interrupt pin if exists */

    pVal = vxFdtPropGet (pFdtDev->offset, "int-gpio-pin", &len);
    if (pVal != NULL)
        {
        pDrvCtrl->intPin = vxFdt32ToCpu (*(UINT32 *)pVal);
        I2CRTC_MSG ("GPIO interrupt pin %d\n", pDrvCtrl->intPin, 2, 3, 4, 5, 6);
        
        if (vxbGpioAlloc (pDrvCtrl->intPin) == ERROR)
            {
            I2CRTC_MSG ("vxbGpioAlloc failed!\n", 1, 2, 3, 4, 5, 6);
            goto Exit;
            }
            
        if (vxbGpioSetDir (pDrvCtrl->intPin, GPIO_DIR_INPUT) == ERROR)
            {
            (void)vxbGpioFree (pDrvCtrl->intPin);
            I2CRTC_MSG ("vxbGpioSetDir failed!\n", 1, 2, 3, 4, 5, 6);
            goto Exit;
            }
        
        pVal = vxFdtPropGet (pFdtDev->offset, "int-gpio-polarity", &len);
        
        if (pVal != NULL)
            {
            intPol = vxFdt32ToCpu (*(UINT32 *)pVal);
            I2CRTC_MSG ("GPIO interrupt polarity %d\n", intPol, 2, 3, 4, 5, 6);
            
            /* configure the interrupt trigger mode and polarity */
            
            if (vxbGpioIntConfig (pDrvCtrl->intPin, INTR_TRIGGER_EDGE, intPol) 
               == ERROR)
                {
                (void)vxbGpioFree (pDrvCtrl->intPin);
                I2CRTC_MSG ("vxbGpioIntConfig failed!\n", 1, 2, 3, 4, 5, 6);
                goto Exit;
                }
            }
        else
            {
            (void)vxbGpioFree (pDrvCtrl->intPin);
            I2CRTC_MSG ("Invalid interrupt polarity\n", 1, 2, 3, 4, 5, 6);
            goto Exit;
            }
        
        /* connect interrupt */

        if (vxbGpioIntConnect (pDrvCtrl->intPin, (VOIDFUNCPTR)vxbI2cRtcAlarmIsr, 
                               (void *)pDev) == ERROR)
            {
            (void)vxbGpioFree (pDrvCtrl->intPin);
            I2CRTC_MSG ("vxbGpioIntConnect failed\n", 1, 2, 3, 4, 5, 6);
            goto Exit;
            }

        if (vxbGpioIntEnable (pDrvCtrl->intPin, (VOIDFUNCPTR)vxbI2cRtcAlarmIsr, 
                              (void *)pDev) == ERROR)
            {
            (void)vxbGpioIntDisconnect (pDrvCtrl->intPin,
                                        (VOIDFUNCPTR)vxbI2cRtcAlarmIsr,
                                        (void *)pDev);
            (void)vxbGpioFree (pDrvCtrl->intPin);
            I2CRTC_MSG ("vxbGpioIntEnable failed\n", 1, 2, 3, 4, 5, 6);
            goto Exit;
            }
        
        /* set GPIO interrupt flag */
        
        pDrvCtrl->gpioInt = TRUE;
        }
    else
        {
        /* If no GPIO interrupt, use legacy interrupt */
        
        pDrvCtrl->intRes = vxbResourceAlloc (pDev, VXB_RES_IRQ, 0);

        if (pDrvCtrl->intRes == NULL)
            {
            I2CRTC_MSG ("vxbI2cRtcAttach: allocated IRQ pRes failed."
                        "RTC time feature is still available, "
                        "but alarm feature isn't.\n", 1, 2, 3, 4, 5, 6);
            
            i2cRtc->alarmNum = 0;
            }
        else
            {
            if (vxbIntConnect (pDev, pDrvCtrl->intRes, 
                               (VOIDFUNCPTR) vxbI2cRtcAlarmIsr, (void *) pDev) != OK)
                {
                I2CRTC_MSG ("vxbIntConnect failed\n", 1, 2, 3, 4, 5, 6);
                goto Exit;
                }

            if (vxbIntEnable (pDev, pDrvCtrl->intRes) != OK)
                {
                I2CRTC_MSG ("vxbIntEnable failed\n", 1, 2, 3, 4, 5, 6);
                (void) vxbIntDisconnect (pDev, pDrvCtrl->intRes);
                goto Exit;
                }
            }
        }

    if ((pDrvCtrl->intRes != NULL) || (pDrvCtrl->gpioInt))
        {
        
        /* spawn a task to monitor the alarm interrupt */
        
        rc = taskSpawn (RTC_ALARM_MON_TASK_NAME, RTC_ALARM_MON_TASK_PRI, 0,
                        RTC_ALARM_MON_TASK_STACK, (FUNCPTR) vxbI2cRtcAlarmMon,
                        (_Vx_usr_arg_t)pDev, 0, 0, 0, 0, 0, 0, 0, 0, 0);
       
        if (rc == TASK_ID_NULL)
            {
            I2CRTC_MSG ("Spawn task failed\n", 1, 2, 3, 4, 5, 6);
            goto Exit;
            }
        }

    rtcRegister(pDev, i2cRtc);

    return OK;

Exit:
    if (pRes != NULL)
        (void)vxbResourceFree(pDev, pRes);
    
    if (i2cRtc != NULL)
        vxbMemFree (i2cRtc);

    if (pDrvCtrl->intRes != NULL)
        (void)vxbResourceFree (pDev, pDrvCtrl->intRes);

    if (pDrvCtrl->irqSem != NULL)
        (void)semDelete (pDrvCtrl->irqSem);
    
    if (pDrvCtrl->writebuf != NULL)
        vxbMemFree (pDrvCtrl->writebuf);
    
    vxbMemFree (pDrvCtrl);

    return ERROR;
    }

/******************************************************************************
*
* vxbI2cRtcRead - rtc read routine
*
* This routine reads rtc time.
*
* RETURNS: OK or ERROR
*
* ERRNO
*/

LOCAL STATUS vxbI2cRtcRead
    (
    VXB_DEV_ID pDev,
    UINT32     offset,
    UINT8 *    buf,
    UINT32     length
    )
    {
    I2C_RTC_CTRL *pDrvCtrl = vxbDevSoftcGet(pDev);
    UINT32 i = 0;
    I2C_MSG msg[2];
    UINT8 msgbuf[2];

    memset(msg, 0, sizeof(I2C_MSG) * 2);
    memset(msgbuf, 0, sizeof(msgbuf));

    msgbuf[i++] = (UINT8)offset;

    msg[0].addr = pDrvCtrl->i2cAddr;
    msg[0].scl  = STD_MODE ;
    msg[0].buf  = msgbuf;
    msg[0].len  = i;

    msg[1].addr  = pDrvCtrl->i2cAddr;
    msg[1].scl   = pDrvCtrl->dataScl;
    msg[1].flags = I2C_M_RD;
    msg[1].buf   = buf;
    msg[1].len   = length;

    return (vxbI2cDevXfer(pDev, msg, 2));
    }

/******************************************************************************
*
* vxbI2cRtcWrite - write rtc time
*
* This routine programs rtc time.
*
* RETURNS: OK or ERROR
*
* ERRNO
*/

LOCAL STATUS vxbI2cRtcWrite
    (
    VXB_DEV_ID pDev,
    UINT32     offset,
    UINT8 *    buf,
    UINT32     length
    )
    {
    I2C_RTC_CTRL *pDrvCtrl = vxbDevSoftcGet(pDev);
    I2C_MSG msg;
    UINT32 i = 0;

    memset(&msg, 0, sizeof(I2C_MSG));

    msg.addr = pDrvCtrl->i2cAddr;
    msg.buf  = pDrvCtrl->writebuf;
    msg.scl  = pDrvCtrl->dataScl;

    msg.flags = 0;
    msg.buf[i++] = (UINT8)offset;
    memcpy(&msg.buf[i], buf, length);
    msg.len = i + length;
    msg.wrTime = I2C_RTC_WRDELAY;

    return (vxbI2cDevXfer(pDev, &msg, 1));
    }

/*****************************************************************************
*
* vxbI2cRtcInstUnlink -  vxBus unlink handler
*
* VxBus unlink handler.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

STATUS vxbI2cRtcInstUnlink
    (
    VXB_DEV_ID pDev,
    void *     unused
    )
    {
    I2C_RTC_CTRL *pDrvCtrl = vxbDevSoftcGet(pDev);
    struct tm rtcTime;

    memset (&rtcTime, 0, sizeof(struct tm));

    free(pDrvCtrl->writebuf);

    vxbMemFree ((char *) pDrvCtrl);

    return (OK);
    }

/*******************************************************************************
*
* vxbI2cRtcDateCheck - check whether date/time values are valid
*
* This routine tests the validity of the values in the tm structure.
*
* RETURNS: OK, or ERROR if any values are invalid
*/

LOCAL STATUS vxbI2cRtcDateCheck
    (
    VXB_DEV_ID  pDev,
    struct tm * rtcTime    /* pointer to time keeping structure */
    )
    {
    int monthday = 0;

    if (NULL == rtcTime)
        return ERROR;

    /* Check validity of year value */

    if ((rtcTime->tm_year <  (MIN_YEAR - TM_YEAR_BASE)) ||
        (rtcTime->tm_year >  (MAX_YEAR - TM_YEAR_BASE)))
        {
        I2CRTC_MSG ("ERROR: year should be between 100 and 199, BASE_YEAR is 1900\n",
                   1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* Check validity of month value */

    if (rtcTime->tm_mon < 0 || rtcTime->tm_mon > 11)
        {
        I2CRTC_MSG ("ERROR: month should be between 0 and 11\n", 1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* Check validity of day of month value */

    if (IS_LEAP_YEAR (rtcTime->tm_year+TM_YEAR_BASE) && (1 == rtcTime->tm_mon))
        monthday = 29;
    else
        monthday = month_days[rtcTime->tm_mon];

    if (rtcTime->tm_mday <1 || rtcTime->tm_mday> monthday)
        {
        I2CRTC_MSG ("ERROR: mDay should be between 1 and %d\n", monthday, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* Check validity of hours value */

    if (rtcTime->tm_hour > 23)
        {
        I2CRTC_MSG ("ERROR: hour should be less than 24\n", 1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* Check validity of minutes value */

    if (rtcTime->tm_min > 59)
        {
        I2CRTC_MSG ("ERROR: minute should be less than 60\n", 1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* Check validity of seconds value */

    if (rtcTime->tm_sec > 59)
        {
        I2CRTC_MSG ("ERROR: second should be less than 60\n", 1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    return OK;
    }

/*******************************************************************************
*
* vxbI2cRtcConvert - convert the year and month to days
*
* This routine allows the caller to convert the year and month to days.
*
* RETURNS: return the seconds.
*/

LOCAL int vxbI2cRtcConvert
    (
    int startYear,
    int endYear,
    int month
    )
    {
    int i = 0;
    int leapyear = 0;
    int dayspriormonths = 0;

    /* Count leap years since epoch */

    for (i = startYear; i < endYear; i++)
        {
        if (IS_LEAP_YEAR (i))
            leapyear++;
        }

    /* Count days between start of year and end of prior month */

    for (i = 1; i < month; i++)
        dayspriormonths += month_days[i - 1];

    /* Leapday */

    if (IS_LEAP_YEAR (endYear) && (FEBRUARY < month))
        dayspriormonths++;

    return ((dayspriormonths + leapyear) + (endYear - startYear) * DAYS_IN_YEAR);
    }

/*******************************************************************************
*
* vxbI2cRtcTimeSet - set the RTC's date/time per caller's values
*
* This routine allows the caller to set the RTC time and date.  The caller
* must allocate space for an tm structure, fill the structure
* with the desired time and date, and call this routine.
*
* RETURNS: OK, or ERROR if date/time values are invalid.
*/

LOCAL STATUS vxbI2cRtcTimeSet
    (
    VXB_DEV_ID  pDev,
    struct tm * rtcTime   /* pointer to time keeping structure */
    )
    {
    UINT8 i2cBuff[7];
    int day = 0;
    int i = 0;
    time_t calTime;        /* calendar time in seconds */

    I2C_RTC_CTRL *pDrvCtrl = vxbDevSoftcGet(pDev);

    /* Determine whether date/time values are valid */

    if (vxbI2cRtcDateCheck (pDev, rtcTime) != OK)
        return ERROR;

    /* Debug info */

    I2CRTC_MSG ("vxbI2cRtcTimeSet:        \n", 1, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("rtcTime->tm_year = %03d\n", rtcTime->tm_year, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("rtcTime->tm_mon  = %02d\n", rtcTime->tm_mon, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("rtcTime->tm_mday = %02d\n", rtcTime->tm_mday, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("rtcTime->tm_hour = %02d\n", rtcTime->tm_hour, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("rtcTime->tm_min  = %02d\n", rtcTime->tm_min, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("rtcTime->tm_sec  = %02d\n", rtcTime->tm_sec, 2, 3, 4, 5, 6);

    /* If the RTC has the 7 distinct registers  */

    if (pDrvCtrl->i2cRtcInfo.rtcRegNum == 0x7)
        {
        /* Bit 7 is set to enable oscillator for MCP79410 */
        
        i2cBuff[0] = (BIN_TO_BCD (rtcTime->tm_sec)) | MCP79410_BIT_ST;     /* seconds */

        i2cBuff[1] = BIN_TO_BCD (rtcTime->tm_min);     /* minutes */
        i2cBuff[2] = BIN_TO_BCD (rtcTime->tm_hour);    /* hours   */
        i2cBuff[4] = BIN_TO_BCD (rtcTime->tm_mday);    /* dayOfMon*/
        i2cBuff[5] = BIN_TO_BCD (rtcTime->tm_mon + BIAS_MONTH); /* month   */

        /* For RTC, the years range are 0-99 */

        i2cBuff[6] = BIN_TO_BCD (BIASED_YEAR(rtcTime->tm_year)); /* year */

        /* Convert the previous year and month to days */

        day = vxbI2cRtcConvert (MIN_YEAR, rtcTime->tm_year+TM_YEAR_BASE,
                                rtcTime->tm_mon+BIAS_MONTH) + rtcTime->tm_mday;

        /* Starts from 2000/1/1, the init weekday is Saturday, range is [1-7] */

        i2cBuff[3] = (UINT8)(((((day) % DAY_IN_WEEK) +
                               INIT_WEEKDAY) % DAY_IN_WEEK) + BIAS_WDAY);

        /* Write the date/time data */

        if (vxbI2cRtcWrite (pDev, RTC_SEC_REG, i2cBuff, 7) != OK)
            return ERROR;
        }

    /* If the RTC has 4 cascaded registers  */

    else if (pDrvCtrl->i2cRtcInfo.rtcRegNum == 0x4)
        {
        calTime = mktime (rtcTime);

        I2CRTC_MSG ("calTime          = %ld\n", calTime, 2, 3, 4, 5, 6);

        for (i = RTC_CNT_BYTE0_ADDR; i <= RTC_CNT_BYTE3_ADDR; i++)
            {
            i2cBuff[i] = (UINT8)(calTime & 0xff);
            calTime = calTime >> 8;
            }

        /* Write date/time data */

        if (vxbI2cRtcWrite (pDev, RTC_CNT_BYTE0_ADDR, i2cBuff, 4) != OK)
            return ERROR;
        }
    else
        return ERROR;

    return OK;
    }

/*******************************************************************************
*
* vxbI2cRtcTimeGet - get current RTC date/time
*
* This routine allows the caller to obtain the current RTC time and date.
* The caller must allocate space for an tm structure, then call
* this routine.
*
* RETURNS: OK, or ERROR if unable to retrieve the current RTC date and time.
*/

LOCAL STATUS vxbI2cRtcTimeGet
    (
    VXB_DEV_ID  pDev,
    struct tm * rtcTime   /* pointer to time keeping structure */
    )
    {
    I2C_RTC_CTRL *pDrvCtrl = vxbDevSoftcGet(pDev);
    time_t tim = 0;
    UINT8 i2cBuff[7];

    /* Clear the array */

    memset(i2cBuff, 0, sizeof(i2cBuff));

    I2CRTC_MSG ("vxbI2cRtcTimeGet:       %d \n", pDrvCtrl->i2cRtcInfo.rtcRegNum, 2, 3, 4, 5, 6);

    if (pDrvCtrl->i2cRtcInfo.rtcRegNum == 0x7)
        {
        /* Read date/time data */

        if (vxbI2cRtcRead (pDev, 0, i2cBuff, 7) != OK)
            return ERROR;

        rtcTime->tm_sec = BCD_TO_BIN (i2cBuff[0] & 0x7F);
        rtcTime->tm_min = BCD_TO_BIN (i2cBuff[1] & 0x7F);
        rtcTime->tm_hour = BCD_TO_BIN (i2cBuff[2] & 0x3F);
        rtcTime->tm_wday = BCD_TO_BIN (i2cBuff[3] & 0x07) - BIAS_WDAY;
        rtcTime->tm_mday = BCD_TO_BIN (i2cBuff[4] & 0x3F);
        rtcTime->tm_mon = BCD_TO_BIN (i2cBuff[5] & 0x1F);

        /*
         * Assume 20YY not 19YY, but ansi-time years since 1900, so add 100.
         * Ignore CENTURY Bit in month register for general.
         */

        rtcTime->tm_year = UNBIASED_YEAR(BCD_TO_BIN (i2cBuff[6]));

        /* The range of tm_mon is 0-11 but in RTC, the range is 1-12 */

        rtcTime->tm_mon -= BIAS_MONTH;

        /* __jullday needs years since TM_YEAR_BASE */

        rtcTime->tm_yday = __julday (rtcTime->tm_year,
                                     rtcTime->tm_mon,
                                     rtcTime->tm_mday);
        }

    /* If the RTC has 4 cascaded registers  */

    else if (pDrvCtrl->i2cRtcInfo.rtcRegNum == 0x4)
        {
        /* Read date/time data */

        if (vxbI2cRtcRead (pDev, RTC_CNT_BYTE0_ADDR, i2cBuff, 4) != OK)
            return ERROR;

		UINT32 tmp;
        tmp = ((UINT32)(i2cBuff[3] << 24) | (UINT32)(i2cBuff[2] << 16) |
               (UINT32)(i2cBuff[1] << 8)  | (UINT32)(i2cBuff[0]));
		tim = (time_t)tmp;

        I2CRTC_MSG ("calTime          = %ld\n", tim, 2, 3, 4, 5, 6);

        /* Convert calendar time into broken-down time */

        (void) gmtime_r (&tim, rtcTime);
        }
    else
        return ERROR;

    /* Debug info */

    I2CRTC_MSG ("rtcTime->tm_year = %03d\n", rtcTime->tm_year, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("rtcTime->tm_mon  = %02d\n", rtcTime->tm_mon, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("rtcTime->tm_mday = %02d\n", rtcTime->tm_mday, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("rtcTime->tm_hour = %02d\n", rtcTime->tm_hour, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("rtcTime->tm_min  = %02d\n", rtcTime->tm_min, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("rtcTime->tm_sec  = %02d\n", rtcTime->tm_sec, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("rtcTime->tm_wday = %02d\n", rtcTime->tm_wday, 2, 3, 4, 5, 6);

    return OK;
    }

/*******************************************************************************
*
* vxbI2cRtcAlarmGet - get current alarm date/time
*
* This routine allows the caller to obtain the current alarm time and date.
* The caller must allocate space for an tm structure, then call
* this routine.
*
* RETURNS: OK, or ERROR if unable to retrieve the current alarm date and time.
*/

LOCAL STATUS vxbI2cRtcAlarmGet
    (
    VXB_DEV_ID    pDev,
    UINT8         alarmUnit, /* Alarm No */
    struct tm *   alarmTime  /* pointer to alarm keeping structure */
    )
    {
    UINT8 i2cBuff[4];
    time_t tim = 0;
    I2C_RTC_CTRL *pDrvCtrl = vxbDevSoftcGet(pDev);

    memset (i2cBuff, 0, sizeof (i2cBuff));

    switch (alarmUnit)
        {
        case 0:
            /* If the RTC has 4 cascaded registers  */

            if (pDrvCtrl->i2cRtcInfo.alarm1 == 0x4)
                {
                if (vxbI2cRtcRead (pDev, RTC_WDALM_COUNTER_BYTE0, i2cBuff, 3) != OK)
                    return ERROR;

                tim = ((i2cBuff[2] << 16) | (i2cBuff[1] << 8) | (i2cBuff[0]));

                /* Convert calendar time into broken-down time */

                alarmTime->tm_sec = (((int)tim % SECS_PER_DAY) % SECS_PER_HOUR) %
                                    SECS_PER_MIN;
                alarmTime->tm_min = (((int)tim % SECS_PER_DAY) % SECS_PER_HOUR) /
                                    SECS_PER_MIN;
                alarmTime->tm_hour = ((int)tim % SECS_PER_DAY) / SECS_PER_HOUR;

                alarmTime->tm_mday = ((int)tim -
                                      alarmTime->tm_sec -
                                      alarmTime->tm_min * SECS_PER_MIN -
                                      alarmTime->tm_hour * SECS_PER_HOUR) /
                                      SECS_PER_DAY;
                }

            /* If the RTC has 7 registers  */

            else if (pDrvCtrl->i2cRtcInfo.alarm1 == 0x7)
                {
                if (vxbI2cRtcRead (pDev, RTC_ALARM1_REG, i2cBuff, 4) != OK)
                    return ERROR;

                alarmTime->tm_sec = BCD_TO_BIN (i2cBuff[0]);
                alarmTime->tm_min = BCD_TO_BIN (i2cBuff[1]);
                alarmTime->tm_hour = BCD_TO_BIN (i2cBuff[2]);
                alarmTime->tm_mday = BCD_TO_BIN (i2cBuff[3]);
                }
            else
                {
                I2CRTC_MSG("not supported Alarm type \r\n", 1, 2, 3, 4, 5, 6);
                return ERROR;
                }
            break;
        case 1:
            if (pDrvCtrl->i2cRtcInfo.alarm2 == 0xB)
                {
                if (vxbI2cRtcRead (pDev, RTC_ALARM2_REG, i2cBuff, 3) != OK)
                    return ERROR;

                alarmTime->tm_min = BCD_TO_BIN (i2cBuff[0]);
                alarmTime->tm_hour = BCD_TO_BIN (i2cBuff[1]);
                alarmTime->tm_mday = BCD_TO_BIN (i2cBuff[2]);
                }
            else
                {
                I2CRTC_MSG("not supported Alarm type \r\n", 1, 2, 3, 4, 5, 6);
                return ERROR;
                }
            break;
        default:
            return ERROR;
        }

    /* Debug info */

    I2CRTC_MSG ("vxbI2cRtcAlarmGet:  \r\n", 1, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("alarmTime->tm_mday   = %02d\r\n", alarmTime->tm_mday, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("alarmTime->tm_hour   = %02d\r\n", alarmTime->tm_hour,  2, 3, 4, 5, 6);
    I2CRTC_MSG ("alarmTime->tm_min = %02d\r\n", alarmTime->tm_min, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("alarmTime->tm_sec = %02d\r\n", alarmTime->tm_sec, 2, 3, 4, 5, 6);

    return (OK);
    }

/*******************************************************************************
*
* vxbI2cRtcAlarmSet - set current alarm date/time
*
* This routine allows the caller to obtain the current alarm time and date.
* The caller must allocate space for an tm structure, then call this routine.
*
* RETURNS: OK, or ERROR if unable to retrieve the current alarm date and time.
*/

LOCAL STATUS vxbI2cRtcAlarmSet
    (
    VXB_DEV_ID     pDev,
    UINT8          alarmUnit,    /* which Alarm */
    struct tm *    alarmTime,    /* pointer to time keeping structure */
    RTC_ALARM_FUNC rtcAlarmFunc, /* callback routine */
    void *         prtcAlarmArg  /* callback parameter */
    )
    {
    UINT8 control = 0;
    UINT8 temp = 0;
    UINT32 time = 0;
    UINT8 i2cBuff[4];
    I2C_RTC_CTRL *pDrvCtrl = vxbDevSoftcGet(pDev);

    memset (i2cBuff, 0, sizeof (i2cBuff));

    /* Alarm 1  */

    switch (alarmUnit)
        {
        case 0:
            /* If the RTC has 4 cascaded registers  */

            if (pDrvCtrl->i2cRtcInfo.alarm1 == 0x4)
                {
                /* Clear any pending alarm flag */

                if (vxbI2cRtcRead (pDev, RTC_CNT_SR_REG, (UINT8 *) &temp, 1) != OK)
                    return ERROR;

                temp &= (UINT8)(~RTC_CNT_SR_REG_AF);

                if (vxbI2cRtcWrite (pDev, RTC_CNT_SR_REG, (UINT8 *) &temp, 1) != OK)
                    return ERROR;

                /* Clear alarm interrupt enable bit */

                if (vxbI2cRtcRead (pDev, RTC_CNT_CTRL_REG, (UINT8 *) &control, 1) != OK)
                    return ERROR;

                /*
                 * Disable any existing alarm before setting the new one
                 * (or lack thereof).
                 */

                control &= (UINT8)(~(RTC_CNT_CTRL_REG_WACE | RTC_CNT_CTRL_REG_AIE));

                if (vxbI2cRtcWrite (pDev, RTC_CNT_CTRL_REG, (UINT8 *) &control, 1) != OK)
                    return ERROR;

                if (rtcAlarmFunc)
                    {
	                /* Compute number of seconds since epoch */

	                time = (UINT32)(alarmTime->tm_sec) +
	                       (UINT32)((alarmTime->tm_min) * SECS_PER_MIN) +
	                       (UINT32)((alarmTime->tm_hour) * SECS_PER_HOUR) +
	                       (UINT32)((alarmTime->tm_mday) * SECS_PER_DAY);

	                if (time > (1 << 23))
	                    return ERROR;

	                for (temp = 0; temp < 3; temp++)
	                    {
	                    i2cBuff[temp] = time & 0xff;
	                    time = time >> 8;
	                    }

	                /* Write Alarm date/time */

	                if (vxbI2cRtcWrite (pDev, RTC_WDALM_COUNTER_BYTE0, i2cBuff, 3) != OK)
	                    return ERROR;

	                control |= (UINT8)RTC_CNT_CTRL_REG_WACE;   /* WD/ALM Counter Enable    */
	                control |= (UINT8)RTC_CNT_CTRL_REG_AIE;    /* Alarm Interrupt Enable   */
	                control |= (UINT8)RTC_CNT_CTRL_REG_INTCN;  /* Interrupt Control (INTCN)*/
	                control &= (UINT8)(~RTC_CNT_CTRL_REG_WDALM); /* set alarm, rather than WD*/

	                if (vxbI2cRtcWrite (pDev, RTC_CNT_CTRL_REG, (UINT8 *) &control, 1) != OK)
	                    return ERROR;

	                /* Install the alarm callback routine and parameter */

                    pDrvCtrl->rtcAlarm1Func = rtcAlarmFunc;
                    pDrvCtrl->rtcAlarm1Arg  = prtcAlarmArg;
                    }
                }

            /* If the RTC has 7 registers  */

            else if (pDrvCtrl->i2cRtcInfo.alarm1 == 0x7)
                {
                i2cBuff[0] = BIN_TO_BCD (alarmTime->tm_sec);
                i2cBuff[1] = BIN_TO_BCD (alarmTime->tm_min);
                i2cBuff[2] = BIN_TO_BCD (alarmTime->tm_hour);
                i2cBuff[3] = BIN_TO_BCD (alarmTime->tm_mday);

                /* Clear alarm interrupt enable bit */

                if (vxbI2cRtcRead (pDev, RTC_CTRL_REG, (UINT8 *)&control, 1) != OK)
                    return ERROR;

                control &= (UINT8)(~RTC_CTRL_REG_A1IE);

                if (vxbI2cRtcWrite (pDev, RTC_CTRL_REG, (UINT8 *)&control, 1) != OK)
                    return ERROR;

                /* Clear any pending alarm flag */

                if (vxbI2cRtcRead (pDev, RTC_SR_REG, (UINT8 *)&temp, 1) != OK)
                    return ERROR;

                temp &= (UINT8)(~RTC_SR_REG_A1F);

                if (vxbI2cRtcWrite (pDev, RTC_SR_REG, (UINT8 *)&temp, 1) != OK)
                    return ERROR;

                if (rtcAlarmFunc)
                    {
	                /* Write Alarm date/time */

	                if (vxbI2cRtcWrite (pDev, RTC_ALARM1_REG, i2cBuff, 4) != OK)
	                    return ERROR;

	                /* Reenable the alarm */

	                control |= (UINT8)RTC_CTRL_REG_A1IE;
	                control |= (UINT8)RTC_CTRL_REG_INTCN;

	                if (vxbI2cRtcWrite (pDev, RTC_CTRL_REG, (UINT8 *) &control, 1) != OK)
	                    return ERROR;

	                /* Install the alarm callback routine and parameter */

                    pDrvCtrl->rtcAlarm1Func = rtcAlarmFunc;
                    pDrvCtrl->rtcAlarm1Arg  = prtcAlarmArg;
                    }
                }
            else
                {
                I2CRTC_MSG("not supported Alarm type \r\n", 1, 2, 3, 4, 5, 6);
                return ERROR;
                }
            break;

        /* Alarm 2  */

        case 1:
            if (pDrvCtrl->i2cRtcInfo.alarm2 == 0xB)
                {
                i2cBuff[0] = BIN_TO_BCD (alarmTime->tm_min);
                i2cBuff[1] = BIN_TO_BCD (alarmTime->tm_hour);
                i2cBuff[2] = BIN_TO_BCD (alarmTime->tm_mday);

                /* Clear alarm interrupt enable bit */

                if (vxbI2cRtcRead (pDev, RTC_CTRL_REG, (UINT8 *)&control, 1) != OK)
                    return ERROR;

                control &= (UINT8)(~RTC_CTRL_REG_A2IE);

                if (vxbI2cRtcWrite (pDev, RTC_CTRL_REG, (UINT8 *)&control, 1) != OK)
                    return ERROR;

                /* Clear any pending alarm flag */

                if (vxbI2cRtcRead (pDev, RTC_SR_REG, (UINT8 *)&temp, 1) != OK)
                    return ERROR;

                temp &= (UINT8)(~RTC_SR_REG_A2F);

                if (vxbI2cRtcWrite (pDev, RTC_SR_REG, (UINT8 *)&temp, 1) != OK)
                    return ERROR;

                if (rtcAlarmFunc)
                    {
	                /* Write Alarm date/time */

	                if (vxbI2cRtcWrite (pDev, RTC_ALARM2_REG, i2cBuff, 3) != OK)
	                    return ERROR;

	                control |= (UINT8)RTC_CTRL_REG_A2IE;
	                control |= (UINT8)RTC_CTRL_REG_INTCN;

	                if (vxbI2cRtcWrite (pDev, RTC_CTRL_REG, (UINT8 *) &control, 1) != OK)
	                    return ERROR;

	                /* Install the alarm callback routine and parameter */

                    pDrvCtrl->rtcAlarm2Func = rtcAlarmFunc;
                    pDrvCtrl->rtcAlarm2Arg  = prtcAlarmArg;
                    }
                }
            else
                {
                I2CRTC_MSG("not supported Alarm type \r\n", 1, 2, 3, 4, 5, 6);
                return ERROR;
                }
            break;

        default:
            return ERROR;
        }
    
    /* Debug info */

    I2CRTC_MSG ("vxbI2cRtcAlarmSet:  \r\n", 1, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("alarmTime>tm_sec   = %02d\r\n", alarmTime->tm_sec, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("alarmTime->tm_min  = %02d\r\n", alarmTime->tm_min, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("alarmTime->tm_hour    = %02d\r\n", alarmTime->tm_hour, 2, 3, 4, 5, 6);
    I2CRTC_MSG ("alarmTime->tm_mday    = %02d\r\n", alarmTime->tm_mday, 2, 3, 4, 5, 6);

    return OK;
    }

/*******************************************************************************
*
* vxbI2cRtcAlarmMon - monitor and call alarm interrupt handler
*
* This monitor is a spawned task, it calls the interrupt handler when semaphore
* is given.
*
* RETURN: N/A
*
* ERRNO: N/A
*/

LOCAL void vxbI2cRtcAlarmMon
    (
    VXB_DEV_ID pDev
    )
    {
    I2C_RTC_CTRL *pDrvCtrl = vxbDevSoftcGet(pDev);

    while(1)
        {
        if (semTake (pDrvCtrl->irqSem, WAIT_FOREVER) != OK)
            continue;

        /* call the alarm handler*/

        vxbI2cRtcAlarmHandler (pDev);
        }
    }

/*******************************************************************************
*
* vxbI2cRtcAlarmHandler - handle a alarm interrupt
*
* This routine handles clearing interrupts from the RTC alarm and run the ISR
* service routine.
*
* RETURN: N/A
*
* ERRNO: N/A
*/

LOCAL void vxbI2cRtcAlarmHandler
    (
    VXB_DEV_ID pDev
    )
    {
    I2C_RTC_CTRL * pDrvCtrl;
    UINT8          stat = 0;
    
    if (pDev == NULL)
        return;
    
    pDrvCtrl = vxbDevSoftcGet(pDev);
    if (pDrvCtrl == NULL)
        return;

    if (pDrvCtrl->i2cRtcInfo.alarm1 == 0x4)
        {
        if (vxbI2cRtcRead (pDev, RTC_CNT_SR_REG, (UINT8 *) &stat, 1) != OK)
            return;
        
        if ((stat & RTC_CNT_SR_REG_AF) != 0)
            {
            /* Clear any pending alarm flag */
            
            stat &= (UINT8)(~RTC_CNT_SR_REG_AF);

            if (vxbI2cRtcWrite (pDev, RTC_CNT_SR_REG, (UINT8 *) &stat, 1) != OK)
                return;

            if ((pDrvCtrl->rtcAlarm1Func != NULL))
                {
                pDrvCtrl->rtcAlarm1Func (pDrvCtrl->rtcAlarm1Arg);
                }
            }
        }

    else if ((pDrvCtrl->i2cRtcInfo.alarm1 == 0x7) ||
             (pDrvCtrl->i2cRtcInfo.alarm2 == 0xB))
        {
        if (vxbI2cRtcRead (pDev, RTC_SR_REG, (UINT8 *)&stat, 1) != OK)
            return;

        if ((stat & RTC_SR_REG_A1F) != 0)
            {
            /* Clear any pending alarm flag */

            stat &= (UINT8)(~RTC_SR_REG_A1F);

            if (vxbI2cRtcWrite (pDev, RTC_SR_REG, (UINT8 *)&stat, 1) != OK)
                return;

            /* Call the alarm1 service routine. */

            if ((pDrvCtrl->rtcAlarm1Func != NULL))
                {
                pDrvCtrl->rtcAlarm1Func (pDrvCtrl->rtcAlarm1Arg);
                }
            }

        if (stat & RTC_SR_REG_A2F)
            {
            /* Clear any pending alarm flag */

            if (vxbI2cRtcRead (pDev, RTC_SR_REG, (UINT8 *)&stat, 1) != OK)
                return;

            if ((stat & RTC_SR_REG_A2F) != 0)
                {
                /* Clear any pending alarm flag */

                stat &= (UINT8)(~RTC_SR_REG_A2F);

                if (vxbI2cRtcWrite (pDev, RTC_SR_REG, (UINT8 *)&stat, 1) != OK)
                    return;

                /* Call the alarm1 service routine. */

                if ((pDrvCtrl->rtcAlarm1Func != NULL))
                    {
                    pDrvCtrl->rtcAlarm1Func (pDrvCtrl->rtcAlarm1Arg);
                    }
                }
            }
        }

    return;
    }

/*******************************************************************************
*
* vxbI2cRtcAlarmIsr - interrupt service routine
*
* This routine handles interrupts of RTC alarm.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

LOCAL STATUS vxbI2cRtcAlarmIsr
    (
    VXB_DEV_ID pDev
    )
    {
    I2C_RTC_CTRL *pDrvCtrl = vxbDevSoftcGet(pDev);

    /*
     * Clear the RTC alarm interrupt need to clear the RTC register,
     * which should get through the I2C Bus access, the new I2C interrupt
     * will be occurred, so here give the semaphore to monitor task which will
     * handle this.
     */
    
    (void)semGive (pDrvCtrl->irqSem);

    return OK;
    }

#ifdef  I2C_RTC_CTRL_DEBUG
/*******************************************************************************
*
* vxbI2cRtcShow - show the date/time on the user's display
*
* This routine retrieves the current RTC date and time and sends it in a
* user-readable fashion to the user's display.
*
* RETURNS: OK, or ERROR if unable to retrieve or print the current RTC
* date and time.
*/

LOCAL STATUS vxbI2cRtcShow
    (
    VXB_DEV_ID pDev,
    int        verbose
    )
    {
    struct tm * rtcTime;     /* pointer to time-keeping structure */
    char outputBuffer[50];   /* output buffer for returned string */

    /* Firstly, malloc the struct */

    if ((rtcTime = (struct tm * )malloc(sizeof(struct tm))) == NULL)
        {
        return ERROR;
        }

    memset(rtcTime,  0,  sizeof(struct tm));

    /* Retrieve the current RTC date and time */

    if (verbose > 10)
        {
        if (vxbI2cRtcTimeGet (pDev, rtcTime) == ERROR)
            {
            (void) printf ("            Retrieve the time error, try set time first.\n");
            goto Exit;
            }

        if (!asctime_r (rtcTime, outputBuffer))
            goto Exit;;
        (void) printf ("            %s", outputBuffer);
        }

     free(rtcTime);
     return OK;

Exit:
     free(rtcTime);
     return ERROR;
    }
#endif
