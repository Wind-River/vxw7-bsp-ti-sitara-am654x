/* vxbTiK3I2cRtc.h - header file for I2C RTC driver  */

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

#ifndef __INCvxbTiK3I2cRtch
#define __INCvxbTiK3I2cRtch

#include <hwif/vxBus.h>
#include <hwif/buslib/vxbFdtLib.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BSP_VERSION

/* RTC register addresses */
/*  7 distinct registers */

#define RTC_SEC_REG                 0x0
#define RTC_MIN_REG                 0x1
#define RTC_HOUR_REG                0x2
#define RTC_DAY_REG                 0x3
#define RTC_DATE_REG                0x4
#define RTC_MONTH_REG               0x5
#define RTC_YEAR_REG                0x6

/* Alarm 1 BASE */

#define RTC_ALARM1_REG              0x07

/* Alarm 2 BASE */

#define RTC_ALARM2_REG              0x0B

/* Control register */

#define RTC_CTRL_REG                0x0E
#define RTC_CTRL_REG_INTCN          0x04
#define RTC_CTRL_REG_A2IE           0x02
#define RTC_CTRL_REG_A1IE           0x01

/* Status register */

#define RTC_SR_REG                  0x0F
#define RTC_SR_REG_A2F              0x02
#define RTC_SR_REG_A1F              0x01

/* 4 cascaded registers */

#define RTC_CNT_BYTE0_ADDR          0x00
#define RTC_CNT_BYTE1_ADDR          0x01
#define RTC_CNT_BYTE2_ADDR          0x02
#define RTC_CNT_BYTE3_ADDR          0x03
#define RTC_WDALM_COUNTER_BYTE0     0x04
#define RTC_WDALM_COUNTER_BYTE1     0x05
#define RTC_WDALM_COUNTER_BYTE2     0x06

/* Control */

#define RTC_CNT_CTRL_REG            0x07

/* Alarm Int. Enable */

#define RTC_CNT_CTRL_REG_AIE        0x01
#define RTC_CNT_CTRL_REG_INTCN      0x08

/* 1=Watchdog, 0=Alarm */

#define RTC_CNT_CTRL_REG_WDALM      0x20

/* WD/Alarm counter enable */

#define RTC_CNT_CTRL_REG_WACE       0x40 

/* Status */

#define RTC_CNT_SR_REG              0x08

/* Alarm Flag */

#define RTC_CNT_SR_REG_AF           0x01

/* Oscillator Stop Flag */

#define RTC_CNT_SR_REG_OSF          0x80

/* misc defines */

#define SECS_PER_YEAR               (60 * 60 * 24 * 365)
#define SECS_PER_DAY                (60 * 60 * 24)
#define SECS_PER_HOUR               (60 * 60)
#define SECS_PER_MIN                (60)
#define DAYS_IN_YEAR                365
#define DAY_IN_WEEK                 7
#define JANUARY                     1
#define FEBRUARY                    2
#define DECEMBER                    12
#define IS_LEAP_YEAR(year)          ((((year) % 4 == 0) && ((year) % 100 != 0)) \
                                    || (year) % 400 == 0)

/* RTC Start Oscillator bit flag */

#define MCP79410_BIT_ST             0x80

/* 
 * The year register only can save 00-99, so the BASE_YEAR is needed,
 * Currently, setting the BASE_YEAR as 2000.
 */

#define MIN_YEAR                    2000
#define MAX_YEAR                    2099

#define INIT_WEEKDAY                5
#define I2C_RTC_WRDELAY             5

/* RTC alarm monitor task name */

#define RTC_ALARM_MON_TASK_NAME     "tRtcAlarmMon"

/* RTC alarm monitor task priority */

#define RTC_ALARM_MON_TASK_PRI      100

/* RTC alarm monitor task stack size */

#define RTC_ALARM_MON_TASK_STACK    8192

/* month days in one year */

static const int month_days[12] = 
    {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };

/* typedef */

typedef struct vxbRtcFunctionality RTC_FUNC_SET;

/*
 * the following macros convert from BCD to binary and back.
 * Be careful that the arguments are chars, only char width returned.
 */

#define BCD_TO_BIN(bcd) ((((((bcd) & 0xf0) >> 4) * 10) + ((bcd) & 0xf)) & 0xff)
#define BIN_TO_BCD(bin) ((UINT8)(((((bin) / 10) << 4) + ((bin) % 10)) & 0xff))

#endif /* BSP_VERSION */

#ifdef __cplusplus
}
#endif

#endif /* __INCvxbTiK3I2cRtch */

