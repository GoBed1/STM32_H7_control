#ifndef STANDBY_CHECK_APP_H
#define STANDBY_CHECK_APP_H

#include "stm32h7xx_hal.h"  // H7系列，其他芯片改对应头文件
#include "rtc.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdio.h>

// ============================================================
// 寄存器地址（与 modbus_registers 对应）
// 格式：高字节=小时，低字节=分钟
// 例：0x1500 = 21:00，0x0600 = 06:00
// ============================================================
#define STATUS_POWER_OFF_TIME   111   // reg[111]：关机时间
#define STATUS_POWER_ON_TIME    112   // reg[112]：开机时间

#define POWER_OFF_DEFAULT    ((21 << 8) | 0)   // 默认 21:00 关机
#define POWER_ON_DEFAULT     ((6  << 8) | 0)   // 默认 06:00 开机

// ============================================================
// 对外接口
// ============================================================

// 初始化：设置寄存器默认值，检测是否从待机唤醒
void rtc_power_init(void);

// GPS同步成功后调用：把UTC时间写入RTC
void rtc_sync_from_gps(uint8_t utc_h, uint8_t utc_m, uint8_t utc_s,
                        uint8_t year, uint8_t month, uint8_t date);

// 每10s调用一次：检查是否到关机时间
void rtc_power_schedule_check(void);

// 是否从待机唤醒（在main里上电时判断）
uint8_t rtc_is_wakeup_from_standby(void);

// GPS是否已同步（同步后才允许关机）
uint8_t rtc_is_gps_synced(void);

#endif // STANDBY_CHECK_APP_H