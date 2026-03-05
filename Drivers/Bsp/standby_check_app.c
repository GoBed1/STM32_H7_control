#include "standby_check_app.h"
// #include "read_encoder_task.h"  // 访问 modbus_registers[]
#include "stm32h7xx_hal.h"

extern RTC_HandleTypeDef hrtc;
extern uint16_t modbus_registers[];

// 外部声明：关闭外设（LED+喇叭），定义在 read_encoder_task.c
extern void shutdown_all_peripherals(void);

// GPS 同步标志
static uint8_t s_gps_synced = 0;

// ============================================================
// 内部函数：设置 RTC Alarm B（UTC时间）
// ============================================================
static void _set_alarm_b(uint8_t utc_h, uint8_t utc_m, uint8_t utc_s)
{
    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);

    RTC_AlarmTypeDef sAlarm = {0};
    sAlarm.AlarmTime.Hours          = utc_h;
    sAlarm.AlarmTime.Minutes        = utc_m;
    sAlarm.AlarmTime.Seconds        = utc_s;
    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    sAlarm.AlarmMask                = RTC_ALARMMASK_DATEWEEKDAY; // 忽略日期，每天触发
    sAlarm.AlarmSubSecondMask       = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmDateWeekDaySel      = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay         = 1;
    sAlarm.Alarm                    = RTC_ALARM_B;

    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
    {
        printf("[RTC] Alarm B set FAILED!\r\n");
    }
    else
    {
        printf("[RTC] Alarm B set: UTC %02d:%02d:%02d\r\n", utc_h, utc_m, utc_s);
    }
}

// ============================================================
// 内部函数：进入待机
// ============================================================
static void _enter_standby(void)
{
    printf("[RTC] Entering STANDBY...\r\n");
    osDelay(200); // 等串口打印完

    // 清除唤醒/待机标志
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRBF);

    // 使能 RTC 唤醒源
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); // 可选，有按键唤醒时开启

    // 进入待机模式，不返回
    HAL_PWR_EnterSTANDBYMode();
}

// 是否从待机唤醒
uint8_t rtc_is_wakeup_from_standby(void)
{
    // STM32H7 用 PWR_FLAG_SB_D1，其他系列用 PWR_FLAG_SB
    return (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) ? 1 : 0;
}

// ============================================================
// GPS 是否已同步
// ============================================================
uint8_t rtc_is_gps_synced(void)
{
    return s_gps_synced;
}

// ============================================================
// 初始化：设默认值 + 检测唤醒来源
// ============================================================
void rtc_power_init(void)
{
    // 写默认关机/开机时间到寄存器（只有寄存器为0时才写，防止覆盖外部设置）
    if (modbus_registers[STATUS_POWER_OFF_TIME] == 0)
        modbus_registers[STATUS_POWER_OFF_TIME] = POWER_OFF_DEFAULT;
    if (modbus_registers[STATUS_POWER_ON_TIME] == 0)
        modbus_registers[STATUS_POWER_ON_TIME] = POWER_ON_DEFAULT;

    if (rtc_is_wakeup_from_standby())
    {
        printf("[RTC] Wakeup from STANDBY, normal boot.\r\n");
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    }
    else
    {
        printf("[RTC] Cold boot.\r\n");
    }
}

// ============================================================
// GPS 同步成功后调用（把GPS的UTC时间写入RTC）
// utc_h/m/s：GPS给的UTC时间
// year：2位年份，如25表示2025
// ============================================================
void rtc_sync_from_gps(uint8_t utc_h, uint8_t utc_m, uint8_t utc_s,
                        uint8_t year, uint8_t month, uint8_t date)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours   = utc_h;
    sTime.Minutes = utc_m;
    sTime.Seconds = utc_s;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    sDate.Year  = year;
    sDate.Month = month;
    sDate.Date  = date;
    sDate.WeekDay = RTC_WEEKDAY_MONDAY; // 无所谓

    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    s_gps_synced = 1;
    printf("[RTC] Synced from GPS: UTC %02d:%02d:%02d  %04d-%02d-%02d\r\n",
           utc_h, utc_m, utc_s, 2000 + year, month, date);
}

// ============================================================
// 每 10s 调用一次：检查是否到关机时间
// ============================================================
void rtc_power_schedule_check(void)
{
    // GPS未同步，不执行（防止时间不准就关机）
    if (!s_gps_synced)
    {
        printf("[RTC] GPS not synced, skip power check.\r\n");
        return;
    }

     // 从 GPS 解析结果取实时时间（UTC）
    uint8_t utc_h = g_nmea_gnss.time_h;
    uint8_t utc_m = g_nmea_gnss.time_m;

    // 转北京时间
    uint8_t beijing_h = (utc_h + 8) % 24;
    uint8_t beijing_m = utc_m;
    uint16_t now_hhmm = (uint16_t)((beijing_h << 8) | beijing_m);

    // 读关机/开机寄存器
    uint16_t off_hhmm = modbus_registers[STATUS_POWER_OFF_TIME]; // reg[111]
    uint16_t on_hhmm  = modbus_registers[STATUS_POWER_ON_TIME];  // reg[112]

    printf("[PWR] Beijing %02d:%02d | OFF=%02d:%02d ON=%02d:%02d\r\n",
           beijing_h, beijing_m,
           off_hhmm >> 8, off_hhmm & 0xFF,
           on_hhmm  >> 8, on_hhmm  & 0xFF);

    // 精确匹配关机时间
    if (now_hhmm == off_hhmm)
    {
        printf("[PWR] >>> 到达关机时间 %02d:%02d，关机! <<<\r\n",
               beijing_h, beijing_m);

        shutdown_all_peripherals();

        // 开机时间 北京→UTC
        uint8_t on_h_utc = ((on_hhmm >> 8) + 24 - 8) % 24;
        uint8_t on_m     = (uint8_t)(on_hhmm & 0xFF);
        _set_alarm_b(on_h_utc, on_m, 0);

        _enter_standby(); // 不返回
    }
}

