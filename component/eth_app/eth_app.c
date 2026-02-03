#include "eth_app.h"

#include <stdbool.h>

#include "main.h"
#include "cmsis_os.h"
#include "lwip/dns.h"
#include "lwip/apps/sntp.h"
#include "lwip/err.h"

#include "syslog.h"

// External variables from main.cpp
extern volatile bool lwip_sntp_timestamp_received;
extern volatile uint32_t lwip_sntp_timestamp;
extern volatile bool rtc_time_refreshed;

static rtc_update_callback_t rtc_update_callback = NULL;

void eth_register_rtc_update_callback(rtc_update_callback_t callback)
{
  rtc_update_callback = callback;
}

void dns_gethostbyname_callback(const char *name, const ip_addr_t *ipaddr, void *arg)
{
  ip4_addr_t *result_ip = (ip4_addr_t *)arg;
  if (ipaddr != NULL)
  {
    *result_ip = *ip_2_ip4(ipaddr);
  }
  else
  {
    result_ip->addr = 0x00000000;
  }
}

ip4_addr_t dns_get_ip_by_hostname(const char *hostname)
{
  ip4_addr_t ip = {0x00000000};
  err_t err;
  LOCK_TCPIP_CORE();
  // HACK ip是局部变量，生命周期只在该函数内，tcpip修改其值可能会出现地址无效的问题
  err = dns_gethostbyname(hostname, (ip_addr_t *)&ip, dns_gethostbyname_callback, &ip);
  UNLOCK_TCPIP_CORE();
  osDelay(10);
  
  if (err == ERR_OK)
  {
    LOG_DEBUG("ETH", "DNS success: %s -> %s", hostname, ip4addr_ntoa(&ip));
    return ip;
  }
  else if (err == ERR_INPROGRESS)
  {
    LOG_DEBUG("ETH", "DNS waiting...");
    for (int j = 0; j < 50; j++)
    { 
      // 加锁保护读取
      LOCK_TCPIP_CORE();
      bool resolved = (ip.addr != 0x00000000);
      UNLOCK_TCPIP_CORE();
      if (resolved)
      {
        LOG_INFO("ETH", "DNS resolved: %s -> %s", 
                    hostname, ip4addr_ntoa(&ip));
        return ip;
      }
      osDelay(10);
    }
  }
  else
  {
    LOG_WARNING("ETH", "[%d]DNS resolution failure", err);
  }

  if (ip.addr == 0x00000000)
  {
    LOG_WARNING("ETH", "DNS resolution failed or timeout for %s", hostname);
  }
  return ip;
}

void sntp_time_received_callback(uint32_t timestamp)
{
  lwip_sntp_timestamp_received = true;
  lwip_sntp_timestamp = timestamp;
}

void set_sntp_time(uint32_t timestamp)
{
  const uint32_t SECONDS_PER_DAY = 86400;
  const uint32_t SECONDS_PER_HOUR = 3600;
  const uint32_t SECONDS_PER_MINUTE = 60;
  const uint32_t GMT8_OFFSET = 8 * SECONDS_PER_HOUR;

  // 验证时间戳合理性（避免无效时间）
  if (timestamp < 946684800 || timestamp > 4070908800UL)
  { // 2000-2099年范围
    return;
  }

  // 使用简单的时间转换，避免C++标准库函数
  // Unix时间戳基准：1970年1月1日00:00:00 UTC
  // 先用UTC计算天数（用于年月日和星期计算）
  uint32_t days = timestamp / SECONDS_PER_DAY;
  uint32_t seconds_today = timestamp % SECONDS_PER_DAY;

  // 加GMT+8时区偏移（只影响时分秒）
  seconds_today += GMT8_OFFSET;
  if (seconds_today >= SECONDS_PER_DAY)
  {
    days += 1;
    seconds_today -= SECONDS_PER_DAY;
  }

  // 计算年、月、日
  uint32_t year = 1970;
  uint32_t days_in_year;

  while (true)
  {
    bool is_leap_year = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    days_in_year = is_leap_year ? 366 : 365;
    if (days < days_in_year) break;
    days -= days_in_year;
    year++;
  }

  // 当前年份的闰年状态
  bool is_leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);

  // 月份天数表
  uint8_t days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (is_leap)
  {
    days_in_month[1] = 29; // 闰年二月29天
  }

  uint32_t month = 1;
  while (days >= days_in_month[month - 1])
  {
    days -= days_in_month[month - 1];
    month++;
    // 边界保护
    if (month > 12) return;
  }
  uint32_t day = days + 1;

  // 计算时、分、秒
  uint32_t hour = seconds_today / SECONDS_PER_HOUR;
  uint32_t minute = (seconds_today % SECONDS_PER_HOUR) / SECONDS_PER_MINUTE;
  uint32_t second = seconds_today % SECONDS_PER_MINUTE;

  // 计算星期几（使用已调整时区的days，1970年1月1日是星期四）
  uint32_t weekday = (days + 4) % 7; // 0=Sunday, 1=Monday, ...

  if (rtc_update_callback != NULL)
  {
    rtc_update_callback((uint16_t)year, (uint8_t)month, (uint8_t)day, 
                  (uint8_t)hour, (uint8_t)minute, (uint8_t)second, 
                  (uint8_t)weekday);
  }
}

void sntp_request_immediate(void)
{
  LOCK_TCPIP_CORE();
  sntp_stop();
  UNLOCK_TCPIP_CORE();

  // 等待一会儿确保停止完成
  osDelay(100);

  LOCK_TCPIP_CORE();
  // 重新初始化SNTP以触发立即请求
  sntp_init();
  UNLOCK_TCPIP_CORE();
}
