#include "gps_app.h"
//#include "board_manage.h"
#include "nmea.h"

//#define LOGD(...) // SYSLOG_DEBUG("GPS",__VA_ARGS__)
//#define LOGI(...) // SYSLOG_INFO("GPS",__VA_ARGS__)
//#define LOGE(...) // SYSLOG_ERR("GPS",__VA_ARGS__)
#define LOGD(...) printf(__VA_ARGS__)
#define LOGI(...) printf(__VA_ARGS__)
#define LOGE(...) printf(__VA_ARGS__)

#define TSET_GPS_NMEA_PARSER 0

#define WT_RTK_UM982  1
#define WT_GPS_UM626N 2
#define WT_GPS_6N     3
#ifndef GPS_TYPE_STD
#define GPS_TYPE_STD WT_GPS_UM626N
#endif

static void gps_print_nmea_data(const char *tag)
{
    LOGD("[%s] fix=%u sat=%u view=%u mode=%u time=%02u:%02u:%02u date=%04u-%02u-%02u\r\n",
        tag,
        g_nmea_gnss.fix_quality,
        g_nmea_gnss.satellite,
        g_nmea_gnss.satellite_in_view,
        g_nmea_gnss.fix_mode,
        g_nmea_gnss.time_h,
        g_nmea_gnss.time_m,
        g_nmea_gnss.time_s,
        g_nmea_gnss.date_year,
        g_nmea_gnss.date_m,
        g_nmea_gnss.date_d);

    LOGD("[%s] lat=%.6f lon=%.6f alt=%.2f hdop=%.2f pdop=%.2f vdop=%.2f spd=%.2fkn/%.2fkmh cog=%.2f mask=0x%08lX\r\n",
        tag,
        g_nmea_gnss.latitude_deg,
        g_nmea_gnss.longitude_deg,
        g_nmea_gnss.altitude_m,
        g_nmea_gnss.precision_m,
        g_nmea_gnss.pdop_m,
        g_nmea_gnss.vdop_m,
        g_nmea_gnss.speed_knots,
        g_nmea_gnss.speed_kmh,
        g_nmea_gnss.course_deg,
        (unsigned long)g_nmea_gnss.sentence_mask);
}

/*
TEST OUTPUT:
NMEA test[1] OK
NMEA test[2] OK
NMEA test[3] OK
NMEA test[4] OK
NMEA test[5] OK
NMEA test[6] OK
NMEA test[7] OK
NMEA test[8] OK
[TEST] fix=1 sat=8 view=8 mode=3 time=07:32:37 date=2026-03-02
[TEST] lat=22.426970 lon=114.208656 alt=75.88 hdop=1.00 pdop=1.80 vdop=1.50 spd=0.12kn/0.22kmh cog=45.60 mask=0x000000FF
 */
static void gps_test_nmea_parser(void)
{
    static const char *test_sentences[] = {
        "$GNGGA,073237.00,2225.61814,N,11412.51906,E,1,08,1.7,75.88,M,-2.36,M,,*57\r\n",
        "$GNGLL,2225.61814,N,11412.51906,E,073237.00,A,A*74\r\n",
        "$GNGSA,A,3,08,10,23,16,27,03,14,30,,,,1.8,1.0,1.5*03\r\n",
        "$GNGSV,2,1,08,03,15,120,35,08,45,067,42,10,32,250,40,14,22,300,37*61\r\n",
        "$GNRMC,073237.00,A,2225.61814,N,11412.51906,E,0.12,45.6,020326,,,A*42\r\n",
        "$GNVTG,45.6,T,,M,0.12,N,0.22,K,A*27\r\n",
        "$GNZDA,073237.00,02,03,2026,00,00*7D\r\n",
        "$GNTXT,01,01,02,ANTENNA OK*28\r\n",
    };

    uint32_t ok_count = 0;
    for (uint32_t i = 0; i < (uint32_t)(sizeof(test_sentences) / sizeof(test_sentences[0])); i++)
    {
        const uint8_t *s = (const uint8_t *)test_sentences[i];
        uint16_t n = (uint16_t)strlen(test_sentences[i]);
        int ret = nmea_parse(s, n);
        if (ret == NMEA_OK)
        {
            ok_count++;
            LOGI("NMEA test[%lu] OK\r\n", (unsigned long)(i + 1U));
            
        }
        else
        {
            LOGE("NMEA test[%lu] FAIL: %d\r\n", (unsigned long)(i + 1U), ret);
        }
    }
    gps_print_nmea_data("TEST");
}

// GPS 时间设置函数
// 首次获取时间时，设置RTC并标记时间源；后续只更新GPS缓存
void gps_set_unix_time(uint32_t unix_seconds)
{
//    if (unix_seconds == 0)
//    {
//        return;
//    }
//
//    // 验证时间合理性（2000-2100年范围）
//    if (unix_seconds < 946684800 || unix_seconds > 4102444800UL)
//    {
//        return;
//    }
//
//    // 首次获取时间时，设置RTC并标记时间源
//    if (!rtc_updated_flag)
//    {
//        if (board_rtc_set_unix_time(unix_seconds))
//        {
//            rtc_updated_flag = true;
//            board_rtc_mark_source(BOARD_RTC_SOURCE_GPS);
//            SYSLOG_INFO(SYSLOG_TAG_GPS, "RTC initialized by GPS");
//        }
//    }
//    else
//    {
//        // RTC已设置，仅更新GPS时间源缓存
//        board_time_update_source(BOARD_RTC_SOURCE_GPS, unix_seconds);
//    }
}

void config_gps_app(void)
{
   (void)uart_manage_enable_dma_recv_by_name("gps");

   osDelay(1000);

#if (GPS_TYPE_STD == WT_RTK_UM982)
    osDelay(10);
    const char cfgmsg_gga[] = "GPGGA COM1 100\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t*)cfgmsg_gga, sizeof(cfgmsg_gga) - 1U);
    osDelay(10);
    const char cfgmsg_rmc[] = "GPRMC COM1 1\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t*)cfgmsg_rmc, sizeof(cfgmsg_rmc) - 1U);
    osDelay(10);
    const char cfgmsg_save[] = "SAVECONFIG\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t*)cfgmsg_save, sizeof(cfgmsg_save) - 1U);
    osDelay(10);
#elif (GPS_TYPE_STD == WT_GPS_UM626N)
    // 只启用RMC消息，关闭其他所有消息
    const char cfgmsg_gga[] = "$CFGMSG,0,0,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t*)cfgmsg_gga, sizeof(cfgmsg_gga) - 1U);
    osDelay(10);
    const char cfgmsg_gll[] = "$CFGMSG,0,1,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t*)cfgmsg_gll, sizeof(cfgmsg_gll) - 1U);
    osDelay(10);
    const char cfgmsg_gsa[] = "$CFGMSG,0,2,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t*)cfgmsg_gsa, sizeof(cfgmsg_gsa) - 1U);
    osDelay(10);
    const char cfgmsg_gsv[] = "$CFGMSG,0,3,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t*)cfgmsg_gsv, sizeof(cfgmsg_gsv) - 1U);
    osDelay(10);
    const char cfgmsg_rmc[] = "$CFGMSG,0,4,1\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t*)cfgmsg_rmc, sizeof(cfgmsg_rmc) - 1U);
    osDelay(10);
    const char cfgmsg_vtg[] = "$CFGMSG,0,5,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t*)cfgmsg_vtg, sizeof(cfgmsg_vtg) - 1U);
    osDelay(10);
    const char cfgmsg_zda[] = "$CFGMSG,0,6,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t*)cfgmsg_zda, sizeof(cfgmsg_zda) - 1U);
    osDelay(10);
    const char cfgmsg_gst[] = "$CFGMSG,0,7,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t*)cfgmsg_gst, sizeof(cfgmsg_gst) - 1U);
    osDelay(10);
#elif (GPS_TYPE_STD == WT_GPS_6N)
    const char cfgmsg_freq[] = "$PCAS03,1,0,0,0,0,0,0,0*03\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t*)cfgmsg_freq, sizeof(cfgmsg_freq) - 1U);
    osDelay(10);
    const char cfgmsg_save[] = "$PCAS00*01\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t*)cfgmsg_save, sizeof(cfgmsg_save) - 1U);
    osDelay(10);
#endif
}

void update_gps_app(void)
{
#if TSET_GPS_NMEA_PARSER
    gps_test_nmea_parser();
    osDelay(1000);
    return;
#endif

    uart_inferface_t *m_obj = uart_manage_get_obj_by_name("gps");
    if (m_obj == NULL)
    {
        LOGE("GPS uart interface not found\r\n");
        return;
    }

    lwrb_sz_t available = lwrb_get_full(&m_obj->process_ring_buffer);
    if (available == 0)
    {
        return;
    }
    uint8_t to_read_buffer[128];
    lwrb_sz_t to_read = (available > sizeof(to_read_buffer)) ? sizeof(to_read_buffer) : available;
    lwrb_sz_t read_size = lwrb_read(&m_obj->process_ring_buffer, to_read_buffer, to_read);
    
    if (read_size > 0)
    {   
        int parse_result = nmea_parse(to_read_buffer, (uint16_t)read_size);
        if (parse_result != NMEA_OK)
        {
            LOGE("[PE%d]%.*s\r\n",parse_result,(int)read_size, (const char *)to_read_buffer);
        }else{
            // RMC消息解析成功，检查是否有有效的时间和日期数据
            // RMC包含：UTC time (field 1) 和 Date (field 7)
            if (g_nmea_gnss.date_year > 0 && g_nmea_gnss.date_m > 0 && g_nmea_gnss.date_d > 0)
            {
                LOGD("RMC: fix=%u time=%02u:%02u:%02u date=%04u-%02u-%02u\r\n",
                    g_nmea_gnss.fix_quality,
                    g_nmea_gnss.time_h,
                    g_nmea_gnss.time_m,
                    g_nmea_gnss.time_s,
                    g_nmea_gnss.date_year,
                    g_nmea_gnss.date_m,
                    g_nmea_gnss.date_d);
                
                // 将GPS解析到的日期时间转换为Unix时间戳
                uint16_t year = g_nmea_gnss.date_year;
                uint8_t month = g_nmea_gnss.date_m;
                uint8_t day = g_nmea_gnss.date_d;
                uint8_t hour = g_nmea_gnss.time_h;
                uint8_t minute = g_nmea_gnss.time_m;
                uint8_t second = g_nmea_gnss.time_s;
                
                // 计算Unix时间戳
                uint64_t days = 0;
                for (uint16_t y = 1970; y < year; ++y)
                {
                    bool is_leap = ((y % 4U == 0U && y % 100U != 0U) || (y % 400U == 0U));
                    days += is_leap ? 366ULL : 365ULL;
                }
                
                uint8_t dim[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
                bool is_leap_year = ((year % 4U == 0U && year % 100U != 0U) || (year % 400U == 0U));
                if (is_leap_year)
                {
                    dim[1] = 29;
                }
                
                for (uint8_t m = 1; m < month; ++m)
                {
                    days += (uint64_t)dim[m - 1];
                }
                days += (uint64_t)(day - 1U);
                
                uint64_t sec64 = days * 86400ULL + (uint64_t)hour * 3600ULL + (uint64_t)minute * 60ULL + (uint64_t)second;
                uint32_t unix_seconds = (uint32_t)sec64;
                
                // 更新GPS时间
                gps_set_unix_time(unix_seconds);
            }
            else
            {
                LOGD("RMC: Invalid date/time data\r\n");
            }
        }
    }
}
