#include "gps_app.h"
#include "nmea.h"

void init_gps_app(void)
{
    (void)uart_manage_enable_dma_recv_by_name("gps");

   osDelay(1000);
   const char cfgmsg_gga[] = "$CFGMSG,0,0,1\r\n";
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
    const char cfgmsg_rmc[] = "$CFGMSG,0,4,0\r\n";
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
}

void process_gps_app(void)
{
    uart_inferface_t *m_obj = uart_manage_get_obj_by_name("gps");
    if (m_obj == NULL)
    {
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

        uart_manage_dma_send_by_name("echo", to_read_buffer, (uint16_t)read_size);
        int parse_result = nmea_parse(to_read_buffer, (uint16_t)read_size);
        if (parse_result != NMEA_OK)
        {
            printf("NMEA parse error: %d\r\n", parse_result);
        }else{
            printf("fix=%u sat=%u time=%02u:%02u:%02u lat=%.6f lon=%.6f alt=%.2f hdop=%.2f\r\n",
                g_nmea_gnss.fix_quality,
                g_nmea_gnss.satellite,
                g_nmea_gnss.time_h,
                g_nmea_gnss.time_m,
                g_nmea_gnss.time_s,
                g_nmea_gnss.latitude_deg,
                g_nmea_gnss.longitude_deg,
                g_nmea_gnss.altitude_m,
                g_nmea_gnss.precision_m);
        }
    }
}
