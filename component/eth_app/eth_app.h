#ifndef __ETH_APP_H__
#define __ETH_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "lwip/ip4_addr.h"
#include "lwip/ip_addr.h"

// DNS functions
void dns_gethostbyname_callback(const char *name, const ip_addr_t *ipaddr, void *arg);
ip4_addr_t dns_get_ip_by_hostname(const char *hostname);

// SNTP functions
typedef void (*rtc_update_callback_t)(uint16_t year, uint8_t month, uint8_t day, 
                                     uint8_t hour, uint8_t minute, uint8_t second, 
                                     uint8_t weekday);

void sntp_time_received_callback(uint32_t timestamp);
void set_sntp_time(uint32_t timestamp);
void sntp_request_immediate(void);
void eth_register_rtc_update_callback(rtc_update_callback_t setter);

#ifdef __cplusplus
}
#endif

#endif /* __ETH_APP_H__ */
