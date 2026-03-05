#ifndef GPS_APP_H
#define GPS_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "uart_manage.h"

extern uart_inferface_t gps_app;

void config_gps_app(void);
void update_gps_app(void);

#ifdef __cplusplus
}
#endif

#endif // GPS_APP_H
