#ifndef LAN8742A_MODBUS_TCP_H
#define LAN8742A_MODBUS_TCP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * FreeRTOS task that runs a Modbus-TCP slave on the on-chip LAN8742A Ethernet
 * interface (lwIP socket layer). Create this task once the lwIP stack and
 * network interface are fully initialised.
 */
void lan8742a_modbus_tcp_server_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* LAN8742A_MODBUS_TCP_H */ 