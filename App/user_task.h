#ifndef _USER_TASK_H
#define _USER_TASK_H

#include "main.h"
#include "hardware.h"
#include "uart_app.h"
#include "nex_modbus_rtu_client.h"
#include "port.h"
#ifdef __cplusplus
extern "C" {
#endif

#define HEARTBEAT_LED_PORT  LED0_GPIO_Port
#define HEARTBEAT_LED_PIN   LED0_Pin

void init_user_task(void);
void start_user_task(void *argument);
void SOUND_LED_task(void *argument);
void YX95R_LED_task(void *argument);
void ModbusRecv_task(void *argument);

void RecvMaster_task(void *argument);

void ReadSoc_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
