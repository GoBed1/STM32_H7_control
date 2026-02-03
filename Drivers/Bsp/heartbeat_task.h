/*
* heartbeat_task.h
*
*  Created on: Sep 9, 2025
*      Author: river
*/

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led_driver.h"

#ifndef BSP_HEARTBEAT_TASK_H_
#define BSP_HEARTBEAT_TASK_H_

#define STATE_ONLINE 0
#define STATE_OFFLINE 1

/* system protection uses event 0. */
#define SYSTEM_PROTECT NO_OFFLINE
#define OFFLINE_ERROR_LEVEL 0
#define OFFLINE_WARNING_LEVEL 1
#define APP_PROTECT_LEVEL 2

#define BEEP_DISABLE 0xFF

// LED控制函数指针类型
typedef void (*heartbeat_led_func_t)(void); // 心跳灯（翻转）
typedef void (*offline_led_func_t)(int on); // 离线灯（开关）

typedef void (*offline_t)(void);

typedef enum
{
    NO_OFFLINE = 0, /*!< system is normal */
	OFFLINE_ENCODER_TASK1,
	OFFLINE_ENCODER_TASK2,
	OFFLINE_ENCODER_TASK3,
	OFFLINE_ENCODER_FORWARD,
    OFFLINE_EVENT_MAX_NUM,
} offline_event;

struct offline_event_obj
{
    offline_event event;
    uint8_t enable;
    uint8_t level;
    uint8_t beep_times;
    uint32_t offline_time;
    offline_t offline_first_func;
    offline_t offline_func;
    offline_t online_first_func;
    offline_t online_func;
};

struct offline_manage_obj
{
    offline_event event;
    uint8_t enable;
    uint8_t online_state;
    uint8_t last_state;
    uint8_t error_level;
    offline_t offline_first_func;
    offline_t offline_func;
    offline_t online_first_func;
    offline_t online_func;

    /* if offline event number is more than 1, beep_times equal minimum of all events. */
    /* max value is 0xFE */
    uint8_t beep_times;
    uint32_t last_time;
    uint32_t offline_time;
};


// 注册心跳灯控制函数
void register_heartbeat_led_func(heartbeat_led_func_t func);
void register_offline_led_func(offline_led_func_t func);

void offline_service_task_init(void);
void offline_event_init(struct offline_manage_obj obj);
void offline_event_time_update(offline_event event);
void offline_event_enable(offline_event event);
void offline_event_disable(offline_event event);
void StartHeartbeatTask(void *argument);

uint8_t get_system_status(void);
void init_heartbeat();
void update_heartbeat();

#endif /* BSP_HEARTBEAT_TASK_H_ */
