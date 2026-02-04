#include "board.h"
#include <stdio.h>
#include "printf_redirect.h"
#include "async_logger.h"

#include <main.h>
#include "cmsis_os.h"
#include <usart.h>
#include "FreeRTOS.h"
#include "task.h"

#include "led_driver.h"
#include "read_encoder_task.h"
#include "heartbeat_task.h"
// #include "read_encoder_task.h"
#ifdef ENABLE_ERROR_CODE_RATE_TEST
#include <test_communication.h>
#endif

uint32_t get_time_ms(void)
{
#if defined(FREERTOS) || defined(USE_FREERTOS) || defined(configUSE_PREEMPTION)
    // 判断是否在中断中
    if (__get_IPSR() != 0) {
        // 在中断
        return (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
    } else {
        // 任务上下文
        return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    }
#else
    // 非FreeRTOS环境,HAL_GetTick()本身就是线程安全的，可以在中断服务函数（ISR）中直接调用
    return HAL_GetTick();
#endif
}

// #include "modbus_rtu.hpp"
// extern ModbusRtu bms_led_sound_app;

void show_heartbeat(void) {
    toggle_led("led1");
}

// 适配心跳灯
void heartbeat_led_adapter(void) {
    toggle_led("led1");
}

// 适配离线灯
void offline_led_adapter(int on) {
    set_led("led2", on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

osThreadId_t HeartbeatTaskHandle;
const osThreadAttr_t HeartbeatTask_attributes = {
  .name = "HeartbeatTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void init_app(void)
{
  // register LED
  register_led("led1", LED1_PORT, LED1_PIN);
  register_led("led2", LED2_PORT, LED2_PIN);

  specify_redirect_uart(&huart1);
  printf_set_mutex(printf_create_mutex());
  printf_use_mutex(1);
  (void)async_logger_init(&huart1); 
  printf("\r\n[INFO] [BOARD] specify redirect printf to huart1\r\n");

#ifdef ENABLE_ENCODER_TASK
  init_read_encoder_task();
  printf("[INFO] [BOARD] ENABLE_ENCODER_TASK\r\n");
#ifdef ENABLE_ERROR_CODE_RATE_TEST
  init_lora_testing();
  create_lora_testing_task();
  printf("[INFO] [BOARD] ENABLE_ERROR_CODE_RATE_TEST\r\n");
#endif
#endif

#ifdef ENABLE_HEARTBEAT_TASK
  register_heartbeat_led_func(heartbeat_led_adapter);
  register_offline_led_func(offline_led_adapter);
// bms_led_sound_app.
// uint16_t bms_results[10] = {0};
//  bms_led_sound_app.asyncReadHolding(0x04, 0x0000, 1, bms_results);
  HeartbeatTaskHandle = osThreadNew(StartHeartbeatTask, NULL, &HeartbeatTask_attributes);
  printf("[INFO] [BOARD] ENABLE_HEARTBEAT_TASK\r\n");
#endif
}
