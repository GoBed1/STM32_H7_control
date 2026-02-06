#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"
#include "cmsis_os.h"
#include "task.h"

#include "Modbus.h"

// extern ModbusRtu bms_led_sound_app;
// 函数声明
void init_read_encoder_task();
#ifdef ENABLE_HUB_SLAVE
// Slave功能
void init_encoder_forward_slave(void);
void StartReadEncoderTask(void *argument);
void encoder_update_multi(uint8_t id, uint16_t high, uint16_t low, bool state);
#endif
typedef enum
{
    BUZZER_OFF = 0,
    BUZZER_7M = 1,
    BUZZER_3M = 2,
} buzzer_mode_t;




#ifdef ENABLE_HUB_MASTER
// Master功能
void init_encoder_forward_master(void);
void StartNonBlockReadEncoderForwardTask(void *argument);
void StartBlockReadEncoderForwardTask(void *argument);
#endif

#ifdef __cplusplus
}
#endif


