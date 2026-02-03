#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"
#include "cmsis_os.h"
#include "task.h"

#include "Modbus.h"

// 函数声明
void init_read_encoder_task();
#ifdef ENABLE_HUB_SLAVE
// Slave功能
void init_encoder_forward_slave(void);
void StartReadEncoderTask(void *argument);
void encoder_update_multi(uint8_t id, uint16_t high, uint16_t low, bool state);
#endif

#ifdef ENABLE_HUB_MASTER
// Master功能
void init_encoder_forward_master(void);
void StartNonBlockReadEncoderForwardTask(void *argument);
void StartBlockReadEncoderForwardTask(void *argument);
#endif

#ifdef __cplusplus
}
#endif


