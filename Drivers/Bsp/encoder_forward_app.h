#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "task.h"

// 寄存器映射定义
// 保持寄存器 (Holding Registers) - 可读写
#define REG_SYSTEM_VERSION   0x0000  // 系统版本号
#define REG_RESERVE_01       0x0001  // 保留

#define REG_TOTAL_SENSORS 0x0002  // 传感器总数
#define REG_ERROR_CODE    0x0003  // 错误码

#define REG_SENSOR1_POS_HI    0x0004  // 传感器1位置高16位
#define REG_SENSOR1_POS_LO    0x0005  // 传感器1位置低16位

#define REG_SENSOR2_POS_HI    0x0006  // 传感器2位置高16位
#define REG_SENSOR2_POS_LO    0x0007  // 传感器2位置低16位

#define REG_SENSOR3_POS_HI    0x0008  // 传感器3位置高16位
#define REG_SENSOR3_POS_LO    0x0009  // 传感器3位置低16位

// 寄存器总数
#define REGS_TOTAL_NUM    256

// 函数声明
#ifdef ENABLE_HUB_SLAVE
// Slave功能
void init_encoder_forward_slave(void);
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
