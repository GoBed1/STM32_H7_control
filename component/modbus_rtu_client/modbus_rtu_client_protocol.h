#ifndef MODBUS_RTU_CLIENT_PROTOCOL_H
#define MODBUS_RTU_CLIENT_PROTOCOL_H

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "timers.h"
#include <stdint.h>

#define MODBUS_MAX_FRAME_SIZE  256

// Modbus 错误码
typedef enum {
    MODBUS_OK = 0,              // 成功
    MODBUS_ERR_TIMEOUT,         // 超时
    MODBUS_ERR_CRC,             // CRC 校验失败
    MODBUS_ERR_FRAME_LEN,       // 帧长度错误
    MODBUS_ERR_SEND_FAIL,       // 发送失败
    MODBUS_ERR_PARSE_FAIL,      // 解析失败
} modbus_error_code_t;

typedef struct modbus_rtu_protocol_t modbus_rtu_protocol_t;

typedef int (*modbus_send_callback_t)(const uint8_t *data, uint16_t len);

struct modbus_rtu_protocol_t {
    uint8_t server_id;
    osThreadId_t thread_id;
    TimerHandle_t timeout_timer;
    modbus_send_callback_t sender;  
    uint8_t recv_packet[MODBUS_MAX_FRAME_SIZE];
    uint16_t recv_packet_len;
    volatile modbus_error_code_t error_code;
};

// 基本配置：设置从站地址并复位状态
void modbus_rtu_client_config_param(modbus_rtu_protocol_t *self, uint8_t server_id);

// 注册底层发送函数（返回 0 表示成功）
void modbus_rtu_client_set_sender(modbus_rtu_protocol_t *self,
                                  modbus_send_callback_t sender);

// 创建一次性超时定时器（毫秒）
void modbus_rtu_client_create_timeout_timer(modbus_rtu_protocol_t *self,
                                            uint32_t timeout_ms);

// 设置当前线程句柄，用于同步等待
void modbus_rtu_client_set_thread_id(modbus_rtu_protocol_t *self, osThreadId_t thread_id);

// UART 空闲事件回调（用于接收数据）
void modbus_rtu_client_recv_event(void *self, uint8_t *data, uint16_t len);

// 计算 CRC16
uint16_t modbus_rtu_calculate_crc(const uint8_t *data, uint16_t len);

// 同步读取保持寄存器/输入寄存器（目前支持功能码 0x03）
int modbus_rtu_read_sync(modbus_rtu_protocol_t *self, uint8_t func_code,
                         uint16_t start_addr, uint16_t quantity);

#endif // MODBUS_RTU_CLIENT_PROTOCOL_H