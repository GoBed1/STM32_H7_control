
#ifdef ENABLE_ERROR_CODE_RATE_TEST

#ifndef UART_ERROR_CODE_RATE_H
#define UART_ERROR_CODE_RATE_H

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif
// UART7 接收：双缓冲大小定义（供中断与任务共享）
#ifndef UART7_RX_BUFFER_SIZE
#define UART7_RX_BUFFER_SIZE 128
#endif

// 供中断与任务访问的共享变量（由comm_testing.c定义）
extern uint8_t uart7_rx_buffer[2][UART7_RX_BUFFER_SIZE];
extern volatile uint8_t uart7_rx_active_idx;
extern volatile uint16_t uart7_rx_len;
extern volatile uint8_t uart7_rx_idle_flag;
extern uint8_t uart7_rx_ready_buffer[UART7_RX_BUFFER_SIZE];

// 中断处理入口（在UART7_IRQHandler中调用）
void comm_testing_uart7_idle_irq_handler(void);

// 误码率测试配置参数
#define ECR_PACKET_SIZE          23      // 测试数据包大小（字节）：帧头1+序号1+地址1+指令1+长度1+数据16+CRC2
#define ECR_MAX_PACKETS          1000    // 最大测试包数量
#define ECR_TIMEOUT_MS           1000    // 接收超时时间（毫秒）
#define ECR_SEND_INTERVAL_MS     500     // 发送间隔时间（毫秒）

// 数据帧格式定义
#define ECR_FRAME_HEADER         0xF5    // 帧头
#define ECR_PACKET_NUM_LENGTH    1       // 包序号长度
#define ECR_ADDRESS_LENGTH       1       // 地址码长度
#define ECR_COMMAND_LENGTH       1       // 指令码长度
#define ECR_DATA_LENGTH_LENGTH   1       // 数据段长度字段长度
#define ECR_DATA_LENGTH          16      // 数据段长度（0x00-0x0F）
#define ECR_CRC_LENGTH           2       // CRC校验长度

// 误码率测试状态
typedef enum {
    ECR_STATE_IDLE = 0,
    ECR_STATE_SENDING,
    ECR_STATE_RECEIVING,
    ECR_STATE_COMPLETED,
    ECR_STATE_ERROR
} comm_testing_state_t;

// 误码率测试结果结构体
typedef struct {
    uint32_t total_packets;      // 总发送/接收包数
    uint32_t correct_packets;    // 正确接收包数
    uint32_t error_packets;      // 错误包数
    uint32_t timeout_packets;    // 超时包数
    float error_rate;            // 误码率（百分比）
    uint32_t test_duration_ms;   // 测试持续时间（毫秒）
} comm_testing_result_t;

// 误码率测试控制结构体
typedef struct {
    comm_testing_state_t state;
    uint32_t packet_count;
    uint32_t start_time;
    uint32_t last_send_time;
    uint8_t send_buffer[ECR_PACKET_SIZE];
    uint8_t recv_buffer[ECR_PACKET_SIZE];
    comm_testing_result_t result;
    bool test_running;
    // 1分钟统计变量
    uint32_t last_minute_start_time;
    uint32_t send_count_last_minute;
    uint32_t receive_count_last_minute;
} comm_testing_control_t;

// 函数声明
void init_lora_testing(void);

// 任务函数
void create_lora_testing_task(void);
void comm_testing_master_task(void *argument);

// 内部函数
void comm_testing_generate_send_buffer(uint8_t *data, uint32_t packet_num);

#ifdef __cplusplus
}
#endif

#endif /* UART_ERROR_CODE_RATE_H */

#endif
