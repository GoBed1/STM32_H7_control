
#ifdef ENABLE_ERROR_CODE_RATE_TEST

#include <test_communication.h>
#include "board.h"
#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// 全局控制结构体
static comm_testing_control_t g_comm_testing_control = {0};

// 使用全局标志位替代信号量，指示有新的接收数据到达
static volatile bool g_comm_testing_rx_ready_flag = false;

// 任务句柄
static osThreadId_t comm_testing_master_task_handle = NULL;
static osThreadId_t comm_testing_slave_task_handle = NULL;
static osThreadId_t comm_testing_rx_handler_task_handle = NULL;

// 不再使用信号量

// 任务属性
static const osThreadAttr_t comm_testing_master_task_attributes = {
    .name = "ECRMasterTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

static const osThreadAttr_t comm_testing_slave_task_attributes = {
    .name = "ECRSlaveTask", 
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

static const osThreadAttr_t comm_testing_rx_handler_task_attributes = {
    .name = "ECRRxHandlerTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityHigh,  // 高优先级处理接收数据
};

// ================= UART7 双缓冲接收（供中断与任务共享）=================
// 在此文件定义（唯一定义），在头文件中以 extern 声明
uint8_t uart7_rx_buffer[2][UART7_RX_BUFFER_SIZE];
volatile uint8_t uart7_rx_active_idx = 0;
volatile uint16_t uart7_rx_len = 0;
volatile uint8_t uart7_rx_idle_flag = 0;
uint8_t uart7_rx_ready_buffer[UART7_RX_BUFFER_SIZE];

/**
 * @brief 初始化误码率测试模块
 */
void init_lora_testing(void)
{
    memset(&g_comm_testing_control, 0, sizeof(comm_testing_control_t));
}

/**
 * @brief 计算CRC16校验
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC16校验值
 */
static uint16_t comm_testing_calculate_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief 生成测试数据
 * @param data 数据缓冲区
 * @param packet_num 包序号
 */
void comm_testing_generate_send_buffer(uint8_t *data, uint32_t packet_num)
{
    // 生成数据包：帧头1字节0xF5+包序号1字节+地址码1字节+指令码1字节+数据段长度1字节+数据段N字节+CRC16整包校验2字节
    data[0] = 0xF5;  // 帧头
    
    // 包序号（1字节，达到最大值255则置0）
    data[1] = (uint8_t)(packet_num % 256);
    
    // 地址码（1字节）
    data[2] = 0x01;  // 固定地址码
    
    // 指令码（1字节）
    data[3] = 0x02;  // 固定指令码
    
    // 数据段长度（1字节）
    uint8_t data_length = 16;  // 固定16字节数据段
    data[4] = data_length;
    
    // 数据段：0x00-0x0F (16字节数据)
    for (int i = 0; i < 16; i++) {
        data[5 + i] = (uint8_t)i;  // 0x00, 0x01, 0x02, ..., 0x0F
    }
    
    // 计算CRC16校验（对整个数据包进行校验）
    uint16_t crc = comm_testing_calculate_crc16(data, 21);  // 帧头1+序号1+地址1+指令1+长度1+数据16=21字节
    data[21] = (uint8_t)(crc & 0xFF);        // CRC低字节
    data[22] = (uint8_t)((crc >> 8) & 0xFF); // CRC高字节
}

/**
 * @brief 主控端任务（发送数据）
 */
void comm_testing_master_task(void *argument)
{

    printf("[INFO] [TESTING] Master task started\r\n");
    
    while (1) {
        uint32_t current_time = get_time_ms();
        
        // 检查发送间隔
        if (current_time - g_comm_testing_control.last_send_time >= ECR_SEND_INTERVAL_MS) {
            // 准备发送数据
            uint8_t *send_data = g_comm_testing_control.send_buffer;
            send_data[0] = 0xF5;  // 帧头
            send_data[1] = (uint8_t)(g_comm_testing_control.packet_count % 256);  // 包序号
            send_data[2] = 0xFF;  // 地址码
            send_data[3] = 0x02;  // 指令码
            send_data[4] = 16;    // 数据段长度
            
            // 数据段：0x00-0x0F (16字节数据)
            for (int i = 0; i < 16; i++) {
                send_data[5 + i] = (uint8_t)i;
            }
            
            // 计算CRC16校验
            uint16_t crc = comm_testing_calculate_crc16(send_data, 21);
            send_data[21] = (uint8_t)(crc & 0xFF);
            send_data[22] = (uint8_t)((crc >> 8) & 0xFF);

            // 发送数据
            HAL_StatusTypeDef status = HAL_UART_Transmit(&huart7, 
                send_data, ECR_PACKET_SIZE, ECR_TIMEOUT_MS);
            
            if (status == HAL_OK) {
                printf("[SEND] ");
                for (uint16_t i = 0; i < ECR_PACKET_SIZE; i++) {
                    printf("%02X ", send_data[i]);
                }
                printf("\r\n");
                g_comm_testing_control.send_count_last_minute++;
            } else {
                printf("[ERROR] [TESTING] Send failed: %d\r\n", status);
            }
            
            g_comm_testing_control.last_send_time = current_time;
            g_comm_testing_control.packet_count++;

        }
        osDelay(1);
    }
}

// ================= 任务与中断共享变量声明结束 =================

// 中断回调：UART7 空闲中断处理（在 UART7_IRQHandler 中调用）
void comm_testing_uart7_idle_irq_handler(void)
{
    // 清除空闲中断标志
    __HAL_UART_CLEAR_IDLEFLAG(&huart7);
    // 计算数据长度（剩余计数转为已接收长度）
    uart7_rx_len = UART7_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart7.hdmarx);
    if (uart7_rx_len > UART7_RX_BUFFER_SIZE) {
        uart7_rx_len = 0; // 防御性保护
    }
    // 将当前缓冲区数据拷贝到就绪缓冲区，交由任务处理
    if (uart7_rx_len > 0) {
        memcpy(uart7_rx_ready_buffer, uart7_rx_buffer[uart7_rx_active_idx], uart7_rx_len);
    }
    uart7_rx_idle_flag = 1;
    // 切换DMA目标缓冲区
    uart7_rx_active_idx ^= 1;
    // 停止并重启DMA接收到新缓冲区
    HAL_UART_DMAStop(&huart7);
    HAL_UART_Receive_DMA(&huart7, uart7_rx_buffer[uart7_rx_active_idx], UART7_RX_BUFFER_SIZE);
}

/**
 * @brief UART7 DMA接收数据处理任务
 */
void comm_testing_rx_handler_task(void *argument)
{
    // 启动UART7的DMA接收，接收缓冲区为uart7_rx_buffer，长度为UART7_RX_BUFFER_SIZE
    HAL_UART_Receive_DMA(&huart7, uart7_rx_buffer[uart7_rx_active_idx], UART7_RX_BUFFER_SIZE);
    // 使能UART7空闲中断
    __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);
    printf("[INFO] [TESTING] RX Handler task started\r\n");
    while (1) {
        if (uart7_rx_idle_flag) {
            uart7_rx_idle_flag = 0;
#ifdef CHECK_ECR_SLAVE
        printf("[RECV] ");
#elif defined(CHECK_ECR_MASTER)
        printf("[ ACK] ");
#endif
            for (uint16_t i = 0; i < uart7_rx_len; i++) {
                printf("%02X ", uart7_rx_ready_buffer[i]);
            }
            printf("\r\n");
#ifdef CHECK_ECR_SLAVE
            // 回显：将收到的数据通过 UART7 发送回去
            if (uart7_rx_len > 0) {
                HAL_StatusTypeDef txStatus = HAL_UART_Transmit(&huart7, uart7_rx_ready_buffer, uart7_rx_len, ECR_TIMEOUT_MS);
                if (txStatus != HAL_OK) {
                    printf("[ERROR] [TESTING] Echo TX failed: %d\r\n", txStatus);
                }
                else{
                    printf("[ECHO] ");
                    for (uint16_t i = 0; i < uart7_rx_len; i++) {
                        printf("%02X ", uart7_rx_ready_buffer[i]);
                    }
                    printf("\r\n");
                }
            }
#endif
        }
        osDelay(1);
    }
}

/**
 * @brief 创建误码率测试任务
 */
void create_lora_testing_task(void)
{

#ifdef CHECK_ECR_MASTER
    comm_testing_master_task_handle = osThreadNew(comm_testing_master_task, NULL, &comm_testing_master_task_attributes);
    if (comm_testing_master_task_handle != NULL) {
        printf("[INFO] [TESTING] Master task created\r\n");
    } else {
        printf("[ERROR] [TESTING] Failed to create master task\r\n");
    }
#endif

    // 始终创建 RX Handler 任务
    comm_testing_rx_handler_task_handle = osThreadNew(comm_testing_rx_handler_task, NULL, &comm_testing_rx_handler_task_attributes);
    if (comm_testing_rx_handler_task_handle != NULL) {
        printf("[INFO] [TESTING] RX Handler task created\r\n");
    } else {
        printf("[ERROR] [TESTING] Failed to create RX Handler task\r\n");
    }
}

#endif
