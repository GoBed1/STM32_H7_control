release v1.0.0
# Overview
Realizes asynchronous reading functionality of the Modbus RTU client

# Dependency Lib
https://github.com/riveriver/uart_manage.git

# Example
```
// in app/task.c
#include "read_sensor_task.h"
#include <stdio.h>
#include "uart_manage.h"
#include "modbus_rtu_client_protocol.h"
#include "main.h"

#define ENABLE_LOG_DEBUG
#ifdef ENABLE_LOG_DEBUG
#define LOG_D(...) LOG_DEBUG("SENSOR", __VA_ARGS__)
#else
#define LOG_D(...) // LOG_DEBUG("SENSOR", __VA_ARGS__)
#endif
#define LOG_E(...) LOG_ERR("SENSOR", __VA_ARGS__)
#define LOG_I(...) LOG_INFO("SENSOR", __VA_ARGS__)


#define GRAVITY_ACCEL 9.8f

extern UART_HandleTypeDef huart2;
#define MODBUS_RTU_UART huart2

typedef struct {
    float x;
    float y;
    float z;
} accel_data_t;

typedef struct {
    modbus_rtu_protocol_t modbus;
    osThreadId_t thread_id;
} sensor_app_t;

static void sensor_data_parse(uint16_t *regs, accel_data_t *accel)
{
    accel->x = ((int16_t)regs[0] / 32768.0f) * 4.0f * GRAVITY_ACCEL;
    accel->y = ((int16_t)regs[1] / 32768.0f) * 4.0f * GRAVITY_ACCEL;
    accel->z = ((int16_t)regs[2] / 32768.0f) * 4.0f * GRAVITY_ACCEL;
}

static int sensor_parse_modbus_response(modbus_rtu_protocol_t *mb, uint16_t *regs)
{
    // 检查最小长度：ID(1) + FC(1) + ByteCount(1) + Data(6) + CRC(2) = 11
    if (mb->recv_packet_len < 11) {
        LOG_E("Frame too short: %d", mb->recv_packet_len);
        return MODBUS_ERR_FRAME_LEN;
    }

    // 检查从站地址
    if (mb->recv_packet[0] != mb->server_id) {
        LOG_E("Slave ID mismatch: expected 0x%02X, got 0x%02X", mb->server_id, mb->recv_packet[0]);
        return MODBUS_ERR_PARSE_FAIL;
    }

    // 检查功能码（应该是 0x03）
    if (mb->recv_packet[1] != 0x03) {
        LOG_E("Function code mismatch: expected 0x03, got 0x%02X", mb->recv_packet[1]);
        return MODBUS_ERR_PARSE_FAIL;
    }

    // 检查字节数
    uint8_t byte_count = mb->recv_packet[2];
    if (byte_count + 5 != mb->recv_packet_len) {
        LOG_E("Frame length mismatch: byte_count=%d, total_len=%d", byte_count, mb->recv_packet_len);
        return MODBUS_ERR_FRAME_LEN;
    }

    // 校验 CRC（Modbus RTU 的 CRC 是小端序）
    uint16_t calc_crc = modbus_rtu_calculate_crc(mb->recv_packet, mb->recv_packet_len - 2);
    uint16_t recv_crc = mb->recv_packet[mb->recv_packet_len - 2] | 
                        ((uint16_t)mb->recv_packet[mb->recv_packet_len - 1] << 8);
    if (calc_crc != recv_crc) {
        LOG_E("CRC error: expected 0x%04X, got 0x%04X", calc_crc, recv_crc);
        return MODBUS_ERR_CRC;
    }

    // 解析 3 个寄存器数据（Modbus 大端序），通过指针输出
    for (int i = 0; i < 3; i++) {
        regs[i] = ((uint16_t)mb->recv_packet[3 + i * 2] << 8) | mb->recv_packet[4 + i * 2];
    }
    // for (int i = 0; i < 3; i++) {
    //     regs[i] = (uint16_t)mb->recv_packet[3 + i * 2];
    // }

    return MODBUS_OK;
}

static const osThreadAttr_t read_sensor_thread_attr = {
    .name = "read_sensor_thread",
    .stack_size = 2048 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

uart_manage_t sensor_uart;
sensor_app_t sensor_app;
static void read_sensor_thread(void *args);

int sensor_app_lower_send( const uint8_t *data, uint16_t len){
    return uart_manage_dma_send(&sensor_uart, data, len);
}

static void sensor_app_recv_event(uint8_t *data, uint16_t len)
{
    modbus_rtu_client_recv_event(&sensor_app.modbus, data, len);
}

void sensor_app_init()
{
    osThreadId_t tid = osThreadNew(read_sensor_thread, NULL, &read_sensor_thread_attr);
    if (tid == NULL) {
        LOG_E("Failed to create read sensor task");
    } else {
        sensor_app.thread_id = tid;
        LOG_I("Read sensor task created");
    }
}

static void read_sensor_thread(void *args)
{
    (void)args;
    uart_manage_init(&sensor_uart, &MODBUS_RTU_UART, sensor_app_recv_event);
    
	modbus_rtu_client_config_param(&sensor_app.modbus,0x50);
    modbus_rtu_client_set_sender(&sensor_app.modbus, sensor_app_lower_send);
    modbus_rtu_client_create_timeout_timer(&sensor_app.modbus, 1000);
    modbus_rtu_client_set_thread_id(&sensor_app.modbus, sensor_app.thread_id);

    uart_manage_start_dma_recv(&sensor_uart);

    LOG_I("Read sensor task started");
    
    TickType_t wakeup_time = xTaskGetTickCount();
    TickType_t stats_time = wakeup_time;
    
    char hex_buf[128] = {0};
    uint32_t read_freq = 0;
    
    for (;;) {
        int err = modbus_rtu_read_sync(&sensor_app.modbus, 0x03, 0x32, 3);
        if (err != MODBUS_OK) {
            LOG_E("[%d]Failed to read sensor", err);
        } else {
            xTimerStop(sensor_app.modbus.timeout_timer, 0);
            
        // int idx = 0;
        // for (int i = 0; i < sensor_app.modbus.recv_packet_len && idx < sizeof(hex_buf) - 3; i++) {
        //     idx += sprintf(hex_buf + idx, "%02X ", sensor_app.modbus.recv_packet[i]);
        // }
        // LOG_D("r[%d]:%s", sensor_app.modbus.recv_packet_len, hex_buf);
        read_freq++;
        TickType_t now = xTaskGetTickCount();
        if ((now - stats_time) >= pdMS_TO_TICKS(5000)) {
            LOG_D("c:[%lu,%lu]", now,read_freq);
            read_freq = 0;
            stats_time = now;
        }


            uint16_t regs[3] = {0};
            err = sensor_parse_modbus_response(&sensor_app.modbus, regs);
            if (err == MODBUS_OK) {
                // LOG_I("Raw regs: [0]=0x%04X (%d), [1]=0x%04X (%d), [2]=0x%04X (%d)", 
                //       regs[0], (int16_t)regs[0], regs[1], (int16_t)regs[1], regs[2], (int16_t)regs[2]);
            } else {
                LOG_E("Parse frame failed: %d", err);
            }
        }
        
        vTaskDelayUntil(&wakeup_time, pdMS_TO_TICKS(5));
    }
}

// in uart_app.c
#include "uart_manage.h"

extern uart_manage_t sensor_uart;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    if(huart == sensor_uart.huart) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)sensor_uart.rx_buf, sizeof(sensor_uart.rx_buf));
        if(sensor_uart.recv_callback) {
            sensor_uart.recv_callback(sensor_uart.rx_buf, size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(sensor_uart.huart, sensor_uart.rx_buf, sizeof(sensor_uart.rx_buf));
    }
}

```