#include "modbus_rtu_client_protocol.h"
#include "main.h"
#include <string.h>

#define MB_RTU_PROTO_E(...) LOG_ERR("MODBUS", __VA_ARGS__)

static void modbus_rtu_client_timeout_event(TimerHandle_t timer);

uint16_t modbus_rtu_calculate_crc(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void modbus_rtu_client_recv_event(void *self, uint8_t *data, uint16_t len)
{
    modbus_rtu_protocol_t *p = (modbus_rtu_protocol_t *)self;

    if (len > sizeof(p->recv_packet)) {
        len = sizeof(p->recv_packet);
    }
    memcpy(p->recv_packet, data, len);
    p->recv_packet_len = len;

    if (p->thread_id) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(p->thread_id, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void modbus_rtu_client_timeout_event(TimerHandle_t timer)
{
    modbus_rtu_protocol_t *p = (modbus_rtu_protocol_t *)pvTimerGetTimerID(timer);
    p->error_code = MODBUS_ERR_TIMEOUT;
    if (p->thread_id) {
        xTaskNotifyGive(p->thread_id);
    }
}

void modbus_rtu_client_config_param(modbus_rtu_protocol_t *self,uint8_t server_id)
{
    self->server_id = server_id;
    self->recv_packet_len = 0;
    self->error_code = MODBUS_OK;
}

void modbus_rtu_client_set_sender(modbus_rtu_protocol_t *self,
                                 modbus_send_callback_t sender)
{
    self->sender = sender;
}

void modbus_rtu_client_create_timeout_timer(modbus_rtu_protocol_t *self,
                                            uint32_t timeout_ms)
{
    self->timeout_timer = xTimerCreate("mb_timeout",
                                       pdMS_TO_TICKS(timeout_ms),
                                       pdFALSE,
                                       (void *)self,
                                       modbus_rtu_client_timeout_event);
    if (!self->timeout_timer) {
        MB_RTU_PROTO_E("Failed to create timeout timer");
    }
}   
void modbus_rtu_client_set_thread_id(modbus_rtu_protocol_t *self, osThreadId_t thread_id)
{
    self->thread_id = thread_id;
}

int modbus_rtu_read_sync(modbus_rtu_protocol_t *self, uint8_t func_code,
                   uint16_t start_addr, uint16_t quantity)
{
    uint8_t req_buf[8];
    int req_len = -1;
        
    if (func_code == 0x03) {
        req_buf[0] = self->server_id;
        req_buf[1] = 0x03;
        req_buf[2] = (start_addr >> 8) & 0xFF;
        req_buf[3] = start_addr & 0xFF;
        req_buf[4] = (quantity >> 8) & 0xFF;
        req_buf[5] = quantity & 0xFF;
        uint16_t crc = modbus_rtu_calculate_crc(req_buf, 6);
        req_buf[6] = crc & 0xFF;
        req_buf[7] = (crc >> 8) & 0xFF;
        req_len = 8;
    } else {
        MB_RTU_PROTO_E("Unsupported function code: 0x%02X", func_code);
        return -1;
    }
    
    if (req_len < 0) {
        MB_RTU_PROTO_E("Build request failed");
        return -1;
    }
    
    self->error_code = MODBUS_OK;
    self->recv_packet_len = 0;
    self->thread_id = xTaskGetCurrentTaskHandle();

    if (self->sender(req_buf, req_len) != 0) {
        self->error_code = MODBUS_ERR_SEND_FAIL;
        return self->error_code;
    }

    xTimerStart(self->timeout_timer, 0);
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return self->error_code;
}

