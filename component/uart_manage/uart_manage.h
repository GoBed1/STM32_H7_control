#ifndef _UART_MANAGE_H_
#define _UART_MANAGE_H_

#include "main.h"

typedef void (*uart_recv_callback_t)(uint8_t *data, uint16_t len);

typedef struct uart_manage_t {
    UART_HandleTypeDef *huart;
    uint8_t rx_buf[256];
    uint8_t tx_buf[256];
    uart_recv_callback_t recv_callback;
} uart_manage_t;

int uart_manage_init(uart_manage_t *uart, UART_HandleTypeDef *huart, uart_recv_callback_t recv_callback);
int uart_manage_dma_send(uart_manage_t *uart, const uint8_t *data, uint16_t len);
int uart_manage_start_dma_recv(uart_manage_t *uart);

#endif // _UART_MANAGE_H_
 
