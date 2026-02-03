#include "uart_manage.h"
#include "stm32h7xx_hal.h"
#include "syslog.h"

#define UARTM_I(...) LOG_INFO("UARTM", __VA_ARGS__)

int uart_manage_init(uart_manage_t *uart, UART_HandleTypeDef *huart, uart_recv_callback_t recv_callback)
{
    uart->huart = huart;
    uart->recv_callback = recv_callback;
    return 0;
}

int uart_manage_dma_send(uart_manage_t *uart, const uint8_t *data, uint16_t len)
{
    if (len > sizeof(uart->tx_buf)) {
        return -1;
    }

    memcpy(uart->tx_buf, data, len);
    
    uint32_t addr_aligned = (uint32_t)uart->tx_buf & ~31U;
    uint32_t len_aligned = ((uint32_t)uart->tx_buf + len - addr_aligned + 31U) & ~31U;
    SCB_CleanDCache_by_Addr((uint32_t *)addr_aligned, len_aligned);
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(uart->huart, uart->tx_buf, len);
    return (status == HAL_OK) ? 0 : -1;
}

int uart_manage_start_dma_recv(uart_manage_t *uart)
{
    uint32_t addr_aligned = (uint32_t)uart->rx_buf & ~31U;
    uint32_t len_aligned = ((uint32_t)uart->rx_buf + sizeof(uart->rx_buf) - addr_aligned + 31U) & ~31U;
    SCB_InvalidateDCache_by_Addr((uint32_t *)addr_aligned, len_aligned);
    HAL_UARTEx_ReceiveToIdle_DMA(uart->huart, uart->rx_buf, sizeof(uart->rx_buf));
}


