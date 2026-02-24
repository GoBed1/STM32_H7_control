// USART1 application setup
#ifndef UART_MANAGE_H
#define UART_MANAGE_H

#include "lwrb.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#include "usart.h"

#include "stm32h7xx_hal.h"
#include "fifo.h"

/* DMA buffer placement */
#if defined(__GNUC__)
#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define DMA_BUFFER
#endif

/* Cache line size on Cortex-M7 */
#define DCACHE_LINE_SIZE 32U

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

#define USART1_RECV_BUFFER_SIZE 256U
#define USART1_SEND_BUFFER_SIZE 256U
#define USART1_SEND_FIFO_SIZE 256U

extern uint8_t usart1_recv_buff[USART1_RECV_BUFFER_SIZE];
extern uint8_t usart1_send_buff[USART1_SEND_BUFFER_SIZE];
extern uint8_t usart1_send_fifo_buff[USART1_SEND_FIFO_SIZE];

typedef uint32_t (*usart_call_back)(uint8_t *buf, uint16_t len);
typedef struct
{
    UART_HandleTypeDef *uart_h;
    DMA_HandleTypeDef *dma_h;
    uint16_t recv_buffer_size;
    uint8_t *recv_buffer;
    usart_call_back recv_callback;

    uint8_t *send_buffer;
    uint16_t send_buffer_size;
    fifo_s_t send_fifo;
    uint8_t *send_fifo_buffer;
    uint16_t send_fifo_size;
    uint8_t is_sending;
} uart_manage_t;

typedef enum
{
    INTERRUPT_TYPE_UART = 0,
    INTERRUPT_TYPE_DMA_HALF = 1,
    INTERRUPT_TYPE_DMA_ALL = 2
} interrput_type;


extern uart_manage_t echo_uart_manage;

void echo_uart_init(void);
void echo_process_loop(uart_manage_t *m_obj);

#ifdef __cplusplus
}
#endif

#endif // UART_MANAGE_H
