/*
 * async_logger.h - lightweight UART DMA async logging (ring buffer + TX task)
 */
#ifndef BSP_ASYNC_LOGGER_H_
#define BSP_ASYNC_LOGGER_H_

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ASYNC_LOGGER_RING_SIZE
#define ASYNC_LOGGER_RING_SIZE 2048u
#endif

// Initialize with a UART handle to use for DMA TX.
// Creates internal objects; returns 0 on success.
int async_logger_init(UART_HandleTypeDef *huart);

// Start the logging task (priority below normal by default). Safe to call once.
int async_logger_start(void);

// Non-blocking enqueue; drops if no space when drop_if_full=1, else blocks up to timeout_ms.
// Returns number of bytes accepted.
size_t async_logger_write(const uint8_t *data, size_t len, uint32_t timeout_ms, int drop_if_full);

// ISR hook to be called from HAL_UART_TxCpltCallback.
void async_logger_on_tx_cplt_from_isr(UART_HandleTypeDef *huart);

// Optional helpers
void async_logger_flush(uint32_t timeout_ms);
int  async_logger_is_running(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ASYNC_LOGGER_H_ */
