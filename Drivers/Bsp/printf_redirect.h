/*
 * printf_redirect.h
 *
 *  Created on: Sep 9, 2025
 *      Author: river
 */

#ifndef BSP_PRINTF_REDIRECT_H_
#define BSP_PRINTF_REDIRECT_H_

#include "stm32h7xx_hal.h"
#include <sys/stat.h>
#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

void specify_redirect_uart(UART_HandleTypeDef *huart);
// Optional: enable a mutex around _write to avoid interleaved logs
osMutexId_t printf_create_mutex(void);
void printf_set_mutex(osMutexId_t mutex);
void printf_use_mutex(int enable);
// Optional: route printf to async DMA logger once it is started
void printf_redirect_use_async_logger(int enable);
int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);

#ifdef __cplusplus
}
#endif

#endif /* BSP_PRINTF_REDIRECT_H_ */
