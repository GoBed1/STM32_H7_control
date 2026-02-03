// https://github.com/cnoviello/mastering-stm32/blob/master/nucleo-f030R8/system/src/retarget/retarget.c

#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/times.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include "cmsis_os2.h"
#include "async_logger.h"

#include <printf_redirect.h>

#if !defined(OS_USE_SEMIHOSTING)

#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

UART_HandleTypeDef *gHuart;

// Optional mutex for printf to avoid interleaved output across tasks.
// Disabled by default to keep footprint minimal.
static osMutexId_t s_printfMutex = NULL;
static uint8_t s_printfMutexEnabled = 0; // 0 = off (default), 1 = on
static uint8_t s_useAsyncLogger = 0; // optional routing to DMA logger

// Internal helpers (non-API) to configure mutex; small wrappers exposed via header.
static void prv_printf_lock(void) {
  if (!s_printfMutexEnabled) return;
  // Only lock if called from thread context; skip if IRQ to avoid deadlocks
  if (__get_IPSR() != 0U) return; // in ISR
  if (s_printfMutex) {
    (void)osMutexAcquire(s_printfMutex, osWaitForever);
  }
}

static void prv_printf_unlock(void) {
  if (!s_printfMutexEnabled) return;
  if (__get_IPSR() != 0U) return; // in ISR
  if (s_printfMutex) {
    (void)osMutexRelease(s_printfMutex);
  }
}

// Public lightweight controls (do not break existing API)
void printf_use_mutex(int enable) {
  s_printfMutexEnabled = (enable != 0);
}

void printf_set_mutex(osMutexId_t mutex) {
  s_printfMutex = mutex;
}

osMutexId_t printf_create_mutex(void) {
  const osMutexAttr_t attr = {
    .name = "printf_mtx",
    .attr_bits = osMutexPrioInherit,
    .cb_mem = NULL,
    .cb_size = 0
  };
  s_printfMutex = osMutexNew(&attr);
  return s_printfMutex;
}

void printf_redirect_use_async_logger(int enable) {
  s_useAsyncLogger = (enable != 0);
}

void specify_redirect_uart(UART_HandleTypeDef *huart) {
  gHuart = huart;

  /* Disable I/O buffering for STDOUT stream, so that
   * chars are sent out as soon as they are printed. */
  setvbuf(stdout, NULL, _IONBF, 0);
}

int _isatty(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 1;

  errno = EBADF;
  return 0;
}

int _write(int fd, char* ptr, int len) {
  if (!(fd == STDOUT_FILENO || fd == STDERR_FILENO)) {
    errno = EBADF;
    return -1;
  }
  if (gHuart == NULL || ptr == NULL || len < 0) {
    errno = EINVAL;
    return -1;
  }

  // Option A: use async DMA logger if enabled and initialized
  if (s_useAsyncLogger && async_logger_is_running()) {
    size_t w = async_logger_write((const uint8_t*)ptr, (size_t)len, 0, /*drop_if_full*/1);
    // If dropped partially, still report bytes accepted (like non-blocking write)
    return (int)w;
  }

  // Option B: blocking HAL transmit with optional mutex
  prv_printf_lock();
  HAL_StatusTypeDef hstatus = HAL_UART_Transmit(gHuart, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
  prv_printf_unlock();

  if (hstatus == HAL_OK) return len;
  errno = EIO;
  return -1;
}

// int _write(int fd, char *ptr, int len) {
//     static uint8_t rc = USBD_OK;

//     do {
//         rc = CDC_Transmit_FS((uint8_t *)ptr, len);
//     } while (USBD_BUSY == rc);

//     if (USBD_FAIL == rc) {
//         /// NOTE: Should never reach here.
//         /// TODO: Handle this error.
//         return 0;
//     }
//     return len;
// }

// ITM version (commented out - use UART instead)
// int _write(int file, char *ptr, int len) {
//     int DataIdx;
//     for (DataIdx = 0; DataIdx < len; DataIdx++) {
//         ITM_SendChar(*ptr++);
//     }
//     return len;
// }

int _close(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 0;

  errno = EBADF;
  return -1;
}

int _lseek(int fd, int ptr, int dir) {
  (void) fd;
  (void) ptr;
  (void) dir;

  errno = EBADF;
  return -1;
}

int _read(int fd, char* ptr, int len) {
  if (fd != STDIN_FILENO) {
    errno = EBADF;
    return -1;
  }
  if (gHuart == NULL || ptr == NULL || len <= 0) {
    errno = EINVAL;
    return -1;
  }
  uint32_t nread = 0;
  // Simple blocking read of up to len bytes; preserve existing behavior of at least one
  while (nread < (uint32_t)len) {
    HAL_StatusTypeDef hstatus = HAL_UART_Receive(gHuart, (uint8_t *)ptr + nread, 1, HAL_MAX_DELAY);
    if (hstatus == HAL_OK) {
      nread++;
      // return on first byte for compatibility with prior implementation
      break;
    } else {
      errno = EIO;
      return -1;
    }
  }
  return (int)nread;
}

int _fstat(int fd, struct stat* st) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
    if (st) st->st_mode = S_IFCHR;
    return 0;
  }
  errno = EBADF;
  return -1;
}

#endif //#if !defined(OS_USE_SEMIHOSTING)
