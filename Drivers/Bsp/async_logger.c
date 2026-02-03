/*
 * async_logger.c - UART DMA async logger
 */
#include "async_logger.h"
#include <string.h>

// Simple ring buffer
static uint8_t s_ring[ASYNC_LOGGER_RING_SIZE];
static volatile uint32_t s_head = 0; // write index
static volatile uint32_t s_tail = 0; // read index
static volatile uint32_t s_inflight = 0; // bytes currently under DMA

static UART_HandleTypeDef *s_uart = NULL;
static osMutexId_t s_mtx = NULL;
static osSemaphoreId_t s_sem_tx_done = NULL; // binary, signaled on DMA complete
static osThreadId_t s_task = NULL;
static volatile uint8_t s_running = 0;

// internal utilities
static inline uint32_t ring_count(void) {
  uint32_t h = s_head, t = s_tail;
  return (h >= t) ? (h - t) : (ASYNC_LOGGER_RING_SIZE - (t - h));
}
static inline uint32_t ring_space(void) {
  return ASYNC_LOGGER_RING_SIZE - 1u - ring_count();
}
static inline void ring_write(const uint8_t *src, uint32_t len) {
  uint32_t h = s_head;
  uint32_t first = ASYNC_LOGGER_RING_SIZE - h;
  if (first > len) first = len;
  memcpy(&s_ring[h], src, first);
  if (len > first) {
    memcpy(&s_ring[0], src + first, len - first);
  }
  s_head = (h + len) % ASYNC_LOGGER_RING_SIZE;
}
static inline uint32_t ring_peek(uint8_t *dst, uint32_t maxlen) {
  uint32_t cnt = ring_count();
  if (cnt == 0) return 0;
  if (cnt > maxlen) cnt = maxlen;
  uint32_t t = s_tail;
  uint32_t first = ASYNC_LOGGER_RING_SIZE - t;
  if (first > cnt) first = cnt;
  memcpy(dst, &s_ring[t], first);
  if (cnt > first) memcpy(dst + first, &s_ring[0], cnt - first);
  return cnt;
}
static inline void ring_drop(uint32_t len) {
  s_tail = (s_tail + len) % ASYNC_LOGGER_RING_SIZE;
}

static void logger_task(void *arg) {
  (void)arg;
  s_running = 1;
  const uint32_t chunk = 512; // max DMA fragment per submit
  static uint8_t local[512];
  for (;;) {
    // Wait until there is data
    if (ring_count() == 0 && s_inflight == 0) {
      osDelay(2);
      continue;
    }

    // If a DMA transfer is in-flight, wait for completion
    if (s_inflight != 0) {
      uint32_t sent = s_inflight;
      osSemaphoreAcquire(s_sem_tx_done, osWaitForever);
      // consumed previously submitted bytes
      osMutexAcquire(s_mtx, osWaitForever);
      ring_drop(sent);
      osMutexRelease(s_mtx);
      s_inflight = 0;
      continue;
    }

    // Snapshot data
    uint32_t to_send;
    // Critical section to avoid race with writers
    osMutexAcquire(s_mtx, osWaitForever);
    to_send = ring_peek(local, chunk);
    osMutexRelease(s_mtx);

    if (to_send == 0) {
      continue;
    }

    // Start DMA transmit
    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(s_uart, local, to_send);
    if (st == HAL_OK) {
      s_inflight = to_send;
    } else {
      // On failure, drop a small portion to avoid spin
      osMutexAcquire(s_mtx, osWaitForever);
      ring_drop(to_send > 16 ? 16 : to_send);
      osMutexRelease(s_mtx);
      osDelay(1);
    }
  }
}

int async_logger_init(UART_HandleTypeDef *huart) {
  if (!huart) return -1;
  s_uart = huart;
  s_head = s_tail = s_inflight = 0;
  const osMutexAttr_t mtx_attr = { .name = "alog_mtx", .attr_bits = osMutexPrioInherit };
  s_mtx = osMutexNew(&mtx_attr);
  const osSemaphoreAttr_t sem_attr = { .name = "alog_tx" };
  s_sem_tx_done = osSemaphoreNew(1, 0, &sem_attr);
  return (s_mtx && s_sem_tx_done) ? 0 : -1;
}

int async_logger_start(void) {
  if (s_task) return 0;
  const osThreadAttr_t attr = { .name = "alog", .stack_size = 1024, .priority = osPriorityBelowNormal };
  s_task = osThreadNew(logger_task, NULL, &attr);
  return s_task ? 0 : -1;
}

size_t async_logger_write(const uint8_t *data, size_t len, uint32_t timeout_ms, int drop_if_full) {
  if (!data || len == 0) return 0;
  size_t written = 0;
  uint32_t deadline = (timeout_ms == osWaitForever) ? 0 : (osKernelGetTickCount() + timeout_ms);
  for (;;) {
    osMutexAcquire(s_mtx, osWaitForever);
    uint32_t space = ring_space();
    if (space == 0) {
      osMutexRelease(s_mtx);
      if (drop_if_full) break;
      if (timeout_ms == 0) break;
      if (timeout_ms == osWaitForever || (int32_t)(deadline - osKernelGetTickCount()) > 0) {
        osDelay(1);
        continue;
      }
      break; // timeout
    }
    uint32_t chunk = (uint32_t)(len - written);
    if (chunk > space) chunk = space;
    ring_write(data + written, chunk);
    osMutexRelease(s_mtx);
    written += chunk;
    if (written >= len) break;
  }
  return written;
}

void async_logger_flush(uint32_t timeout_ms) {
  uint32_t start = osKernelGetTickCount();
  while (ring_count() > 0 || s_inflight != 0) {
    if (timeout_ms != osWaitForever && (osKernelGetTickCount() - start) > timeout_ms) break;
    osDelay(1);
  }
}

int async_logger_is_running(void) {
  return s_task != NULL;
}
