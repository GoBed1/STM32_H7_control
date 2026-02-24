#include "uart_manage.h"
#include "board.h"
#include "main.h"
#include "stm32h7xx_hal_cortex.h"
#include "stm32h7xx_hal_uart.h"
#include "lwrb.h"
#include <stdint.h>
#include <string.h>
#include "Modbus.h"
extern RFIDClient RFID_client;
extern EventGroupHandle_t eg; // 初始化事件组为NULL

int32_t uart_manage_dma_send(uart_manage_t *m_obj, uint8_t *buf, uint16_t len);
static void uart_send_completed_hook(uart_manage_t *m_obj);
void uart_manage_enable_dma_recv(uart_manage_t *m_obj);

uint8_t usart1_recv_buff[USART1_RECV_BUFFER_SIZE] DMA_BUFFER;
uint8_t usart1_send_buff[USART1_SEND_BUFFER_SIZE] DMA_BUFFER;
uint8_t usart1_send_fifo_buff[USART1_SEND_FIFO_SIZE] DMA_BUFFER;
uart_manage_t echo_uart_manage;
static uint8_t echo_recv_ring_buff[USART1_RECV_BUFFER_SIZE * 4U] DMA_BUFFER;
lwrb_t echo_lwrb;
uint8_t echo_recv_pending;

uint32_t echo_recv_callback(uint8_t *buf, uint16_t len)
{
  lwrb_sz_t free_len = lwrb_get_free(&echo_lwrb);
  if ((lwrb_sz_t)len > free_len)
  {
    return -1;
  }

  /* 将 DMA 接收缓冲区中的数据拷贝到环形缓冲区 */
  if (lwrb_write(&echo_lwrb, buf, (lwrb_sz_t)len) != (lwrb_sz_t)len)
  {
    return -1;
  }
  echo_recv_pending = 1U;
  return 0;
}

void echo_process_loop(uart_manage_t *m_obj)
{
  if (echo_recv_pending != 0U)
  {
    echo_recv_pending = 0U;
    /* Get length of linear memory at read pointer */
    /* When function returns 0, there is no memory
      available in the buffer for read anymore */
    lwrb_sz_t len;
    uint8_t *data;
    while ((len = lwrb_get_linear_block_read_length(&echo_lwrb)) > 0) {
        /* Get pointer to first element in linear block at read address */
        data = lwrb_get_linear_block_read_address(&echo_lwrb);

        /* If max length needs to be considered */
        /* simply decrease it and use smaller len on skip function */
        if (len > m_obj->send_buffer_size) {
            len = m_obj->send_buffer_size;
        }
        /* Send data via DMA and wait to finish (for sake of example) */
        uart_manage_dma_send(m_obj, data, len);

        /* Now skip sent bytes from buffer = move read pointer */
        lwrb_skip(&echo_lwrb, len);
    }
  }
}

void echo_uart_init(void)
{
    echo_uart_manage.recv_buffer = usart1_recv_buff;
    echo_uart_manage.recv_buffer_size = USART1_RECV_BUFFER_SIZE;
    echo_uart_manage.dma_h = &hdma_usart1_rx;
    echo_uart_manage.uart_h = &huart1;
    echo_uart_manage.send_fifo_buffer = usart1_send_fifo_buff;
    echo_uart_manage.send_fifo_size = USART1_SEND_FIFO_SIZE;
    echo_uart_manage.send_buffer_size = USART1_SEND_BUFFER_SIZE;
    echo_uart_manage.send_buffer = usart1_send_buff;
    echo_uart_manage.is_sending = 0;
    echo_uart_manage.recv_callback = echo_recv_callback;

    fifo_s_init(&(echo_uart_manage.send_fifo), usart1_send_fifo_buff, USART1_SEND_FIFO_SIZE);
    (void)lwrb_init(&echo_lwrb, echo_recv_ring_buff, sizeof(echo_recv_ring_buff));
    echo_recv_pending = 0U;

    (void)uart_manage_enable_dma_recv(&echo_uart_manage);
}

/* 地址按 32 字节 Cache line 向下对齐 */
static inline uintptr_t dma_align_down_32(uintptr_t addr)
{
  return addr & ~(uintptr_t)(DCACHE_LINE_SIZE - 1U);
}

/* 地址按 32 字节 Cache line 向上对齐 */
static inline uintptr_t dma_align_up_32(uintptr_t addr)
{
  return (addr + (DCACHE_LINE_SIZE - 1U)) & ~(uintptr_t)(DCACHE_LINE_SIZE - 1U);
}

/*
 * DMA 发送前清理 D-Cache，确保 DMA 能读到 CPU 最新写入的数据
 */
static inline void dma_clean_cache_by_addr(const void *addr, uint32_t len)
{
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  uintptr_t start = dma_align_down_32((uintptr_t)addr);
  uintptr_t end   = dma_align_up_32((uintptr_t)addr + len);
  SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
  (void)addr;
  (void)len;
#endif
}

/*
 * DMA 接收后失效 D-Cache，确保 CPU 读取的是 DMA 刚写入的新数据
 */
static inline void dma_invalidate_cache_by_addr(const void *addr, uint32_t len)
{
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  uintptr_t start = dma_align_down_32((uintptr_t)addr);
  uintptr_t end   = dma_align_up_32((uintptr_t)addr + len);
  SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
  (void)addr;
  (void)len;
#endif
}

int32_t uart_manage_dma_send(uart_manage_t *m_obj, uint8_t *buf, uint16_t len)
{
  uint16_t to_send_len;
  uint16_t to_tx_fifo_len;

  if (m_obj->is_sending == 0)
  {
      if (len < m_obj->send_buffer_size)
      {
          to_send_len = len;
          to_tx_fifo_len = 0;
      }
      else if (len < m_obj->send_buffer_size + m_obj->send_fifo_size)
      {
          to_send_len = m_obj->send_buffer_size;
          to_tx_fifo_len = len - m_obj->send_buffer_size;
      }
      else
      {
          to_send_len = m_obj->send_buffer_size;
          to_tx_fifo_len = m_obj->send_fifo_size;
      }
  }
  else
  {
      if (len < m_obj->send_fifo_size)
      {
          to_send_len = 0;
          to_tx_fifo_len = len;
      }
      else
      {
          to_send_len = 0;
          to_tx_fifo_len = m_obj->send_fifo_size;
      }
  }

  if (to_send_len > 0)
  {
      memcpy(m_obj->send_buffer, buf, to_send_len);
      dma_clean_cache_by_addr(m_obj->send_buffer, to_send_len);
      m_obj->is_sending = 1;
      HAL_UART_Transmit_DMA(m_obj->uart_h, m_obj->send_buffer, to_send_len);
  }
  if (to_tx_fifo_len > 0)
  {
      uint8_t len;
      len = fifo_s_puts(&(m_obj->send_fifo), (char *)(buf) + to_send_len, to_tx_fifo_len);

      if (len != to_tx_fifo_len)
      {
          return -1;
      }
  }
  return 0;
}

static void uart_send_completed_hook(uart_manage_t *m_obj)
{
    uint16_t fifo_data_num = 0;
    uint16_t send_num = 0;

    fifo_data_num = m_obj->send_fifo.used_num;

    if (fifo_data_num != 0)
    {
        if (fifo_data_num < m_obj->send_buffer_size)
        {
            send_num = fifo_data_num;
        }
        else
        {
            send_num = m_obj->send_buffer_size;
        }
        fifo_s_gets(&(m_obj->send_fifo), (char *)(m_obj->send_buffer), send_num);
        dma_clean_cache_by_addr(m_obj->send_buffer, send_num);
        m_obj->is_sending = 1;
        HAL_UART_Transmit_DMA(m_obj->uart_h, m_obj->send_buffer, send_num);
    }
    else
    {
        m_obj->is_sending = 0;
    }
    return;
}

static int32_t uart_manage_recv_idle_hook(uart_manage_t *m_obj, interrput_type int_type, uint16_t size)
{
  (void)int_type;

  if ((m_obj == NULL) || (size == 0U) || (size > m_obj->recv_buffer_size))
  {
    return -1;
  }

  dma_invalidate_cache_by_addr(m_obj->recv_buffer, size);

  if(m_obj->recv_callback != NULL)
  {
    m_obj->recv_callback(m_obj->recv_buffer, size);
  }

  return 0;
}

void uart_manage_enable_dma_recv(uart_manage_t *m_obj)
{
  if (m_obj == NULL)
  {
    return;
  }

  if (HAL_UARTEx_ReceiveToIdle_DMA(m_obj->uart_h, m_obj->recv_buffer, m_obj->recv_buffer_size) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  
  if (huart->Instance == USART1)
  {
    if (size > 0U)
    {
      (void)uart_manage_recv_idle_hook(&echo_uart_manage, INTERRUPT_TYPE_UART, size);
    }
    (void)uart_manage_enable_dma_recv(&echo_uart_manage);
  }

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (huart->Instance == huart2.Instance)
	{
		// 		static int count = 0;
		// count222++;
		if (RFID_client.rx_buf[0] == 0x1B && RFID_client.rx_buf[1] == 0x39 && RFID_client.rx_buf[2] == 0x01) // RFID从机
		{
			if (size > 0)
			{

				memcpy(RFID_client.Rx_RFID_buf, RFID_client.rx_buf, size);
				RFID_client.Rx_RFID_len = (uint8_t)size;
			}

			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RFID_client.rx_buf, (uint16_t)sizeof(RFID_client.rx_buf));

			xEventGroupSetBitsFromISR(eg, EVENT_RFID_RX, &xHigherPriorityTaskWoken);
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  
  if (huart->Instance == USART1)
  {
    (void)HAL_UART_DMAStop(huart);
    huart->Instance->ICR = USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_PECF;
    (void)uart_manage_enable_dma_recv(&echo_uart_manage);
  }

}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  (void)huart;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  (void)huart;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// 	if(UartHandle->Instance == huart2.Instance){
	// //		static int count = 0;
	// 		count11111++;

	// 	}
	/* Modbus RTU RX callback BEGIN */
	int i;
	for (i = 0; i < numberHandlers; i++)
	{
		if (mHandlers[i]->port == huart)
		{

			if (mHandlers[i]->xTypeHW == USART_HW)
			{
				RingAdd(&mHandlers[i]->xBufferRX, mHandlers[i]->dataRX);
				HAL_UART_Receive_IT(mHandlers[i]->port, &mHandlers[i]->dataRX, 1);
				xTimerResetFromISR(mHandlers[i]->xTimerT35, &xHigherPriorityTaskWoken);
			}
			break;
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  /* Modbus RTU RX callback END */

	/*
	 * Here you should implement the callback code for other UARTs not used by Modbus
	 *
	 *
	 * */
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        uart_send_completed_hook(&echo_uart_manage);
    }

    /* Modbus RTU TX callback BEGIN */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	int i;
	for (i = 0; i < numberHandlers; i++)
	{
		if (mHandlers[i]->port == huart)
		{
			// notify the end of TX
			xTaskNotifyFromISR(mHandlers[i]->myTaskModbusAHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
			break;
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	/* Modbus RTU TX callback END */

	/*
	 * Here you should implement the callback code for other UARTs not used by Modbus
	 *
	 * */


    return;
}





