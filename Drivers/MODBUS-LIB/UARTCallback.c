/*
 * UARTCallback.c
 *
 *  Created on: May 27, 2020
 *      Author: Alejandro Mera
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "main.h"
#include "Modbus.h"
#include "async_logger.h"
#include "string.h"
extern RFIDClient RFID_client;
extern EventGroupHandle_t eg; // 初始化事件组为NULL
/**
 * @brief
 * This is the callback for HAL interrupts of UART TX used by Modbus library.
 * This callback is shared among all UARTS, if more interrupts are used
 * user should implement the correct control flow and verification to maintain
 * Modbus functionality.
 * @ingroup UartHandle UART HAL handler
 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
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
}

/**
 * @brief
 * This is the callback for HAL interrupt of UART RX
 * This callback is shared among all UARTS, if more interrupts are used
 * user should implement the correct control flow and verification to maintain
 * Modbus functionality.
 * @ingroup UartHandle UART HAL handler
 */
// int count11111 = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// 	if(UartHandle->Instance == huart2.Instance){
	// //		static int count = 0;
	// 		count11111++;

	// 	}
	/* Modbus RTU RX callback BEGIN */
	int i;
	for (i = 0; i < numberHandlers; i++)
	{
		if (mHandlers[i]->port == UartHandle)
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

/*
 * DMA requires to handle callbacks for special communication modes of the HAL
 * It also has to handle eventual errors including extra steps that are not automatically
 * handled by the HAL
 * */
// uint16_t count222 = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (huart->Instance == huart2.Instance)
	{
		// 		static int count = 0;
		// count222++;
		if (RFID_client.rx_buf[0] == 0x1B && RFID_client.rx_buf[1] == 0x39 && RFID_client.rx_buf[2] == 0x01) // RFID从机
		{
			if (Size > 0)
			{

				memcpy(RFID_client.Rx_RFID_buf, RFID_client.rx_buf, Size);
				RFID_client.Rx_RFID_len = (uint8_t)Size;
			}

			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RFID_client.rx_buf, (uint16_t)sizeof(RFID_client.rx_buf));

			xEventGroupSetBitsFromISR(eg, EVENT_RFID_RX, &xHigherPriorityTaskWoken);
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#if ENABLE_USART_DMA == 1
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	/* Modbus RTU RX callback BEGIN */
	int i;
	for (i = 0; i < numberHandlers; i++)
	{
		if (mHandlers[i]->port == huart)
		{

			if (mHandlers[i]->xTypeHW == USART_HW_DMA)
			{
				if (Size) // check if we have received any byte
				{
					mHandlers[i]->xBufferRX.u8available = Size;
					mHandlers[i]->xBufferRX.overflow = false;

					while (HAL_UARTEx_ReceiveToIdle_DMA(mHandlers[i]->port, mHandlers[i]->xBufferRX.uxBuffer, MAX_BUFFER) != HAL_OK)
					{
						HAL_UART_DMAStop(mHandlers[i]->port);
					}
					__HAL_DMA_DISABLE_IT(mHandlers[i]->port->hdmarx, DMA_IT_HT); // we don't need half-transfer interrupt

					xTaskNotifyFromISR(mHandlers[i]->myTaskModbusAHandle, 0, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
				}
			}

			break;
		}
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

	int i;

	for (i = 0; i < numberHandlers; i++)
	{
		if (mHandlers[i]->port == huart)
		{

			if (mHandlers[i]->xTypeHW == USART_HW_DMA)
			{
				while (HAL_UARTEx_ReceiveToIdle_DMA(mHandlers[i]->port, mHandlers[i]->xBufferRX.uxBuffer, MAX_BUFFER) != HAL_OK)
				{
					HAL_UART_DMAStop(mHandlers[i]->port);
				}
				__HAL_DMA_DISABLE_IT(mHandlers[i]->port->hdmarx, DMA_IT_HT); // we don't need half-transfer interrupt
			}

			break;
		}
	}
}

#endif
