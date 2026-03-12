
#include <heartbeat_task.h>
#include <board.h>

void StartHeartbeatTask(void *argument)
{
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
   for(;;)
   {
    // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
#ifdef DEBUG_TASK_STACK
    UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    printf("HeartbeatTask stack high water mark: %lu bytes\r\n", stackHighWaterMark);
#endif
    osDelay(1000);
   }
}
