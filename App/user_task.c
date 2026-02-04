#include "user_task.h"
#include "cmsis_os.h"
#include "com_debug.h"
#include "user_logic.h"
#include "semphr.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern ModbusRtuClient encoder_client;
extern ModbusRtuClient RFID_client;
extern EventGroupHandle_t eg;
extern ModbusRtu_Resend_t Resend;
extern uint8_t test_tx_cmd[8];
// extern SemaphoreHandle_t uart2_rx_semaphore;

// 全局二值信号量（一个管“能不能发”（总线锁），一个管“应答到了没”（应答信号））
// SemaphoreHandle_t mb_bus_sem  = NULL;
// SemaphoreHandle_t mb_resp_sem = NULL;

osThreadId_t RFID_handle = NULL;

// ✅ 定义 Modbus 从机全局变量
nmbs_t modbus_slave;

nmbs_server_t slave_data = {
    .id = 0x03, // 从机地址（LoRa转485模块的地址）
    .coils = {0},
    .regs = {0}};

osThreadId_t user_task_handle;
const osThreadAttr_t user_task_attributes = {
    .name = "userTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

// 创建灯控制任务
osThreadId_t lightTaskHandle;
const osThreadAttr_t lightTask_attributes = {
    .name = "LightTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

// 创建接收控制任务
osThreadId_t modbusRecvTaskHandle;
const osThreadAttr_t modbusRecvTask_attributes = {
    .name = "modbusRecvTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
// 接收工控机数据任务
osThreadId_t RecvMasterTaskHandle;
const osThreadAttr_t RecvMasterTask_attributes = {
    .name = "RecvMasterTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
// 读取电量任务
osThreadId_t ReadSocTaskHandle;
const osThreadAttr_t ReadSocTask_attributes = {
    .name = "ReadSocTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void init_user_task(void)
{
  memset(&encoder_client, 0, sizeof(encoder_client));
  encoder_client.huart = &huart2;
  encoder_client.task_handle = NULL;
  encoder_client.rx_frame_len = 0;
  encoder_client.rx_timeout = 0;

  //  mb_bus_sem  = xSemaphoreCreateBinary();//总线令牌（谁拿到谁才能发）
  //   mb_resp_sem = xSemaphoreCreateBinary();//应答到达信号，初始保持“空”，等 ISR 收到匹配应答后 give

//  xSemaphoreGive(mb_bus_sem); // 初始化时，总线令牌给出去

  // 创建事件组（用来通知RX任务）
  EventGroupCreate_Init();
  // 初始化串口2接收DMA
  HAL_UARTEx_ReceiveToIdle_DMA(encoder_client.huart, encoder_client.rx_buf, (uint16_t)sizeof(encoder_client.rx_buf));

  // 初始化Modbus客户端
  // ✅ 初始化寄存器默认值
  nmbs_error err = nmbs_server_init(&modbus_slave, &slave_data);

  user_task_handle = osThreadNew(start_user_task, NULL, &user_task_attributes); // 创建用户任务线程
                                                                                // 爆闪灯任务
  lightTaskHandle = osThreadNew(SOUND_LED_task, NULL, &lightTask_attributes);   // 灯控任务线程

  modbusRecvTaskHandle = osThreadNew(ModbusRecv_task, NULL, &modbusRecvTask_attributes); // modbus接收任务线程

  RecvMasterTaskHandle = osThreadNew(RecvMaster_task, NULL, &RecvMasterTask_attributes); // 接收工控机数据任务线程

  ReadSocTaskHandle = osThreadNew(ReadSoc_task, NULL, &ReadSocTask_attributes); // 读取电量任务线程
}

void start_user_task(void *argument)
{
  for (;;)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    osDelay(1000);
  }
}
// 爆闪灯
void SOUND_LED_task(void *argument)
{
  debug_println("SOUND_LED_task started...........");

  TickType_t last_500ms = xTaskGetTickCount();
  TickType_t last_10s = xTaskGetTickCount();

  for (;;)
  {
    // 每 500ms 执行一次
    if (xTaskGetTickCount() - last_500ms >= pdMS_TO_TICKS(500))
    {
      last_500ms += pdMS_TO_TICKS(500);
      modbus_TxData_logic();
    }

    // 每 10s 执行一次（读取电量/电流）
    if (xTaskGetTickCount() - last_10s >= pdMS_TO_TICKS(5000))
    {
      last_10s += pdMS_TO_TICKS(5000);
      record_tx_cmd(cmd_read_soc, 6);
      BMS_READ_SOC;                           // 你的读取电量宏/函数
      debug_println("BMS_READ_SOC122221");
      xEventGroupSetBits(eg, EVENT_CMD_SENT); // 通知接收任务去等 ACK
    }

    // 小延时，避免空转占CPU
    osDelay(500);
  }
}
// modbus接收任务
void ModbusRecv_task(void *argument)
{

  debug_println("ModbusRecv_ACk_task started...........");

  ModbusRtuClient *client = &encoder_client;
  client->task_handle = xTaskGetCurrentTaskHandle();
  TickType_t last_wake = xTaskGetTickCount();

  for (;;)
  {
    // ✅ 等待TX任务发送命令的事件
    EventBits_t uxBits = xEventGroupWaitBits(
        eg,             // 事件组
        EVENT_CMD_SENT, // 等待这个事件
        pdTRUE,         // 自动清除标志
        pdFALSE,        // 不需要等待所有位
        portMAX_DELAY   // 无限等待
    );
    // 发送命令
    if ((uxBits & EVENT_CMD_SENT) != 0)
    {
      // 等待中断唤醒（或 3 秒超时）
      uint32_t notif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3000));

      if (notif != 0)
      {

        // debug_println("Recevice1: ");
        // for (int i = 0; i < encoder_client.rx_frame_len; i++)
        // {
        //   debug_println("%02X ", encoder_client.parse_buf[i]);
        // }
        // ✅ 有数据！解析处理
        // timeout count 清零
        Resend.timeout_resend_count = 0;
        // 解析数据帧
        mb_err_op_t status = modbus_RxData_logic(encoder_client.parse_buf, encoder_client.rx_frame_len);
        if (status != OP_OK_QUERY)
        {
          debug_println("slave recv ACK error-%d !", status);
        }
      }
      else
      {

        // 超时重发逻辑
        timeout_resend_logic();
      }
    }
    // 下一轮轮询（100ms）
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
  }
}

// 接收工控机数据任务(从机任务slave task)
void RecvMaster_task(void *argument)
{
  debug_println("modbusSlave_task started...........");
  for (;;)
  {

    // ✅ 轮询处理 Modbus 请求
    nmbs_error a = nmbs_server_poll(&modbus_slave);
    // debug_println("modbusSlave_task poll result: %d", a);

    osDelay(500); // 根据需要调整延迟时间
  }
}

// // 读取电量任务
void ReadSoc_task(void *argument)
{
  debug_println("ReadSoc_task started...........");
  RFID_handle = xTaskGetCurrentTaskHandle();

  for (;;)
  {
    // 读取电量逻辑
    // BMS_READ_SOC;
    // xEventGroupSetBits(eg, EVENT_CMD_SENT);
    // 一直等待接收数据
    // ✅ 等待TX任务发送命令的事件
    EventBits_t uxBits = xEventGroupWaitBits(
        eg,            // 事件组
        EVENT_RFID_RX, // 等待这个事件
        pdTRUE,        // 自动清除标志
        pdFALSE,       // 不需要等待所有位
        portMAX_DELAY  // 无限等待
    );
    // 发送命令
    if ((uxBits & EVENT_RFID_RX) != 0)
    {
      taskENTER_CRITICAL();
      debug_println("Recevice1: ");
      for (int i = 0; i < RFID_client.Rx_RFID_len; i++)
      {
        debug_println("%02X ", RFID_client.Rx_RFID_buf[i]);
      }
      taskEXIT_CRITICAL();
    }

    osDelay(1000); // 根据需要调整延迟时间
  }
}
