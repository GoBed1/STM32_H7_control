#include "read_encoder_task.h"
#include "encoder_forward_app.h"

#define RET_D(...) // printf(__VA_ARGS__)
#define RET_I(...) printf(__VA_ARGS__)
#define RET_E(...) printf(__VA_ARGS__)

#include "read_encoder_task.h"
#define CMD_LED_SWITCH 0
#define STATUS_LED_SWITCH 100
// 喇叭
#define CMD_BUZZER_7m 1
#define CMD_BUZZER_3m 2
// #define CMD_BUZZER_stop 3

#define STATUS_BUZZER 101
#define STATUS_BMS_SOC 102

extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart8;

// Slave全局变量
static modbusHandler_t encoder_forward_server;
static modbusHandler_t encoder_forward_server_backup;
uint16_t modbus_registers[REGS_TOTAL_NUM] = {0};


osThreadId_t ai_safy_slave_handle;
const osThreadAttr_t ai_safy_slave_attributes = {
    .name = "AISafySlave",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t ai_safy_master_handle;
const osThreadAttr_t ai_safy_master_attributes = {
    .name = "AISafyMaster",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};

void init_ai_safy_slave(void)
{
    // 初始化寄存器数组
    memset(modbus_registers, 0, sizeof(modbus_registers));

    // HACK 设置系统信息
    modbus_registers[REG_ERROR_CODE] = 0x0000; // 清零错误码/在线状态
    modbus_registers[REG_TOTAL_SENSORS] = 3;
    modbus_registers[REG_SYSTEM_VERSION] = HUB_SLAVE_VERSION; // 记录系统版本号

    // 配置MODBUS Slave处理器
    encoder_forward_server.uModbusType = MB_SLAVE;
    encoder_forward_server.u8id = FORWARD_SLAVE_ADDR; // Slave ID=3
    encoder_forward_server.port = &huart7;
    encoder_forward_server.EN_Port = NULL; // 无RS485控制引脚
    encoder_forward_server.EN_Pin = 0;
    encoder_forward_server.u16regs = modbus_registers; // 保持寄存器
    encoder_forward_server.u16regsize = REGS_TOTAL_NUM;
    encoder_forward_server.u16timeOut = 1000; // 1秒超时
    encoder_forward_server.xTypeHW = USART_HW;
    RET_I("MODBUS-RTU Slave ID: %d\r\n", FORWARD_SLAVE_ADDR);

    // 初始化MODBUS Slave
    ModbusInit(&encoder_forward_server);
    ModbusStart(&encoder_forward_server);

    RET_I("Register range: 0x0000-0x%04X (%d registers)\r\n",
          REGS_TOTAL_NUM - 1, REGS_TOTAL_NUM);
}

modbusHandler_t bms_sound_light_app;
uint16_t bms_results[2] = {0};
modbus_t telegram[3];
// HACK
uint16_t modbus_master_buf[128] = {0};

// 喇叭逻辑处理函数
void buzzer_logic(void)
{
    // 算目标模式：3m 优先于 7m，3m覆盖7m，二者都为0时关闭喇叭。
    buzzer_mode_t target = BUZZER_OFF;

    if (modbus_registers[CMD_BUZZER_3m] == 1)
    {
        target = BUZZER_3M;
    }
    else if (modbus_registers[CMD_BUZZER_7m] == 1)
    {
        target = BUZZER_7M;
    }
    else
    {
        target = BUZZER_OFF;
    }

    // 只在变化时执行
    static buzzer_mode_t current = BUZZER_OFF;
    if (target == current)
        return;
    uint32_t err = 0;
    switch (target)
    {
    case BUZZER_3M:
        // record_tx_cmd(cmd_sound_3m_warn, 6);
        // BUZZER_SOUND_3M_WARN;
        // bms_led_sound_app.asyncWriteSingle(0x02, 0x0008, 0x0007);
        telegram[2].u16RegAdd = 0x0008; 
        telegram[2].u16reg[0] = 0x0007; 
        ModbusQuery(&bms_sound_light_app, telegram[2]);
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        if (err)
        {
            printf("BUZZER_3M write fail : %d \n",err);
        }else{
            printf("BUZZER_3M write success\n");
        }
        modbus_registers[STATUS_BUZZER] = 1;
        break;

    case BUZZER_7M:
        // record_tx_cmd(cmd_sound_7m_warn, 6);
        // BUZZER_SOUND_7M_WARN;
        // HACK
        // bms_led_sound_app.asyncWriteSingle(0x02, 0x0008, 0x0008);
        telegram[2].u16RegAdd = 0x0008; 
        telegram[2].u16reg[0] = 0x0008; 
        ModbusQuery(&bms_sound_light_app, telegram[2]);
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        if (err)
        {
            printf("BUZZER_7M write fail : %d \n",err);
        }else{
            printf("BUZZER_7M write success\n");
        }
        modbus_registers[STATUS_BUZZER] = 1;
        break;

    default: // OFF
        // record_tx_cmd(cmd_sound_stop, 6);
        // BUZZER_SOUND_STOP;
        // HACK
        // bms_led_sound_app.asyncWriteSingle(0x02, 0x0016, 0x0001);
        telegram[2].u16RegAdd = 0x0016; 
        telegram[2].u16reg[0] = 0x0001; 
        ModbusQuery(&bms_sound_light_app, telegram[2]);
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        if (err)
        {
            printf("BUZZER_SOUND_STOP write fail : %d \n",err);
        }else{
            printf("BUZZER_SOUND_STOP write success\n");
        }
        modbus_registers[STATUS_BUZZER] = 0;
        break;
    }

    current = target;

    // 只有真的发了命令才通知 RX 任务
    // xEventGroupSetBits(eg, EVENT_CMD_SENT);
}

void modbus_TxData_logic(void)
{
    uint16_t cmd_led_switch = modbus_registers[CMD_LED_SWITCH];
    // 读取寄存器0的值，例如
    // 灯打开或关闭命令
    if (cmd_led_switch == 1 && modbus_registers[STATUS_LED_SWITCH] == 0)
    {
        // 红灯打常量开
        // 记录超时或者crc校验重复发送命令的逻辑
        // record_tx_cmd(cmd_red_com, 6);
        // YX95R_LIGHT_ON_RED_COM;
        printf(" LED on \n");
        // bms_led_sound_app.asyncWriteSingle(0x01,0x00c2,0x0041);
        telegram[1].u16RegAdd = 0x00C2;
        telegram[1].u16reg[0] = 0x0041; 
        ModbusQuery(&bms_sound_light_app, telegram[1]);
        uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)); 
        if (err)
        {
            printf("LED on write fail : %d \n",err);
        }else{
            printf("LED on write success\n");
        }
        // StartTaskMaster();
        // 看看发出去的数据是啥sendTxBuffer

        // 更新状态寄存器regs[100]且清除命令寄存器regs[0]
        modbus_registers[STATUS_LED_SWITCH] = 1;
        // 通知RX任务：我发送了命令，你可以等待响应了！
        // xEventGroupSetBits(eg, EVENT_CMD_SENT);
    }
    if (cmd_led_switch == 0 && modbus_registers[STATUS_LED_SWITCH] == 1)
    {
        // 灯关闭
        // debug_println(" LED off ");
        // YX95R_RGB_Light_Off(1); // 红色慢闪
        // record_tx_cmd(cmd_off, 6);

        // YX95R_LIGHT_OFF;
        printf(" LED off \n");
        // HACK
        // bms_led_sound_app.asyncWriteSingle(0x01, 0x00c2, 0x0060);
        telegram[1].u16RegAdd = 0x00C2;
        telegram[1].u16reg[0] = 0x0060; 
        ModbusQuery(&bms_sound_light_app, telegram[1]);
        uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)); 
        if (err)
        {
            printf("LED on write fail : %d \n",err);
        }else{
            printf("LED on write success\n");
        }
        modbus_registers[STATUS_LED_SWITCH] = 0;
        // 通知RX任务：我发送了命令，你可以等待响应了！
        // xEventGroupSetBits(eg, EVENT_CMD_SENT);
    }

    // 喇叭逻辑处理
    buzzer_logic();

    // bms电源读取发送
    // bms_read_logic();
}

void ai_safy_master_thread(void*arrgument)

{
    memset(modbus_master_buf,0,sizeof(modbus_master_buf));

    bms_sound_light_app.uModbusType = MB_MASTER;
    bms_sound_light_app.port =  &huart8;
    bms_sound_light_app.u8id = 0; // For master it must be 0
    bms_sound_light_app.u16timeOut = 500;
    bms_sound_light_app.EN_Port = NULL;
    bms_sound_light_app.EN_Pin = 0;
    bms_sound_light_app.u16regs = modbus_master_buf;
    bms_sound_light_app.u16regsize= sizeof(modbus_master_buf)/sizeof(modbus_master_buf[0]);
    bms_sound_light_app.xTypeHW = USART_HW;
    ModbusInit(&bms_sound_light_app);
    ModbusStart(&bms_sound_light_app);

    printf("bms sound light modbus master start \n");

    // read bms
    telegram[0].u8id = 4; 
    telegram[0].u8fct = MB_FC_READ_REGISTERS; 
    telegram[0].u16RegAdd = 0x0000;
    telegram[0].u16CoilsNo = 1; 
    telegram[0].u16reg = bms_results;

    // write light control
    telegram[1].u8id = 1;                    
    telegram[1].u8fct = MB_FC_WRITE_REGISTER; 
    telegram[1].u16CoilsNo = 1; 

    // write buzzer control
    telegram[2].u8id = 2;                     
    telegram[2].u8fct = MB_FC_WRITE_REGISTER; 
    telegram[2].u16CoilsNo = 1; 

    TickType_t last_10s = xTaskGetTickCount();
    TickType_t last_500ms = xTaskGetTickCount();

    for (;;)
    {
        // 每 500ms 执行一次
        if (xTaskGetTickCount() - last_500ms >= pdMS_TO_TICKS(500))
        {
            last_500ms += pdMS_TO_TICKS(500);
            modbus_TxData_logic();
        }
        // 每 10s 执行一次（读取电量/电流）
        if (xTaskGetTickCount() - last_10s >= pdMS_TO_TICKS(10000))
        {
            last_10s += pdMS_TO_TICKS(10000);
            ModbusQuery(&bms_sound_light_app, telegram[0]);
            int err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)); 
            if (err!=OP_OK_QUERY)
            {
            printf("bms led sound modbus master read fail : %d \n",err);
            }else{
            printf("bms led sound modbus master read success,soc = %d\n",telegram[0].u16reg[0]);
            }
        }
        osDelay(500);
    }
}

void init_read_encoder_task()
{
    init_ai_safy_slave();
    ai_safy_master_handle = osThreadNew(ai_safy_master_thread, NULL, &ai_safy_master_attributes);
}
