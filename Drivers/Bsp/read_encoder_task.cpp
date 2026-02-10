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
uint16_t now_volume;           // 假设音量寄存器地址为103
uint16_t last_volume = 0x001E; // 初始音量为30

void RFID_master_thread(void *argument);
void RFID_OnFrame(RFIDClient *c, const uint8_t *frm, uint16_t len);
void RFID_CheckOffline(RFIDClient *c);
void RFID_WriteToModbusRegs(RFIDClient *c);
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
// RFID
osThreadId_t RFID_master_handle;
const osThreadAttr_t RFID_master_attributes = {
    .name = "RFIDMaster",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};
uint8_t first_findVolume = 1;
EventGroupHandle_t eg = NULL; // 初始化事件组为NULL
void EventGroupCreate_Init(void)
{
    if (eg == NULL)
    {
        eg = xEventGroupCreate();
    }
}
// ========== 内部函数：大端拼接 ==========
static uint32_t be_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           ((uint32_t)p[3] << 0);
}

// ========== 内部函数：解析RFID帧 ==========
static int parse_frame(const uint8_t *frm, uint16_t len,
                       uint8_t *out_rssi, uint8_t *out_soc, uint32_t *out_uid)
{
    if (len < 14)
        return 0;
    if (frm[0] != 0x1B || frm[1] != 0x39 || frm[2] != 0x01)
        return 0;

    *out_rssi = frm[7];
    *out_soc = frm[9];
    *out_uid = be_u32(&frm[10]);

    return 1;
}

// ========== 内部函数：查找UID ==========
static int find_uid(RFIDClient *c, uint32_t uid)
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if ((c->valid_bitmap & (1U << i)) && c->tags[i].uid == uid)
        {
            return i;
        }
    }
    return -1;
}

// ========== 内部函数：分配空闲槽位 ==========
static int alloc_slot(RFIDClient *c)
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if ((c->valid_bitmap & (1U << i)) == 0)
        {
            return i;
        }
    }
    return -1;
}

// rfid相关3个函数的实现
// 1.写入Modbus寄存器
void RFID_WriteToModbusRegs(RFIDClient *c)
{
    // 更新位图到 modbus_registers[3] 的低8位
    modbus_registers[REG_RFID_VALID] =
        (modbus_registers[REG_RFID_VALID] & 0xFF00) | c->valid_bitmap;

    // 写8组数据到 modbus_registers[4~27]
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        uint16_t base = REG_RFID_BASE + i * 3;

        if (c->valid_bitmap & (1U << i))
        {
            // 标签有效时写入数据
            modbus_registers[base + 0] = (uint16_t)(c->tags[i].uid >> 16);
            modbus_registers[base + 1] = (uint16_t)(c->tags[i].uid & 0xFFFF);
            modbus_registers[base + 2] = ((uint16_t)c->tags[i].rssi << 8) | c->tags[i].soc;
        }
        else
        {
            // 标签无效时清零
            modbus_registers[base + 0] = 0;
            modbus_registers[base + 1] = 0;
            modbus_registers[base + 2] = 0;
        }
    }
}
// 2.检查离线
void RFID_CheckOffline(RFIDClient *c)
{
    uint32_t now = xTaskGetTickCount();
    uint32_t timeout = pdMS_TO_TICKS(RFID_OFFLINE_MS);

    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if ((c->valid_bitmap & (1U << i)) == 0)
            continue;

        if ((uint32_t)(now - c->tags[i].last_seen_tick) > timeout)
        {
            c->valid_bitmap &= ~(1U << i);//标记标签无效
            printf("RFID offline: idx=%d, UID=0x%08X\n",
                   i, (unsigned int)c->tags[i].uid);
            memset(&c->tags[i], 0, sizeof(c->tags[i]));//清除标签结构体数据
        }
    }
}
// 3. 收到帧后更新
void RFID_OnFrame(RFIDClient *c, const uint8_t *frm, uint16_t len)
{
    uint8_t rssi, soc;
    uint32_t uid;

    if (!parse_frame(frm, len, &rssi, &soc, &uid))
    {
        printf("RFID parse fail\n");
        return;
    }

    uint32_t now = xTaskGetTickCount();

    int idx = find_uid(c, uid);
    if (idx < 0)
    {
        idx = alloc_slot(c);
        if (idx < 0)
        {
            printf("RFID slots full, UID=0x%08X\n", (unsigned int)uid);
            return;
        }
        c->valid_bitmap |= (1U << idx);
        c->tags[idx].uid = uid;
        printf("RFID new tag: idx=%d, UID=0x%08X\n", idx, (unsigned int)uid);
    }

    c->tags[idx].rssi = rssi;
    c->tags[idx].soc = soc;
    c->tags[idx].last_seen_tick = now;

    printf("RFID update: idx=%d, UID=0x%08X, RSSI=%d, SOC=%d\n",
           idx, (unsigned int)uid, rssi, soc);
}

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
modbusHandler_t rfid_app;

uint16_t bms_results[2] = {0};
uint16_t firstVolume_results[2] = {0};
modbus_t telegram[4];
modbus_t telegram2[3];

// HACK
uint16_t modbus_master_buf[128] = {0};
uint16_t modbus_master_buf2[128] = {0};

// static const uint16_t modbusRFID[] = {

// };

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
        //    static uint16_t buzzer_cmd = 0x0007;
        // telegram[2].u16reg = &buzzer_cmd;
        telegram[2].u16RegAdd = 0x0008;
        telegram[2].u16reg[0] = 0x0007;
        ModbusQuery(&bms_sound_light_app, telegram[2]);
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        if (err)
        {
            printf("BUZZER_3M write fail : %d \n", err);
        }
        else
        {
            printf("BUZZER_3M write success\n");
        }
        modbus_registers[STATUS_BUZZER] = 1;
        break;

    case BUZZER_7M:
        telegram[2].u16RegAdd = 0x0008;
        telegram[2].u16reg[0] = 0x0008;
        ModbusQuery(&bms_sound_light_app, telegram[2]);
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        if (err)
        {
            printf("BUZZER_7M write fail : %d \n", err);
        }
        else
        {
            printf("BUZZER_7M write success\n");
        }
        modbus_registers[STATUS_BUZZER] = 1;
        break;

    default: // OFF
        telegram[2].u16RegAdd = 0x0016;
        telegram[2].u16reg[0] = 0x0001;
        ModbusQuery(&bms_sound_light_app, telegram[2]);
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        if (err != OP_OK_QUERY)
        {
            printf("BUZZER_SOUND_STOP write fail : %d \n", err);
        }
        else
        {
            printf("BUZZER_SOUND_STOP write success\n");
        }
        modbus_registers[STATUS_BUZZER] = 0;
        break;
    }

    current = target;
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
            printf("LED on write fail : %d \n", err);
        }
        else
        {
            printf("LED on write success\n");
        }

        // 更新状态寄存器regs[100]且清除命令寄存器regs[0]
        modbus_registers[STATUS_LED_SWITCH] = 1;
        // 通知RX任务：我发送了命令，你可以等待响应了！
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
            printf("LED on write fail : %d \n", err);
        }
        else
        {
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

void ai_safy_master_thread(void *argument)

{
    memset(modbus_master_buf, 0, sizeof(modbus_master_buf));

    bms_sound_light_app.uModbusType = MB_MASTER;
    bms_sound_light_app.port = &huart8;
    bms_sound_light_app.u8id = 0; // For master it must be 0
    bms_sound_light_app.u16timeOut = 500;
    bms_sound_light_app.EN_Port = NULL;
    bms_sound_light_app.EN_Pin = 0;
    bms_sound_light_app.u16regs = modbus_master_buf;
    bms_sound_light_app.u16regsize = sizeof(modbus_master_buf) / sizeof(modbus_master_buf[0]);
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

    // read buzzer Volume
    // telegram[3].u8id = 2;
    // telegram[3].u8fct = MB_FC_READ_REGISTERS;
    // telegram[3].u16RegAdd = 0x0043;

    // telegram[3].u16CoilsNo = 0;
    // telegram[3].u16reg = firstVolume_results;

    TickType_t last_10s = xTaskGetTickCount();
    TickType_t last_500ms = xTaskGetTickCount();

    for (;;)
    {
        // printf("setVolu : %d\n",modbus_registers[103]);

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
            if (err != OP_OK_QUERY)
            {
                printf("bms led sound modbus master read fail : %d \n", err);
            }
            else
            {
                modbus_registers[STATUS_BMS_SOC] = telegram[0].u16reg[0];
                printf("bms led sound modbus master read success,soc = %d\n", telegram[0].u16reg[0]);
            }
        }
        // 设置音量
        now_volume = modbus_registers[103];

        // 读取音量寄存器
        if (now_volume != last_volume)
        { // 如果音量有变化
            telegram[2].u16RegAdd = 0x0006;
            telegram[2].u16reg[0] = now_volume;
            ModbusQuery(&bms_sound_light_app, telegram[2]); // 调整喇叭的实际输出音量。
            uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
            if (err == OP_OK_QUERY)
            {
                printf("BUZZER_VOLUME write success, lastvolume=%d , nowVolume=%d,modbusReg[103]:%d\n", last_volume, now_volume, modbus_registers[103]);
                last_volume = now_volume; // 更新上一次的音量记录以供下次比较使用。
            }
            else
            {
                printf("BUZZER_VOLUME write fail : %d \n", err);
            }
        }
        osDelay(500);
    }
}
// modbus_t telegram;
RFIDClient RFID_client;
void RFID_master_thread(void *argument)
{

    EventGroupCreate_Init();
    TickType_t last_check = xTaskGetTickCount();
    for (;;)
    {

        EventBits_t uxBits = xEventGroupWaitBits(
            eg,            // 事件组
            EVENT_RFID_RX, // 等待这个事件
            pdTRUE,        // 自动清除标志
            pdFALSE,       // 不需要等待所有位
            portMAX_DELAY  // 无限等待
        );
        if ((uxBits & EVENT_RFID_RX) != 0)
        {
            printf("Recevice rfid: ");
            printf("modbus_reg[3] = %04X (", modbus_registers[3]);
            // 打印二进制格式
            for (int i = 15; i >= 0; i--)
            {
                printf("%d", (modbus_registers[3] >> i) & 1);
                if (i % 4 == 0 && i != 0)
                    printf(" "); // 每4位加空格分隔
            }
            // printf(")\n");
            // for (int i = 0; i < 12; i++)
            // {
            //     printf("%04X ", modbus_registers[i + 4]);
            //     if (i % 3 == 0)
            //         printf("\n");
            // }
            // printf("\n");
            RFID_OnFrame(&RFID_client,
                         RFID_client.Rx_RFID_buf,
                         RFID_client.Rx_RFID_len);

            // API 3: 立即写入Modbus寄存器
            RFID_WriteToModbusRegs(&RFID_client);
        }

        // 每1秒检查一次离线
        if ((TickType_t)(xTaskGetTickCount() - last_check) >= pdMS_TO_TICKS(5000))
        {
            last_check = xTaskGetTickCount();
            RFID_CheckOffline(&RFID_client);
            RFID_WriteToModbusRegs(&RFID_client);
        }

        osDelay(1000);
    }
}

void init_read_encoder_task()
{
    init_ai_safy_slave();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RFID_client.rx_buf, (uint16_t)sizeof(RFID_client.rx_buf));
    // modbus_registers[0] = 0;
    // 初始音量调为最大
    modbus_registers[103] = 0x001E; // 30的十六进制
    now_volume = 0x1E;
    // modbus_registers[3]=0b10000101;
    // modbus_registers[4]=0x1102;
    // modbus_registers[5]=0x5539;
    // modbus_registers[6]=0x5021;
    // modbus_registers[7]=0x1102;
    // modbus_registers[8]=0x5540;
    // modbus_registers[9]=0x3221;
    // modbus_registers[10]=0x1102;
    // modbus_registers[11]=0x5541;
    // modbus_registers[12]=0x4521;
    // modbus_registers[13]=0x1102;
    // modbus_registers[14]=0x5542;
    // modbus_registers[15]=0x7621;
    ai_safy_master_handle = osThreadNew(ai_safy_master_thread, NULL, &ai_safy_master_attributes);
    RFID_master_handle = osThreadNew(RFID_master_thread, NULL, &RFID_master_attributes);
}
