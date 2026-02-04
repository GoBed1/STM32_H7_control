#include "user_logic.h"
// #include "read_encoder_task.h"
#include "encoder_forward_app.h"
#include "modbus_rtu.hpp"
// extern ModbusRtu_Resend_t Resend;
// extern ModbusRtuClient encoder_client;
// #include "Modbus.h"
extern ModbusRtu bms_led_sound_app;
extern EventGroupHandle_t eg;
extern nmbs_server_t slave_data;
extern uint16_t modbus_registers[REGS_TOTAL_NUM];

#define CMD_LED_SWITCH 0
#define STATUS_LED_SWITCH 100
// 喇叭
#define CMD_BUZZER_7m 1
#define CMD_BUZZER_3m 2
// #define CMD_BUZZER_stop 3

#define STATUS_BUZZER 101
#define STATUS_BMS_SOC 102
// #define STATUS_BUZZER_3m 102
// #define STATUS_BUZZER_stop 103

typedef enum
{
    BUZZER_OFF = 0,
    BUZZER_7M = 1,
    BUZZER_3M = 2,
} buzzer_mode_t;

void buzzer_logic(void);
uint16_t BMS_ParseSOC(uint8_t data[], uint16_t data_len);
// 处理接收到的数据包，并返回处理结果
mb_err_op_t modbus_RxData_logic(uint8_t *Rx_data, uint16_t RxLen)
{

    // 提取地址和功能码
    uint8_t slave_id = Rx_data[0];
    uint8_t func_code = Rx_data[1];

    // 根据从机地址分类处理
    switch (slave_id)
    {
    case 0x01: // 灯
        switch (func_code)
        {
        case 0x03: // 读取寄存器
                   // 做crc校验
            // debug_println("CRC OK！");
            return OP_OK_QUERY;
            // break;

        case 0x06: // 写寄存器
            if (RxLen != 8)
                return ERR_BAD_SIZE;
            uint16_t crc = YX95R_CRC16_Calc(Rx_data, RxLen - 2);
            uint16_t received_crc = ((uint16_t)Rx_data[RxLen - 1] << 8) | Rx_data[RxLen - 2];
            if (crc == received_crc)
            {
                // 错误次数清0
                Resend.crcError_resend_count = 0;
                debug_println("LED CRC OK！");
                return OP_OK_QUERY;
            }
            else
            {
                // 重新发送命令并计算发送次数，大于3次，如果还是错误则返回错误
                if (Resend.crcError_resend_count < Resend.crcError_max_resend)
                {
                    Resend.crcError_resend_count++;
                    // 重新发送命令×
                    YX95R_RGB_Write_Register(encoder_client.tx_buf);
                }
                else
                {
                    Resend.crcError_resend_count = 0;
                    // debug_println("CRC ERROR！");
                    return ERR_BAD_CRC;
                }
            }

            break;

        default:
            // status.success = 0;
            return ERR_FUC_CODE;
        }
        break;

    case 0x02: // 喇叭
        switch (func_code)
        {
        case 0x03: // 读取寄存器
            // status.device = DEVICE_LIGHT;
            // status.func = FUNC_READ;
            // status.data_len = Rx_data[2];
            // status.reg_value = (Rx_data[3] << 8) | Rx_data[4];
            // status.success = 1;
            break;

        case 0x06: // 写寄存器
            uint16_t crc = YX95R_CRC16_Calc(Rx_data, RxLen - 2);
            uint16_t received_crc = ((uint16_t)Rx_data[RxLen - 1] << 8) | Rx_data[RxLen - 2];
            if (crc == received_crc)
            {
                // 错误次数清0
                Resend.crcError_resend_count = 0;
                debug_println("SOUND CRC OK！");
                return OP_OK_QUERY;
            }
            else
            {
                // 重新发送命令并计算发送次数，大于3次，如果还是错误则返回错误
                if (Resend.crcError_resend_count < Resend.crcError_max_resend)
                {
                    Resend.crcError_resend_count++;
                    // 重新发送命令×
                    YX95R_RGB_Write_Register(encoder_client.tx_buf);
                }
                else
                {
                    Resend.crcError_resend_count = 0;
                    // debug_println("CRC ERROR！");
                    return ERR_BAD_CRC;
                }
            }
            break;

        default:
            // status.success = 0;
            return ERR_FUC_CODE;
        }
        break;
    case 0x03: // lora-485
        switch (func_code)
        {
        case 0x03: // 读取寄存器

            break;

        case 0x06: // 写寄存器
            debug_println("LORA-485 WRITE！");
            break;

        default:
            // status.success = 0;
            return ERR_FUC_CODE;
        }
        break;
    case 0x04: // bms电源读取
        switch (func_code)
        {
        case 0x03: // 读取寄存器,把电量读出来，写入保持reg[102]
            // taskENTER_CRITICAL();
            // debug_println("Recevice bms SOC: ");
            // for (int i = 0; i < encoder_client.rx_frame_len; i++)
            // {
            //     debug_println("%02X ", encoder_client.parse_buf[i]);
            // }
            // taskEXIT_CRITICAL();
            // 更新状态寄存器regs[102]
            uint16_t SOC = BMS_ParseSOC(encoder_client.parse_buf, (uint16_t)encoder_client.rx_frame_len);
            slave_data.regs[STATUS_BMS_SOC] = SOC;
            debug_println("SOC: %d", slave_data.regs[STATUS_BMS_SOC]);
            break;

        case 0x06: // 写寄存器
            debug_println("LORA-485 WRITE！");
            break;

        default:
            // status.success = 0;
            return ERR_FUC_CODE;
        }
        break;
    default:
        // 未知的从机地址
        // status.success = 0;
        return ERR_BAD_ADDRESS;
    }
    // return ERR_BAD_ADDRESS;
}
// 处理发送的数据包，并返回处理结果
void modbus_TxData_logic(void)
{
    uint16_t cmd_led_switch = modbus_registers[CMD_LED_SWITCH];
    // 读取寄存器0的值，例如
    // 灯打开或关闭命令
    if (cmd_led_switch == 1 && modbus_registers[STATUS_LED_SWITCH] == 0)
    {
        // 红灯打常量开
        // 记录超时或者crc校验重复发送命令的逻辑
        record_tx_cmd(cmd_red_com, 6);
        YX95R_LIGHT_ON_RED_COM;
        bms_led(0x01, 0x0001, 0x0001);
        // 更新状态寄存器regs[100]且清除命令寄存器regs[0]
        slave_data.regs[STATUS_LED_SWITCH] = 1;
        // 通知RX任务：我发送了命令，你可以等待响应了！
        xEventGroupSetBits(eg, EVENT_CMD_SENT);
    }
    if (cmd_led_switch == 0 && slave_data.regs[STATUS_LED_SWITCH] == 1)
    {
        // 灯关闭
        // debug_println(" LED off ");
        // YX95R_RGB_Light_Off(1); // 红色慢闪
        record_tx_cmd(cmd_off, 6);

        YX95R_LIGHT_OFF;
        slave_data.regs[STATUS_LED_SWITCH] = 0;
        // 通知RX任务：我发送了命令，你可以等待响应了！
        xEventGroupSetBits(eg, EVENT_CMD_SENT);
    }
    // 喇叭逻辑处理
    buzzer_logic();

    // bms电源读取发送
    // bms_read_logic();
}

// //bms电源读取发送逻辑
// void bms_read_logic(void)
// {

// }

// 超时重发逻辑
void timeout_resend_logic(void)
{
    // 处理超时重发次数++
    Resend.timeout_resend_count++;
    if (Resend.timeout_resend_count > Resend.timeout_max_resend)
    {
        // 重发次数过多，重置计数器并退出任务
        Resend.timeout_resend_count = 0;

        debug_println("Max timeout resend reached. Exiting task.");
    }
    else
    {
        // debug_println("Resend command due to timeout...%d", Resend.timeout_resend_count);
        // 测试重新发指令
        YX95R_RGB_Write_Register(encoder_client.tx_buf);
        // 发送标志位置为0
        xEventGroupSetBits(eg, EVENT_CMD_SENT);
    }
}

// 喇叭逻辑处理函数
void buzzer_logic(void)
{
    // 算目标模式：3m 优先于 7m，3m覆盖7m，二者都为0时关闭喇叭。
    buzzer_mode_t target = BUZZER_OFF;

    if (slave_data.regs[CMD_BUZZER_3m] == 1)
    {
        target = BUZZER_3M;
    }
    else if (slave_data.regs[CMD_BUZZER_7m] == 1)
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

    switch (target)
    {
    case BUZZER_3M:
        record_tx_cmd(cmd_sound_3m_warn, 6);
        BUZZER_SOUND_3M_WARN;
        slave_data.regs[STATUS_BUZZER] = 1;
        break;

    case BUZZER_7M:
        record_tx_cmd(cmd_sound_7m_warn, 6);
        BUZZER_SOUND_7M_WARN;
        slave_data.regs[STATUS_BUZZER] = 1;
        break;

    default: // OFF
        record_tx_cmd(cmd_sound_stop, 6);
        BUZZER_SOUND_STOP;
        slave_data.regs[STATUS_BUZZER] = 0;
        break;
    }

    current = target;

    // 只有真的发了命令才通知 RX 任务
    xEventGroupSetBits(eg, EVENT_CMD_SENT);
}
// 记录发送命令到TX BUF中,方便重发和查看
void record_tx_cmd(uint8_t *cmd, uint8_t len)
{
    for (int i = 0; i < len; i++)
    {
        encoder_client.tx_buf[i] = cmd[i];
    }
}
// 对bms读取的soc解析函数，返回0-100百分比值，出错返回0xFFFF
uint16_t BMS_ParseSOC(uint8_t data[], uint16_t data_len)
{
    if (data_len < 7)
    {
        return 0xFFFF;
    }

    if (data[1] != 0x03)
    {
        return 0xFFFF;
    }

    if (data[2] != 2)
    {
        return 0xFFFF;
    }
//crc校验
    uint16_t crc_received = ((uint16_t)data[data_len - 1] << 8) | data[data_len - 2];
    uint16_t crc_calc = YX95R_CRC16_Calc(data, data_len - 2);

    if (crc_received == crc_calc)
    {
        debug_println("BMS CRC OK");
    }

    //取SOC原始值
    uint16_t soc_raw = ((uint16_t)data[3] << 8) | data[4];

   //四舍五入转换为0-100百分比
    uint16_t soc_percent = (soc_raw + 50) / 100;

    /* ===== 第7步：限制范围0-100 ===== */
    /* 超过99都改成100 */
    if (soc_percent > 99)
    {
        soc_percent = 100;
    }

    return soc_percent; /* 返回0-100 */
}
