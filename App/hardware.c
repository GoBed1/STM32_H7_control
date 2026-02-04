#include "hardware.h"

/* ============================================================================
 *              YX95R-485M 爆闪灯 Modbus RTU 控制库 - 实现部分
 * ============================================================================
 */
extern UART_HandleTypeDef huart2;

const uint8_t cmd_red_com[6] = {0x01,0x06,0x00,0xC2,0x00,0x41};
const uint8_t cmd_red_slow[6] = {0x01,0x06,0x00,0xC2,0x00,0x51};
const uint8_t cmd_red_fast[6] = {0x01,0x06,0x00,0xC2,0x00,0x61};
const uint8_t cmd_yellow_com[6] = {0x01,0x06,0x00,0xC2,0x00,0x42};
const uint8_t cmd_yellow_slow[6] = {0x01,0x06,0x00,0xC2,0x00,0x52};
const uint8_t cmd_yellow_fast[6] = {0x01,0x06,0x00,0xC2,0x00,0x62};
const uint8_t cmd_green_com[6] = {0x01,0x06,0x00,0xC2,0x00,0x43};
const uint8_t cmd_green_slow[6] = {0x01,0x06,0x00,0xC2,0x00,0x53};
const uint8_t cmd_green_fast[6] = {0x01,0x06,0x00,0xC2,0x00,0x63};
const uint8_t cmd_white_com[6] = {0x01,0x06,0x00,0xC2,0x00,0x44};
const uint8_t cmd_white_slow[6] = {0x01,0x06,0x00,0xC2,0x00,0x54};
const uint8_t cmd_white_fast[6] = {0x01,0x06,0x00,0xC2,0x00,0x64};
// const uint8_t cmd_off[6] = {0x01,0x06,0x00,0xC2,0x00,0x40};
const uint8_t cmd_blue_com[6] = {0x01,0x06,0x00,0xC2,0x00,0x45};
const uint8_t cmd_blue_slow[6] = {0x01,0x06,0x00,0xC2,0x00,0x55};
const uint8_t cmd_blue_fast[6] = {0x01,0x06,0x00,0xC2,0x00,0x65};
const uint8_t cmd_cyan_com[6] = {0x01,0x06,0x00,0xC2,0x00,0x46};
const uint8_t cmd_cyan_slow[6] = {0x01,0x06,0x00,0xC2,0x00,0x56};
const uint8_t cmd_cyan_fast[6] = {0x01,0x06,0x00,0xC2,0x00,0x66};
const uint8_t cmd_purple_com[6] = {0x01,0x06,0x00,0xC2,0x00,0x47};
const uint8_t cmd_purple_slow[6] = {0x01,0x06,0x00,0xC2,0x00,0x57};
const uint8_t cmd_purple_fast[6] = {0x01,0x06,0x00,0xC2,0x00,0x67};
//关灯
const uint8_t cmd_off[6] = {0x01,0x06,0x00,0xC2,0x00,0x60};
/*
==============喇叭cmd===================
*/
const uint8_t cmd_sound_3m_warn[6] =  {0x02,0x06,0x00,0x08,0x00,0x07};
const uint8_t cmd_sound_7m_warn[6] =  {0x02,0x06,0x00,0x08,0x00,0x08};
const uint8_t cmd_sound_stop[6] ={0x02,0x06,0x00,0x16,0x00,0x01};
const uint8_t cmd_sound_volumeUp[6] ={0x02,0x06,0x00,0x04,0x00,0x00};
const uint8_t cmd_sound_volumeDown[6] ={0x02,0x06,0x00,0x05,0x00,0x00};
/*
=============bms cmd====================
*/
const uint8_t cmd_read_soc[6] = {0x04,0x03,0x00,0x00,0x00,0x01};//读取SOC百分比命令
const uint8_t cmd_read_full_capacity[6] = {0x04,0x03,0x00,0x04,0x00,0x01};//读取满容容量命令
const uint8_t cmd_read_remain_capacity[6] = {0x04,0x03,0x00,0x04,0x00,0x01};//读取剩余容量命令
const uint8_t cmd_read_total_voltage[6] = {0x04,0x03,0x00,0x02,0x00,0x01};//读取总电压命令
const uint8_t cmd_read_total_current[6] = {0x04,0x03,0x00,0x01,0x00,0x01};//读取总电流命令



// 发送 Modbus RTU 命令到 RS485 总线
void YX95R_RGB_Send_Command(uint8_t *cmd, uint8_t len) {
    
   HAL_UART_Transmit(&huart2, cmd, len, HAL_MAX_DELAY);   
    HAL_Delay(200);

    // 等待发送完成
    // while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET);
}



/**
 * @brief 计算 Modbus RTU CRC16 校验码
 */
uint16_t YX95R_CRC16_Calc(uint8_t *buffer, uint16_t len) {
    uint16_t crc = 0xFFFF;  // CRC 初始值
    
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buffer[i];     // 与当前字节异或
        
        for (uint16_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;    // 右移一位
                crc ^= 0xA001; // Modbus 专用多项式
            } else {
                crc >>= 1;    // 右移一位
            }
        }
    }
    
    return crc;  // 返回最终CRC值
}

//通用 Modbus 寄存器写入函数
 void YX95R_RGB_Write_Register2(uint8_t addr, uint16_t reg, uint16_t data) {
    uint8_t cmd[8];
    uint16_t crc;
    
    cmd[0] = addr;
    cmd[1] = YX95R_FUNC_WRITE_REG;
    cmd[2] = (reg >> 8) & 0xFF;
    cmd[3] = reg & 0xFF;
    cmd[4] = (data >> 8) & 0xFF;
    cmd[5] = data & 0xFF;
    
    crc = YX95R_CRC16_Calc(cmd, 6);
    cmd[6] = crc & 0xFF;
    cmd[7] = (crc >> 8) & 0xFF;
    
    YX95R_RGB_Send_Command(cmd, 8);
}
 void YX95R_RGB_Write_Register(uint8_t *CMD) {
    uint8_t cmd[8];
    uint16_t crc;
    
    cmd[0] = CMD[0];
    cmd[1] = CMD[1];
    cmd[2] = CMD[2];
    cmd[3] = CMD[3];
    cmd[4] = CMD[4];
    cmd[5] = CMD[5];
    
    crc = YX95R_CRC16_Calc(cmd, 6);
    cmd[6] = crc & 0xFF;
    cmd[7] = (crc >> 8) & 0xFF;
    
    YX95R_RGB_Send_Command(cmd, 8);
}
//读取数据
static void YX95R_RGB_Read_Register(uint8_t addr, uint16_t reg, uint16_t data) {
    uint8_t cmd[8];
    uint16_t crc;
    
    cmd[0] = addr;
    cmd[1] = YX95R_FUNC_READ_REG;
    cmd[2] = (reg >> 8) & 0xFF;
    cmd[3] = reg & 0xFF;
    cmd[4] = (data >> 8) & 0xFF;
    cmd[5] = data & 0xFF;
    
    crc = YX95R_CRC16_Calc(cmd, 6);
    cmd[6] = crc & 0xFF;
    cmd[7] = (crc >> 8) & 0xFF;
    
    YX95R_RGB_Send_Command(cmd, 8);
    
}
//write_register
//音量加
void YX95R_RGB_Volume_Up(uint8_t addr) {
    // YX95R_RGB_Write_Register(addr, YX95R_CMD_VOLUME_UP, 0x0000);
}
//音量减   
void YX95R_RGB_Volume_Down(uint8_t addr) {
    // YX95R_RGB_Write_Register(addr, YX95R_CMD_VOLUME_DOWN, 0x0000);
}
//音量设置(0~100)
void YX95R_RGB_Set_Volume(uint8_t addr, uint16_t volume) {
    // 1. 安全限幅：如果输入超过100，强制限制为100
    if (volume > 100) {
        volume = 100;
    }
    if(volume<0){
        volume=0;
    }
    uint16_t hw_volume = (volume * 30) / 100; 
    YX95R_RGB_Write_Register2(addr, YX95R_CMD_SET_VOLUME, hw_volume);
}
//设置设备地址
void YX95R_RGB_Set_Address(uint8_t current_addr, uint16_t new_addr) {
    // YX95R_RGB_Write_Register(current_addr, YX95R_CMD_SET_ADDRESS, new_addr);
}
//芯片复位
void YX95R_RGB_Chip_Reset(uint8_t addr) {
    // YX95R_RGB_Write_Register(addr, YX95R_CMD_CHIP_RESET, 0x0000);
}
//停止播放曲目
void YX95R_RGB_StopMusic(uint8_t addr) {
    // YX95R_RGB_Write_Register(addr, YX95R_CMD_STOPMUSIC, 0x0001);
}
//循环指定歌曲 (1~6-1：警报声，2：，3：救护车，4：消防，5：警车，6：停车场)
//喇叭播放曲目 (7：吊钩3m范围内有人。8：吊钩7m内有人)
void YX95R_RGB_Loop_Song(uint8_t addr, uint16_t song_index) {
    // YX95R_RGB_Write_Register(addr, YX95R_CMD_LOOP_SONG, song_index);
}
//单独控制灯光 (0xC2 命令)
 // XY 组合：X = 闪烁方式 (4=常亮, 5=慢闪, 6=爆闪), Y = 颜色 (1~7红黄绿白蓝青紫, 0=关闭)
void YX95R_RGB_Control_Light(uint8_t addr, uint8_t X, uint8_t Y) {
    uint16_t data = ((uint16_t)X << 4) | Y;
    // YX95R_RGB_Write_Register(addr, YX95R_CMD_LIGHT_CONTROL, data);
}
//关闭灯光
void YX95R_RGB_Light_Off(uint8_t addr) {
    // YX95R_RGB_Write_Register(addr, YX95R_CMD_LIGHT_CONTROL, 0x0060);
}
//查询设备是否在线
uint8_t YX95R_RGB_Is_Online(uint8_t addr) {
    // 发送读取寄存器命令
    YX95R_RGB_Read_Register(addr, 0x003F, 0x0000);
    
    // 这里应该添加接收响应的代码，并根据响应判断设备是否在线
    // 由于本示例中没有实现接收功能，暂时返回1表示在线
    return 1; // 假设设备在线
}

