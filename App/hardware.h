#ifndef HARDWARE_H
#define HARDWARE_H
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <string.h>
#include "Drivers/Bsp/modbus_rtu.hpp"
#include "Drivers/Bsp/read_encoder_task.h"
#include "Drivers/Bsp/oid_encoder.hpp"

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif


// #include "nex_modbus_rtu_client.h"
/* ============================= 灯控制命令定义 ============================= */

// Modbus 功能码
#define YX95R_FUNC_WRITE_REG 0x06 // 写单个寄存器
#define YX95R_FUNC_READ_REG 0x03  // 读单个寄存器

#define YX95R_CMD_VOLUME_UP 0x0004   // 音量增大
#define YX95R_CMD_VOLUME_DOWN 0x0005 // 音量减小
#define YX95R_CMD_SET_VOLUME 0x0006  // 设置音量

#define YX95R_CMD_SET_ADDRESS 0x00C0 // 设置设备地址

#define YX95R_CMD_CHIP_RESET 0x000C // 芯片复位

#define YX95R_CMD_STOPMUSIC 0x0016 // 停止播放
#define YX95R_CMD_LOOP_SONG 0x0008 // 循环指定歌曲

#define YX95R_CMD_LIGHT_CONTROL 0x00C2 // 灯控制命令寄存器

// extern ModbusRtu bms_led_sound_app;

/* ============================= 灯命令数据定义 ============================= */
// 灯地址：0x01
/* 常亮 COM：X=4，Y=1~7 */
extern const uint8_t cmd_red_com[6];
extern const uint8_t cmd_red_slow[6];
extern const uint8_t cmd_red_fast[6];
extern const uint8_t cmd_yellow_com[6];
extern const uint8_t cmd_yellow_slow[6];
extern const uint8_t cmd_yellow_fast[6];
extern const uint8_t cmd_green_com[6];
extern const uint8_t cmd_green_slow[6];
extern const uint8_t cmd_green_fast[6];
extern const uint8_t cmd_white_com[6];
extern const uint8_t cmd_white_slow[6];
extern const uint8_t cmd_white_fast[6];
extern const uint8_t cmd_blue_com[6];
extern const uint8_t cmd_blue_slow[6];
extern const uint8_t cmd_blue_fast[6];
extern const uint8_t cmd_cyan_com[6];
extern const uint8_t cmd_cyan_slow[6];
extern const uint8_t cmd_cyan_fast[6];
extern const uint8_t cmd_purple_com[6];
extern const uint8_t cmd_purple_slow[6];
extern const uint8_t cmd_purple_fast[6];
// 关闭所有灯：X=7，Y=0
extern const uint8_t cmd_off[6];

/* ============================= 喇叭命令定义 ============================= */
// 喇叭地址：0x02
/* 喇叭循环指定歌曲 (7：吊钩3m范围内有人, 8：吊钩7m内有人) */
extern const uint8_t cmd_sound_3m_warn[6];
extern const uint8_t cmd_sound_7m_warn[6];
extern const uint8_t cmd_sound_stop[6];
// extern const uint8_t cmd_sound_setVolume[6] ;
extern const uint8_t cmd_sound_volumeUp[6];
extern const uint8_t cmd_sound_volumeDown[6];

/*===============================bms命令定义================================*/
extern const uint8_t cmd_read_soc[6];// 读取电池剩余电量百分比
extern const uint8_t cmd_read_full_capacity[6];// 读取电池总容量
extern const uint8_t cmd_read_remain_capacity[6];// 读取电池剩余容量
extern const uint8_t cmd_read_total_voltage[6];// 读取电池总电压
extern const uint8_t cmd_read_total_current[6];// 读取电池总电流
/* ============================= 灯的函数宏声明 ============================= */
#define YX95R_LIGHT_ON_RED_COM YX95R_RGB_Write_Register((uint8_t *)cmd_red_com)         // 红色常亮
#define YX95R_LIGHT_ON_RED_SLOW YX95R_RGB_Write_Register((uint8_t *)cmd_red_slow)       // 红色慢闪
#define YX95R_LIGHT_ON_RED_FAST YX95R_RGB_Write_Register((uint8_t *)cmd_red_fast)       // 红色快闪
#define YX95R_LIGHT_ON_YELLOW_COM YX95R_RGB_Write_Register((uint8_t *)cmd_yellow_com)   // 黄色常亮
#define YX95R_LIGHT_ON_YELLOW_SLOW YX95R_RGB_Write_Register((uint8_t *)cmd_yellow_slow) // 黄色慢闪
#define YX95R_LIGHT_ON_YELLOW_FAST YX95R_RGB_Write_Register((uint8_t *)cmd_yellow_fast) // 黄色快闪
#define YX95R_LIGHT_ON_GREEN_COM YX95R_RGB_Write_Register((uint8_t *)cmd_green_com)     // 绿色常亮
#define YX95R_LIGHT_ON_GREEN_SLOW YX95R_RGB_Write_Register((uint8_t *)cmd_green_slow)   // 绿色慢闪
#define YX95R_LIGHT_ON_GREEN_FAST YX95R_RGB_Write_Register((uint8_t *)cmd_green_fast)   // 绿色快闪
#define YX95R_LIGHT_ON_WHITE_COM YX95R_RGB_Write_Register((uint8_t *)cmd_white_com)     // 白色常亮
#define YX95R_LIGHT_ON_WHITE_SLOW YX95R_RGB_Write_Register((uint8_t *)cmd_white_slow)   // 白色慢闪
#define YX95R_LIGHT_ON_WHITE_FAST YX95R_RGB_Write_Register((uint8_t *)cmd_white_fast)   // 白色快闪
#define YX95R_LIGHT_ON_BLUE_COM YX95R_RGB_Write_Register((uint8_t *)cmd_blue_com)       // 蓝色常亮
#define YX95R_LIGHT_ON_BLUE_SLOW YX95R_RGB_Write_Register((uint8_t *)cmd_blue_slow)     // 蓝色慢闪
#define YX95R_LIGHT_ON_BLUE_FAST YX95R_RGB_Write_Register((uint8_t *)cmd_blue_fast)     // 蓝色快闪
#define YX95R_LIGHT_ON_CYAN_COM YX95R_RGB_Write_Register((uint8_t *)cmd_cyan_com)       // 青色常亮
#define YX95R_LIGHT_ON_CYAN_SLOW YX95R_RGB_Write_Register((uint8_t *)cmd_cyan_slow)     // 青色慢闪
#define YX95R_LIGHT_ON_CYAN_FAST YX95R_RGB_Write_Register((uint8_t *)cmd_cyan_fast)     // 青色快闪
#define YX95R_LIGHT_ON_PURPLE_COM YX95R_RGB_Write_Register((uint8_t *)cmd_purple_com)
#define YX95R_LIGHT_ON_PURPLE_SLOW YX95R_RGB_Write_Register((uint8_t *)cmd_purple_slow)
#define YX95R_LIGHT_ON_PURPLE_FAST YX95R_RGB_Write_Register((uint8_t *)cmd_purple_fast)
/* 关闭所有灯：X=7，Y=0 */
#define YX95R_LIGHT_OFF YX95R_RGB_Write_Register((uint8_t *)cmd_off)

/* ============================= 喇叭的函数宏声明 ============================= */
// 喇叭地址：0x02
//  #define SOUND_3M_WARN YX95R_RGB_Loop_Song(0x02, 7); // 喇叭循环播放曲目 (7：吊钩3m范围内有人)
//  #define SOUND_7M_WARN YX95R_RGB_Loop_Song(0x02, 8); // 喇叭循环播放曲目 (8：吊钩7m范围内有人)
//  #define SOUND_STOP YX95R_RGB_StopMusic(0x02); // 停止喇叭播放
#define SOUND_SET_VOLUME(x) YX95R_RGB_Set_Volume(0x02, x); // 设置喇叭音量 (0~100)
// #define SOUND_VOLUME_UP YX95R_RGB_Volume_Up(0x02); // 喇叭音量加
// #define SOUND_VOLUME_DOWN YX95R_RGB_Volume_Down(0x02); // 喇叭音量减
#define BUZZER_SOUND_3M_WARN YX95R_RGB_Write_Register((uint8_t *)cmd_sound_3m_warn) // 喇叭循环播放曲目 (7：吊钩3m范围内有人)
#define BUZZER_SOUND_7M_WARN YX95R_RGB_Write_Register((uint8_t *)cmd_sound_7m_warn) // 喇叭循环播放曲目 (8：吊钩7m范围内有人)
#define BUZZER_SOUND_STOP YX95R_RGB_Write_Register((uint8_t *)cmd_sound_stop)       // 停止喇叭播放
// #define YX95R_SOUND_SET_VOLUME(x)  YX95R_RGB_Write_Register ((uint8_t*)cmd_sound_setVolume)//设置喇叭音量 (0~100)
#define BUZZER_SOUND_VOLUME_UP YX95R_RGB_Write_Register((uint8_t *)cmd_sound_volumeUp)     // 喇叭音量加
#define BUZZER_SOUND_VOLUME_DOWN YX95R_RGB_Write_Register((uint8_t *)cmd_sound_volumeDown) // 喇叭音量减

/*================================= bms宏函数声明=============================*/
#define BMS_READ_SOC bms_led_sound_app.() // 读取电池剩余电量百分比
#define BMS_READ_FULL_CAP YX95R_RGB_Write_Register((uint8_t *)cmd_read_full_capacity) // 读取电池总容量
#define BMS_READ_REMAIN_CAP YX95R_RGB_Write_Register((uint8_t *)cmd_read_remain_capacity) // 读取电池剩余容量
#define BMS_READ_TOTAL_V YX95R_RGB_Write_Register((uint8_t *)cmd_read_total_voltage) // 读取电池总电压
#define BMS_READ_TOTAL_A YX95R_RGB_Write_Register((uint8_t *)cmd_read_total_current) // 读取电池总电流



/* ===== BMS数据结构 ===== */
typedef struct {
    uint16_t soc;           /* 电量百分比 (0-100) */
    uint16_t voltage;       /* 电压 (mV) */
    int16_t current;        /* 电流 (mA，负值表示放电) */
    uint16_t remaining_cap; /* 剩余容量 (0.1Ah) */
    uint16_t error_code;    /* 错误码 */
} BMS_Data_t;


/**
 * @brief 计算 Modbus CRC16 校验码
 * @param buffer: 数据缓冲区指针
 * @param len: 数据长度
 * @return uint16_t: CRC16 值
 */
uint16_t YX95R_CRC16_Calc(uint8_t *buffer, uint16_t len);

void YX95R_RGB_Write_Register2(uint8_t addr, uint16_t reg, uint16_t data);

void YX95R_RGB_Write_Register(uint8_t *CMD);

// void YX95R_RGB_Write_Register(uint8_t *CMD);

/**
 * @brief 通过 RS485 发送 Modbus 命令包
 * @param cmd: 命令数据指针
 * @param len: 命令长度
 * @return void
 *
 * 注意：
 *  1. 自动控制 RE/DE 脚切换收发模式
 *  2. 等待发送完成才会返回
 *  3. 发送前自动算 CRC 并填充
 */
void YX95R_RGB_Send_Command(uint8_t *cmd, uint8_t len);

// 通用 Modbus 寄存器写入函数
//  static void YX95R_RGB_Write_Register(uint8_t addr, uint16_t reg, uint16_t data);

void YX95R_RGB_Volume_Up(uint8_t addr);

void YX95R_RGB_Volume_Down(uint8_t addr);

void YX95R_RGB_Set_Volume(uint8_t addr, uint16_t volume);

void YX95R_RGB_Set_Address(uint8_t current_addr, uint16_t new_addr);

void YX95R_RGB_Chip_Reset(uint8_t addr);

void YX95R_RGB_StopMusic(uint8_t addr);

void YX95R_RGB_Loop_Song(uint8_t addr, uint16_t song_index);

void YX95R_RGB_Control_Light(uint8_t addr, uint8_t X, uint8_t Y);

void YX95R_RGB_Light_Off(uint8_t addr);

uint8_t YX95R_RGB_Is_Online(uint8_t addr);

#endif // HARDWARE_H
