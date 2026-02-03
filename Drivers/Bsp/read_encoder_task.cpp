#include "read_encoder_task.h"
#include "encoder_forward_app.h"

#define RET_D(...) // printf(__VA_ARGS__)
#define RET_I(...) printf(__VA_ARGS__)
#define RET_E(...) printf(__VA_ARGS__)

#define ENABLE_COMM_TESTING

#ifdef ENABLE_HUB_MASTER

extern UART_HandleTypeDef huart7; 

// Master全局变量
static modbusHandler_t encoder_forward_master;
static uint16_t master_registers[REGS_TOTAL_NUM] = {0};
static uint32_t encoder_slave_data[9] = {0};

osThreadId_t ReadEncoderForwardTaskHandle;
const osThreadAttr_t ReadEncoderForwardTask_attributes = {
  .name = "ReadEncoderForwardTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

#ifdef ENABLE_COMM_TESTING
#include <stdlib.h>   // qsort
#include <limits.h>   // UINT32_MAX

static int cmp_u32_asc(const void *a, const void *b) {
        uint32_t va = *(const uint32_t*)a;
        uint32_t vb = *(const uint32_t*)b;
        return (va > vb) - (va < vb);
}
#endif

// 初始化Master
void init_encoder_forward_master(void)
{
    // 初始化Master寄存器数组
    memset(master_registers, 0, sizeof(master_registers));
    memset(encoder_slave_data, 0, sizeof(encoder_slave_data));
    
    // 配置MODBUS Master处理器
    encoder_forward_master.uModbusType = MB_MASTER;
    encoder_forward_master.u8id = 0;  // Master ID = 0
    encoder_forward_master.port = &huart7;
    encoder_forward_master.EN_Port = NULL;  // 无RS485控制引脚
    encoder_forward_master.EN_Pin = 0;
    encoder_forward_master.u16regs = master_registers;
    encoder_forward_master.u16regsize = REGS_TOTAL_NUM;
    encoder_forward_master.u16timeOut = 500;   // 500ms 超时，减少长时间阻塞
    encoder_forward_master.xTypeHW = USART_HW;
    
    // 初始化MODBUS Master
    ModbusInit(&encoder_forward_master);
    ModbusStart(&encoder_forward_master);

    RET_I("MODBUS-RTU Master\r\n");
}

// 从Slave读取所有传感器数据 (异步方式)
static bool read_all_sensors_non_block_non_block(uint8_t slave_addr)
{
    modbus_t telegram;
    
    // 构建读取所有寄存器的请求
    telegram.u8id = slave_addr;           // Slave ID
    telegram.u8fct = MB_FC_READ_REGISTERS;  // 读保持寄存器
    telegram.u16RegAdd = 0x0000; // 起始地址
    telegram.u16CoilsNo = REGS_TOTAL_NUM; // 读取8个寄存器
    telegram.u16reg = master_registers; // 数据缓冲区
    
    // 发送异步查询
    ModbusQuery(&encoder_forward_master, telegram);
    
    // 返回true表示查询已发送
    return true;
}

// 基于同步阻塞读取的帮助函数：读取指定从站的全部寄存器
uint32_t read_all_encoder_block(uint8_t slave_addr)
{
    modbus_t telegram;

    telegram.u8id = slave_addr;                 // 从站地址
    telegram.u8fct = MB_FC_READ_REGISTERS;      // 读保持寄存器
    telegram.u16RegAdd = 0x0000;                // 起始地址
    telegram.u16CoilsNo = REGS_TOTAL_NUM;   // 读取寄存器数量
    telegram.u16reg = master_registers;         // 存放结果的缓冲区

    // 发送并同步等待返回
    uint32_t result = ModbusQueryV2(&encoder_forward_master, telegram);
    if (result != OP_OK_QUERY) {
        return result;
    }

    // 解析3个传感器的32位位置（高16+低16）
    for (int i = 0; i < 3; i++) {
        uint16_t base_reg = i * 2; // 每个传感器2个寄存器
        encoder_slave_data[i] = ((uint32_t)master_registers[base_reg] << 16) | master_registers[base_reg + 1];
    }
    return result;
}

// Master任务 - 定期从Slave读取传感器数据 (参考编码器任务实现)
void StartNonBlockReadEncoderForwardTask(void *argument)
{    
    // 启用离线事件监控
    // offline_event_enable(OFFLINE_ENCODER_FORWARD);
    
    bool pending = false;
    uint32_t notifyValue = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    static uint8_t cur_addr = 0x00;
    
    for(;;)
    {
        // 发送异步查询请求
        if (!pending)
        {
        	cur_addr = (cur_addr+1)%3; // 0 1 2
            if (read_all_sensors_non_block_non_block(cur_addr + 1))
            {
                pending = true;
            }
        }
        
        // 处理查询响应
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notifyValue, 0) == pdTRUE)
        {
            pending = false;
            if (notifyValue == OP_OK_QUERY)
            {
                // 成功读取，解析数据
                for (int i = 0; i < 3; i++) {
                    uint16_t base_reg = i * 2; // 每个传感器占用2个寄存器
                    encoder_slave_data[i] = (master_registers[base_reg] << 16) | master_registers[base_reg + 1];
                }
                
                // 统计 1s内读取到的次数
                static uint32_t count = 0;
                static uint32_t last_time = 0;
                if (last_time == 0)
                {
                    last_time = get_time_ms();
                }
                if (get_time_ms() - last_time >= 1000)
                {
                    RET_D("0x%X: %ldHz,", cur_addr+1,count);
                    for (int i = 0; i < 3; i++) {
                    RET_D("(%d,0x%08X),", i+1, (unsigned int)encoder_slave_data[i]);
                    }
                    RET_D("\r\n");
                    count = 0;
                    last_time = get_time_ms();
                }
                count++;
            }
            else
            {
                RET_E("Failed to read sensors from slave: %lu\r\n", notifyValue);
            }
        }

        // 更新离线事件时间
        // offline_event_time_update(OFFLINE_ENCODER_FORWARD);
        
        // 等待下一个周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 同步阻塞方式按顺序读取3个从站（1->2->3）
void StartBlockReadEncoderForwardTask(void *argument)
{
    // 启用离线事件监控
    // offline_event_enable(OFFLINE_ENCODER_FORWARD);
    
    // task parameters
    const TickType_t xFrequency = pdMS_TO_TICKS(200);    // 4Hz = 250ms
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // 当前轮询地址，初始化为配置的基地址（默认0x01）
    uint8_t cur_slave = ENCODER_SLAVE_ADDR_BASE;

    
#ifdef ENABLE_COMM_TESTING
    // params for frequency statistics
    const uint32_t WINDOW_MS = 1UL * 60UL * 1000UL;     // 30 min
    uint32_t window_start_ms = get_time_ms();
    uint32_t total_frames = 0;
    uint32_t bad_frames   = 0;
    uint32_t cnt_crc_err  = 0;
    uint32_t cnt_len_err  = 0;
    uint32_t cnt_to_err   = 0;

    // 单次交易时延/抖动统计（仅成功交易样本）
    static const size_t LAT_MAX_SAMPLES = 9000; // 30min@4Hz≈7200，预留余量
    static uint32_t lat_samples[LAT_MAX_SAMPLES];
    size_t lat_count = 0;
    uint32_t lat_min = UINT32_MAX;
    uint32_t lat_max = 0;
    uint64_t lat_sum = 0;      // 累计和（用于均值）
    uint64_t lat_sum_sq = 0;   // 累计平方和（用于方差）
#endif

    // 计算轮询范围：[first, last]
    uint8_t first = ENCODER_SLAVE_ADDR_BASE;
    uint8_t last  = (uint8_t)(ENCODER_SLAVE_ADDR_BASE + ENCODER_SLAVE_COUNT - 1);
    if (ENCODER_SLAVE_COUNT == 0) {
        // 防御：若配置为0，保持默认1个地址
        first = 0x01; last = 0x01;
    }
    // 确保当前地址在有效范围
    if (cur_slave < first || cur_slave > last) {
        cur_slave = first;
    }

    for(;;)
    {
        uint32_t now_ms = get_time_ms();

        // 发送并同步等待返回，并记录单次交易耗时（请求->应答完成）
#ifdef ENABLE_COMM_TESTING
        uint32_t t0 = get_time_ms();
#endif
        uint32_t result = read_all_encoder_block(cur_slave);
        // 本次读取后推进到下一个地址
        cur_slave = (cur_slave == last) ? first : (uint8_t)(cur_slave + 1);

#ifdef ENABLE_COMM_TESTING
        uint32_t t1 = get_time_ms();
        uint32_t dt_ms = t1 - t0;
        // 更新统计
        total_frames++;
        if (result != OP_OK_QUERY) {
            RET_D("[%lu](0x%X,0x%lX)",now_ms,cur_slave, result);
            // 仅统计三类坏帧：CRC 错/长度错/应答超时
            if (result == ERR_BAD_CRC) {
                cnt_crc_err++;
                bad_frames++;
            } else if (result == ERR_BAD_SIZE) {
                cnt_len_err++;
                bad_frames++;
            } else if (result == ERR_TIME_OUT) {
                cnt_to_err++;
                bad_frames++;
            }
            else{
                bad_frames++;
            }
        }
        else {
            if (lat_count < LAT_MAX_SAMPLES) {
                lat_samples[lat_count++] = dt_ms;
            }
            // 更新 min/max/和/平方和（整数，不依赖浮点）
            if (dt_ms < lat_min) lat_min = dt_ms;
            if (dt_ms > lat_max) lat_max = dt_ms;
            lat_sum += dt_ms;
            lat_sum_sq += (uint64_t)dt_ms * (uint64_t)dt_ms;
        }

        // 窗口到期 -> 打印摘要并复位计数
        if (now_ms - window_start_ms >= WINDOW_MS) {
            // 以千分之一百分比精度输出：FER = X.xxx%
            uint32_t fer_milli_pct = 0; // 单位：0.001%
            if (total_frames > 0) {
                fer_milli_pct = (uint32_t)((bad_frames * 1000UL + (total_frames / 2UL)) / total_frames);
            }
            // 验收标准：FER ≤ 0.1% <=> bad_frames * 1000 ≤ total_frames
            const bool pass = (total_frames > 0) ? (bad_frames * 1000UL <= total_frames) : true;

            RET_I("\r\n");
            RET_I("\r\n=== [%lu] ===\r\n",now_ms);
            // 平均帧率（窗口内平均）：总帧率与成功帧率（1 位小数）
            uint32_t elapsed_ms = now_ms - window_start_ms;
            uint32_t ok_frames = (total_frames >= bad_frames) ? (total_frames - bad_frames) : 0U;
            uint32_t fps_total_x10 = (elapsed_ms > 0U) ? ( (total_frames * 10000UL) / elapsed_ms ) : 0U; // ×10 表示 1 位小数
            uint32_t fps_success_x10    = (elapsed_ms > 0U) ? ( (ok_frames   * 10000UL) / elapsed_ms ) : 0U;
            RET_I("[RATE]avg= %lu.%01lu Hz, success= %lu.%01lu Hz\r\n",
                (unsigned long)(fps_total_x10 / 10UL), (unsigned long)(fps_total_x10 % 10UL),
                (unsigned long)(fps_success_x10 / 10UL),    (unsigned long)(fps_success_x10 % 10UL));
            RET_I("=========================================\r\n");
            RET_I("[FER]Total=%lu, Bad=%lu [CRC=%lu, LEN=%lu, TIMEOUT=%lu]\r\n",
                   total_frames, bad_frames, cnt_crc_err, cnt_len_err, cnt_to_err);
            RET_I("FER=%lu.%03lu%%  => %s\r\n",
                   (unsigned long)(fer_milli_pct / 10UL),
                   (unsigned long)(fer_milli_pct % 10UL) * 100UL,   // 将 0.1% 千分之一转为三位小数
                   pass ? "PASS (≤ 0.1%)" : "FAIL (> 0.1%)");
            RET_I("=========================================\r\n");
            // 计算时延分位数与方差（仅成功样本）
            if (lat_count > 0) {
                // 原地排序
                qsort(lat_samples, lat_count, sizeof(uint32_t), cmp_u32_asc);
                size_t idx50 = (size_t)((0.50 * (lat_count - 1)) + 0.5);
                size_t idx95 = (size_t)((0.95 * (lat_count - 1)) + 0.5);
                size_t idx99 = (size_t)((0.99 * (lat_count - 1)) + 0.5);
                if (idx50 >= lat_count) idx50 = lat_count - 1;
                if (idx95 >= lat_count) idx95 = lat_count - 1;
                if (idx99 >= lat_count) idx99 = lat_count - 1;

                uint32_t p50 = lat_samples[idx50];
                uint32_t p95 = lat_samples[idx95];
                uint32_t p99 = lat_samples[idx99];

                // 整数均值与方差（样本方差）
                uint32_t mean_ms = (uint32_t)(lat_sum / (uint64_t)lat_count);
                uint32_t var_ms2 = 0;
                if (lat_count > 1) {
                    // var = (sum_sq - sum^2/n)/(n-1)
                    uint64_t num = lat_sum_sq - (lat_sum * lat_sum) / (uint64_t)lat_count;
                    var_ms2 = (uint32_t)(num / (uint64_t)(lat_count - 1));
                }

                // 验收：P95 ≤ 1.5 × 预计基准（此处用 P50 作为动态基准）
                uint32_t threshold = p50 + (p50 >> 1); // 1.5x
                bool latency_pass = (p95 <= threshold);

                RET_I("[LAT]samples=%lu, min=%lums, max=%lums, mean=%lums, var=%lums^2\r\n",
                       (unsigned long)lat_count,
                       (unsigned long)lat_min, (unsigned long)lat_max,
                       (unsigned long)mean_ms, (unsigned long)var_ms2);
                RET_I("P50=%lums, P95=%lums, P99=%lums => %s (P95 ≤ 1.5×P50)\r\n",
                       (unsigned long)p50, (unsigned long)p95, (unsigned long)p99,
                       latency_pass ? "PASS" : "FAIL");
                RET_I("=========================================\r\n");
            } else {
                RET_I("LAT: no successful samples in window.\r\n");
                RET_I("=========================================\r\n");
            }

            // 复位窗口
            total_frames = bad_frames = 0;
            cnt_crc_err = cnt_len_err = cnt_to_err = 0;
            window_start_ms = now_ms;

            // 重置时延统计
            lat_count = 0;
            lat_min = UINT32_MAX;
            lat_max = 0;
            lat_sum = 0;
            lat_sum_sq = 0;
        }
#endif
        // 固定节拍：4Hz
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#endif

#ifdef ENABLE_HUB_SLAVE
#include "read_encoder_task.h"
#include "oid_encoder.hpp"

extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart2; 
extern UART_HandleTypeDef huart3; 
extern UART_HandleTypeDef huart8; 

// Slave全局变量
static modbusHandler_t encoder_forward_server;
static modbusHandler_t encoder_forward_server_backup;
static uint16_t modbus_registers[REGS_TOTAL_NUM] = {0};

static OidEncoder g_encoder1(1, 1, &huart2); // 用于 huart2
static OidEncoder g_encoder2(2, 1, &huart3); // 用于 huart3
static OidEncoder g_encoder3(3, 1, &huart8); // 用于 huart8

osThreadId_t ReadEncoderForwardTaskHandle;
const osThreadAttr_t ReadEncoderForwardTask_attributes = {
  .name = "ReadEncoderForwardTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t ReadEncoderTask1Handle;
const osThreadAttr_t ReadEncoderTask1_attributes = {
  .name = "ReadEncoderTask1",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};

osThreadId_t ReadEncoderTask2Handle;
const osThreadAttr_t ReadEncoderTask2_attributes = {
  .name = "ReadEncoderTask2",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};

osThreadId_t ReadEncoderTask3Handle;
const osThreadAttr_t ReadEncoderTask3_attributes = {
  .name = "ReadEncoderTask3",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};

void init_encoder_forward_slave(void)
{
    // 初始化寄存器数组
    memset(modbus_registers, 0, sizeof(modbus_registers));
    
    // 设置系统信息
    modbus_registers[REG_ERROR_CODE] = 0x0000; // 清零错误码/在线状态
    modbus_registers[REG_TOTAL_SENSORS] = 3;
    modbus_registers[REG_SYSTEM_VERSION] = HUB_SLAVE_VERSION; // 记录系统版本号
    
    // 配置MODBUS Slave处理器
    encoder_forward_server.uModbusType = MB_SLAVE;
    encoder_forward_server.u8id = FORWARD_SLAVE_ADDR;  // Slave ID
    encoder_forward_server.port = &huart7;
    encoder_forward_server.EN_Port = NULL;  // 无RS485控制引脚
    encoder_forward_server.EN_Pin = 0;
    encoder_forward_server.u16regs = modbus_registers;
    encoder_forward_server.u16regsize = REGS_TOTAL_NUM;
    encoder_forward_server.u16timeOut = 1000;  // 1秒超时
    encoder_forward_server.xTypeHW = USART_HW;
    RET_I("MODBUS-RTU Slave ID: %d\r\n",FORWARD_SLAVE_ADDR);
    
    // 初始化MODBUS Slave
    ModbusInit(&encoder_forward_server);
    ModbusStart(&encoder_forward_server);

    encoder_forward_server_backup = encoder_forward_server; // 拷贝参数
    encoder_forward_server_backup.port = &huart8;
    ModbusInit(&encoder_forward_server_backup);
    ModbusStart(&encoder_forward_server_backup);
    RET_I("MODBUS-RTU Slave ID: %d on huart8\r\n", FORWARD_SLAVE_ADDR);


    RET_I("Register range: 0x0000-0x%04X (%d registers)\r\n", 
           REGS_TOTAL_NUM-1, REGS_TOTAL_NUM);
}

// 更新传感器数据
void encoder_update_data(uint8_t id,const uint32_t* data,bool state)
{
    if (id < 1 || id > 3 || data == NULL) {
        return;
    }
    uint8_t index = id - 1;  // 转换为数组索引
    uint16_t base_reg;

    switch (id) {
        case 1: base_reg = REG_SENSOR1_POS_HI; break;
        case 2: base_reg = REG_SENSOR2_POS_HI; break;
        case 3: base_reg = REG_SENSOR3_POS_HI; break;
        default: return;
    }
    // 更新MODBUS寄存器 - 只存储位置数据 (32位分为两个16位寄存器)
    modbus_registers[base_reg] = (uint16_t)(*data >> 16);      // 高16位
    modbus_registers[base_reg + 1] = (uint16_t)(*data & 0xFFFF); // 低16位
    if(state){
        modbus_registers[REG_ERROR_CODE] |= (1U << (id - 1));
    }else{
        modbus_registers[REG_ERROR_CODE] &= ~(1U << (id - 1));
    }

    RET_D("(E%d,%lu)", id, (unsigned long)*data);
}

void encoder_update_multi(const uint8_t id,const uint16_t count,const uint16_t value,const bool state)
{
    if (id < 1 || id > 3) {
        return;
    }

    uint8_t index = id - 1;  // 转换为数组索引
    uint16_t base_reg;

    // 根据传感器ID确定寄存器基地址
    switch (id) {
        case 1: base_reg = REG_SENSOR1_POS_HI; break;
        case 2: base_reg = REG_SENSOR2_POS_HI; break;
        case 3: base_reg = REG_SENSOR3_POS_HI; break;
        default: return;
    }

    modbus_registers[base_reg] = count;
    modbus_registers[base_reg + 1] = value;
    
    if(state){
        modbus_registers[REG_ERROR_CODE] |= (1U << (id - 1));
    }else{
        modbus_registers[REG_ERROR_CODE] &= ~(1U << (id - 1));
    }

    RET_D("(E%d,%d,%d)", id, count,value);
}

void StartReadEncoderTask(void *argument)
{
    OidEncoder* encoder = static_cast<OidEncoder*>(argument);
    if (!encoder) {
        return;
    }

    encoder->init_();

    bool pending = false;
    uint32_t notifyValue = 0;
    TickType_t xLastWakeTime = 0;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz = 100ms

    for(;;)
    {

#ifdef USE_FAKE_ENCODER
                uint32_t posRaw = 0x00000000 + encoder->getDeviceId();
                encoder_update_multi(encoder->getDeviceId(), (uint16_t)(posRaw >> 16), (uint16_t)(posRaw & 0xFFFF), true);
                RET_D("Encoder%d position: %ld\n", encoder->getDeviceId(), posRaw);
#else
        if (!pending)
        {
            if (encoder->requestMultiAsync())
            {
                pending = true;
            }
        }

        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notifyValue, 0) == pdTRUE)
        {
            pending = false;
            if (notifyValue == OP_OK_QUERY)
            {
                uint32_t pos = encoder->getLastPositionRaw();
                encoder_update_multi(encoder->getDeviceId(), (uint16_t)(pos >> 16), (uint16_t)(pos & 0xFFFF), true);
            }
            else
            {
                RET_E("Encoder%d read error: %ld\n", encoder->getDeviceId(), notifyValue);
                uint32_t pos = 0;
                encoder_update_multi(encoder->getDeviceId(), 0, 0, false);        
            }
        }
#endif  
        vTaskDelay(200);
    }
}

#endif

void init_read_encoder_task()
{

#ifdef ENABLE_HUB_MASTER
    // 初始化MODBUS-RTU Master
    init_encoder_forward_master();
    // 创建Master任务
    ReadEncoderForwardTaskHandle = osThreadNew(StartBlockReadEncoderForwardTask, NULL, &ReadEncoderForwardTask_attributes);
    RET_I("type = HUB_MASTER; version = %d\r\n", HUB_MASTER_VERSION);
#endif

#ifdef ENABLE_HUB_SLAVE
    // 初始化MODBUS-RTU Slave
    init_encoder_forward_slave();

    // 创建编码器读取任务
    ReadEncoderTask1Handle = osThreadNew(StartReadEncoderTask, &g_encoder1, &ReadEncoderTask1_attributes);
    ReadEncoderTask2Handle = osThreadNew(StartReadEncoderTask, &g_encoder2, &ReadEncoderTask2_attributes);
    // ReadEncoderTask3Handle = osThreadNew(StartReadEncoderTask, &g_encoder3, &ReadEncoderTask3_attributes);
    RET_I("device type = HUB_SLAVE; version = %d\r\n", HUB_SLAVE_VERSION);
#endif
    
}


