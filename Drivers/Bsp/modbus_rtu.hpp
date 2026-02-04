#pragma once

#include <stdint.h>

extern "C" {
#include "Modbus.h"
//extern ModbusRtu bms_led_sound_app;
}

class ModbusRtu {
public:
    ModbusRtu();
    ~ModbusRtu() = default;

    // 初始化为主站（RTU/USB/TCP 任选其一）。
    // 对于 RTU：传入 UART 句柄与 RS485 DE 引脚（无 RS485 则 EN_Port 传 NULL，EN_Pin 传 0）。
    // xTypeHW: USART_HW / USART_HW_DMA / USB_CDC_HW / TCP_HW
    bool initMaster(UART_HandleTypeDef *uart,
                    GPIO_TypeDef *enPort,
                    uint16_t enPin,
                    uint16_t timeoutTicks,
                    mb_hardware_t xTypeHW = USART_HW);

    // 异步读取保持寄存器（FC=3）。立即返回；结果通过任务通知送达调用该函数的任务。
    // outRegs 指向的缓冲需在收到通知前保持有效。
    bool asyncReadHolding(uint8_t slaveId,
                          uint16_t startAddr,
                          uint16_t regsCount,
                          uint16_t *outRegs);

    // 异步读取输入寄存器（FC=4）。
    bool asyncReadInput(uint8_t slaveId,
                        uint16_t startAddr,
                        uint16_t regsCount,
                        uint16_t *outRegs);

    // 异步写单保持寄存器（FC=6）。
    bool asyncWriteSingle(uint8_t slaveId,
                          uint16_t addr,
                          uint16_t value);

    // 异步写多保持寄存器（FC=16）。values 数组长度为 regsCount。
    bool asyncWriteMultiple(uint8_t slaveId,
                            uint16_t startAddr,
                            uint16_t regsCount,
                            const uint16_t *values);

protected:
    modbusHandler_t handler_{};
    bool started_{false};
};



