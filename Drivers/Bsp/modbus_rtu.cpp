#include "modbus_rtu.hpp"

ModbusRtu::ModbusRtu()
{
    // zero init handler_
}

bool ModbusRtu::initMaster(UART_HandleTypeDef *uart,
                           GPIO_TypeDef *enPort,
                           uint16_t enPin,
                           uint16_t timeoutTicks,
                           mb_hardware_t xTypeHW)
{
    if (started_) return true;

    handler_.uModbusType = MB_MASTER;
    handler_.port = uart;
    handler_.u8id = 0; // Master 必须为 0
    handler_.EN_Port = enPort;
    handler_.EN_Pin = enPin;
    handler_.u16timeOut = timeoutTicks;
    handler_.xTypeHW = xTypeHW;

    ModbusInit(&handler_);
    if (xTypeHW == USB_CDC_HW) {
#if ENABLE_USB_CDC == 1
        ModbusStartCDC(&handler_);
#else
        return false;
#endif
    } else {
        ModbusStart(&handler_);
    }

    started_ = true;
    return true;
}

bool ModbusRtu::asyncReadHolding(uint8_t slaveId,
                                 uint16_t startAddr,
                                 uint16_t regsCount,
                                 uint16_t *outRegs)
{
    if (!started_ || outRegs == nullptr) return false;

    modbus_t telegram{};
    telegram.u8id = slaveId;
    telegram.u8fct = MB_FC_READ_REGISTERS;
    telegram.u16RegAdd = startAddr;
    telegram.u16CoilsNo = regsCount;
    telegram.u16reg = outRegs;

    ModbusQuery(&handler_, telegram);
    return true;
}

bool ModbusRtu::asyncReadInput(uint8_t slaveId,
                               uint16_t startAddr,
                               uint16_t regsCount,
                               uint16_t *outRegs)
{
    if (!started_ || outRegs == nullptr) return false;

    modbus_t telegram{};
    telegram.u8id = slaveId;
    telegram.u8fct = MB_FC_READ_INPUT_REGISTER;
    telegram.u16RegAdd = startAddr;
    telegram.u16CoilsNo = regsCount;
    telegram.u16reg = outRegs;

    ModbusQuery(&handler_, telegram);
    return true;
}

bool ModbusRtu::asyncWriteSingle(uint8_t slaveId,
                                 uint16_t addr,
                                 uint16_t value)
{
    if (!started_) return false;

    uint16_t tmp = value; // telegram 持有指针，需确保生命周期
    modbus_t telegram{};
    telegram.u8id = slaveId;
    telegram.u8fct = MB_FC_WRITE_REGISTER;
    telegram.u16RegAdd = addr;
    telegram.u16CoilsNo = 1;
    telegram.u16reg = &tmp;

    ModbusQuery(&handler_, telegram);
    return true;
}



bool ModbusRtu::asyncWriteMultiple(uint8_t slaveId,
                                   uint16_t startAddr,
                                   uint16_t regsCount,
                                   const uint16_t *values)
{
    if (!started_ || values == nullptr) return false;

    // 注意：ModbusQuery 在队列中异步使用 u16reg 指针，调用方必须保证 values 在通知返回前保持有效。
    modbus_t telegram{};
    telegram.u8id = slaveId;
    telegram.u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS;
    telegram.u16RegAdd = startAddr;
    telegram.u16CoilsNo = regsCount;
    telegram.u16reg = const_cast<uint16_t *>(values);

    ModbusQuery(&handler_, telegram);
    return true;
}


