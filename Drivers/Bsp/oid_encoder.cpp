#include "oid_encoder.hpp"

void OidEncoder::init_(void){
	if (interface_!= nullptr) {
        GPIO_TypeDef *enPort = nullptr;
        uint16_t enPin = 0;
        uint16_t timeoutTicks = TIMEOUT_MODBUS;
        mb_hardware_t xTypeHW = USART_HW;
		initMaster(interface_, enPort, enPin, timeoutTicks, xTypeHW);
	}
}

bool OidEncoder::requestPositionAsync()
{
    // 位置占两个 16 位保持寄存器，从 0x0000 开始
    return asyncReadHolding(slave_id_, 0x0000, 2, posRegs_);
}

bool OidEncoder::requestMultiAsync()
{
    // 位置占两个 16 位保持寄存器，从 0x0000 开始
    return asyncReadHolding(slave_id_, 0x0002, 2, posRegs_);
}

bool OidEncoder::requestSpeedAsync()
{
    // 速度占一个 16 位保持寄存器，从 0x0002 开始（示例）
    return asyncReadHolding(slave_id_, 0x0002, 1, speedReg_);
}

uint32_t OidEncoder::getLastPositionRaw() const
{
    // 高 16 位在前
    return (static_cast<uint32_t>(posRegs_[0]) << 16) | static_cast<uint32_t>(posRegs_[1]);
}

uint16_t OidEncoder::getLastSpeedRaw() const
{
    return speedReg_[0];
}


