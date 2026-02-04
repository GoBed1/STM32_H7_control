#pragma once

#include <stdint.h>
#include "modbus_rtu.hpp"

// 驱动层：基于 ModbusRtu 的编码器设备。
// 约定：
// - 某品牌/型号的 OID 编码器：位置寄存器位于保持寄存器 0x0000 开始，占 2 个 16 位（高位在前）。
// - 速度寄存器位于 0x0002，占 1 个 16 位。
// 可根据实际协议修改寄存器地址与长度。
typedef UART_HandleTypeDef ModbusRtuInterface;
class OidEncoder : public ModbusRtu {
public:
    explicit OidEncoder(
                       uint8_t device_id,  
                       uint8_t slave_id, 
                       ModbusRtuInterface *interface = nullptr)
        : device_id_(device_id), slave_id_(slave_id), interface_(interface){}

    void init_(void);

    bool requestMultiAsync();

    // 读取位置（异步）：发起读取 2 个保持寄存器。应答到来时，调用者应从 getLastPositionRaw() 读取原始值再自行转单位。
    bool requestPositionAsync();

    // 读取速度（异步）：发起读取 1 个保持寄存器。
    bool requestSpeedAsync();

    // 提供结果缓冲区指针，供上层在通知到达后读取。
    // 注意：缓冲在异步周期内必须保持有效（本实现内部持有静态缓冲，线程安全由任务串行保障）。
    uint32_t getLastPositionRaw() const; // 将两个 16 位合成为 32 位
    uint16_t getLastSpeedRaw() const;
    uint8_t getDeviceId() const { return device_id_; }
        uint16_t posRegs_[2]{}; // 高 16 位在 posRegs_[0]，低 16 位在 posRegs_[1]


private:
    uint8_t device_id_{0};
    uint8_t slave_id_{1};
    // 内部结果缓冲（与 ModbusRtu 发起的异步查询共享）
   // uint16_t posRegs_[2]{}; // 高 16 位在 posRegs_[0]，低 16 位在 posRegs_[1]
    uint16_t speedReg_[1]{};
    ModbusRtuInterface *interface_;
};


