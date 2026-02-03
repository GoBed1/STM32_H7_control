#include <string.h>
#include <stdio.h>
#include "modbus_tcp_server_protocol.h"
#include "modbus_tcp_server_reg.h"

static int mbtcp_read_coil_reg_cb(uint8_t *req, uint8_t *res);
static int mbtcp_write_coil_reg_cb(uint8_t *req, uint8_t *res);
static int mbtcp_write_coil_regs_cb(uint8_t *req, uint8_t *res);
static int mbtcp_read_holding_reg_cb(uint8_t *req, uint8_t *res);
static int mbtcp_write_holding_reg_cb(uint8_t *req, uint8_t *res);
static int mbtcp_write_holding_regs_cb(uint8_t *req, uint8_t *res);
static int mbtcp_read_input_reg_cb(uint8_t *req, uint8_t *res);
static int mbtcp_build_exception_cb(uint8_t *req, uint8_t exception_code, uint8_t *res);

int modbus_protocol_handle(uint8_t *req, uint16_t req_len, uint8_t *res) {
    int len = 0;
    if(mb_tcp_init_flag == 0)mb_init_reg();
    if (req_len < 8)return 0;

    memcpy(res, req, 6);
    res[6] = req[6];

    uint8_t fc = req[7];
    switch (fc) {
        case 1:
            len = mbtcp_read_coil_reg_cb(req, res);
            break;
        case 5:
            len = mbtcp_write_coil_reg_cb(req, res);
            break;
        case 15:
            len = mbtcp_write_coil_regs_cb(req, res);
            break;
        case 3:
            len = mbtcp_read_holding_reg_cb(req, res);
            break;
        case 6:
            len = mbtcp_write_holding_reg_cb(req, res);
            break;
        case 16:
            len = mbtcp_write_holding_regs_cb(req, res);
            break;
        case 4:
            len = mbtcp_read_input_reg_cb(req, res);
            break;
        default:
            len = mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_ILLEGAL_FUNCTION, res);
            break;
    }
    return len;
}

static int mbtcp_build_exception_cb(uint8_t *req, uint8_t exception_code, uint8_t *res) {
    res[7] = req[7] | 0x80;
    res[8] = exception_code;
    res[4] = 0x00;
    res[5] = 0x03;
    return 9; // MBAP (6) + UnitID (1) + Func (1) + Code (1)
}

static int mbtcp_read_holding_reg_cb(uint8_t *req, uint8_t *res) {
    uint16_t start_address = (req[8] << 8) | req[9];
    uint16_t quantity = (req[10] << 8) | req[11];

    if (quantity == 0 || quantity > 125 || start_address + quantity > HOLDING_REGISTERS_SIZE) {
        return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_ILLEGAL_ADDRESS, res);
    }

    uint8_t byte_count = quantity * 2;
    res[7] = 3; // Function Code
    res[8] = byte_count;

    for (int i = 0; i < quantity; i++) {
        uint16_t val;
        mb_err_t err = mb_get_holding_reg_by_address(start_address + i, &val);
        if (err != MB_OK) {
            printf("Failed to read holding register %d: %d", start_address + i, err);
            return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_DEVICE_FAILURE, res);
        }
        res[9 + i * 2] = (val >> 8) & 0xFF;
        res[10 + i * 2] = val & 0xFF;
    }

    uint16_t length = 3 + byte_count; // UnitID + FuncCode + ByteCount + Data
    res[4] = (length >> 8) & 0xFF;
    res[5] = length & 0xFF;

    return 6 + length; // MBAP + PDU
}

static int mbtcp_read_input_reg_cb(uint8_t *req, uint8_t *res) {
    uint16_t start_address = (req[8] << 8) | req[9];
    uint16_t quantity = (req[10] << 8) | req[11];

    if (quantity == 0 || quantity > 125 || start_address + quantity > INPUT_REGISTERS_SIZE) {
        return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_ILLEGAL_ADDRESS, res);
    }

    uint8_t byte_count = quantity * 2;
    res[7] = 4; // Function Code
    res[8] = byte_count;

    for (int i = 0; i < quantity; i++) {
        uint16_t val;
        mb_err_t err = mb_get_input_reg_by_address(start_address + i, &val);
        if (err != MB_OK) {
            printf("Failed to read input register %d: %d", start_address + i, err);
            return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_DEVICE_FAILURE, res);
        }
        res[9 + i * 2] = (val >> 8) & 0xFF;
        res[10 + i * 2] = val & 0xFF;
    }

    uint16_t length = 3 + byte_count;
    res[4] = (length >> 8) & 0xFF;
    res[5] = length & 0xFF;

    return 6 + length;
}

static int mbtcp_write_holding_reg_cb(uint8_t *req, uint8_t *res) {
    uint16_t reg_address = (req[8] << 8) | req[9];
    uint16_t value = (req[10] << 8) | req[11];

    if (reg_address >= HOLDING_REGISTERS_SIZE) {
        return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_ILLEGAL_ADDRESS, res);
    }

    mb_err_t err = mb_set_holding_reg_by_address(reg_address, value);
    if (err != MB_OK) {
        printf("Failed to write holding register %d: %d", reg_address, err);
        return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_DEVICE_FAILURE, res);
    }

    memcpy(res, req, 12);
    res[4] = 0x00;
    res[5] = 0x06;

    return 12;
}

static int mbtcp_write_holding_regs_cb(uint8_t *req, uint8_t *res) {
    uint16_t start_address = (req[8] << 8) | req[9];
    uint16_t quantity = (req[10] << 8) | req[11];
    uint8_t byte_count = req[12];

    if (quantity == 0 || quantity > 123 || byte_count != quantity * 2 || start_address + quantity > HOLDING_REGISTERS_SIZE) {
        return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_ILLEGAL_ADDRESS, res);
    }

    for (int i = 0; i < quantity; i++) {
        uint16_t value = (req[13 + i * 2] << 8) | req[14 + i * 2];
        mb_err_t err = mb_set_holding_reg_by_address(start_address + i, value);
        if (err != MB_OK) {
            printf("Failed to write holding register %d: %d", start_address + i, err);
            return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_DEVICE_FAILURE, res);
        }
    }

    res[7] = 16; // Function Code
    res[8] = req[8];
    res[9] = req[9];
    res[10] = req[10];
    res[11] = req[11];

    uint16_t length = 6; // UnitID + Func + StartAddr + Quantity
    res[4] = 0x00;
    res[5] = length;

    return 6 + length;
}

static int mbtcp_read_coil_reg_cb(uint8_t *req, uint8_t *res) {
    uint16_t start_address = (req[8] << 8) | req[9];
    uint16_t quantity      = (req[10] << 8) | req[11];

    if (quantity == 0 || quantity > 2000 || (start_address + quantity) > COILS_REGISTERS_SIZE) {
        return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_ILLEGAL_ADDRESS, res);
    }

    uint8_t byte_count = (quantity + 7) / 8;
    res[7] = 1; // Function code
    res[8] = byte_count;

    // 清零数据字段
    for(uint8_t i = 0; i < byte_count; ++i){
        res[9 + i] = 0;
    }

    for(uint16_t i = 0; i < quantity; ++i){
        uint8_t coil_val = 0;
        mb_err_t err = mb_get_coil_reg_by_address(start_address + i, &coil_val);
        if(err != MB_OK){
            return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_DEVICE_FAILURE, res);
        }
        if(coil_val){
            res[9 + (i / 8)] |= (1 << (i % 8)); // LSB first
        }
    }

    uint16_t length = 3 + byte_count; // UnitID + Func + ByteCount + Data
    res[4] = (length >> 8) & 0xFF;
    res[5] = length & 0xFF;

    return 6 + length;
}

static int mbtcp_write_coil_reg_cb(uint8_t *req, uint8_t *res) {
    uint16_t coil_address = (req[8] << 8) | req[9];
    uint16_t value_field  = (req[10] << 8) | req[11];

    uint8_t coil_state = (value_field == 0xFF00) ? 1 : 0;

    if(coil_address >= COILS_REGISTERS_SIZE){
        return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_ILLEGAL_ADDRESS, res);
    }

    mb_err_t err = mb_set_coil_reg_by_address(coil_address, coil_state);
    if(err != MB_OK){
        return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_DEVICE_FAILURE, res);
    }

    // 正常响应是请求的回显
    memcpy(res, req, 12);
    res[4] = 0x00;
    res[5] = 0x06;
    return 12;
}

static int mbtcp_write_coil_regs_cb(uint8_t *req, uint8_t *res) {
    uint16_t start_address = (req[8] << 8) | req[9];
    uint16_t quantity      = (req[10] << 8) | req[11];
    uint8_t  byte_count    = req[12];

    if(quantity == 0 || quantity > 1968 || byte_count != (uint8_t)((quantity + 7) / 8) || (start_address + quantity) > COILS_REGISTERS_SIZE){
        return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_ILLEGAL_ADDRESS, res);
    }

    for(uint16_t i = 0; i < quantity; ++i){
        uint8_t coil_state = (req[13 + (i / 8)] >> (i % 8)) & 0x01;
        mb_err_t err = mb_set_coil_reg_by_address(start_address + i, coil_state);
        if(err != MB_OK){
            return mbtcp_build_exception_cb(req, MODBUS_EXCEPTION_DEVICE_FAILURE, res);
        }
    }

    res[7]  = 15; // Function code
    res[8]  = req[8];
    res[9]  = req[9];
    res[10] = req[10];
    res[11] = req[11];

    uint16_t length = 6; // UnitID + Func + StartAddr + Quantity
    res[4] = 0x00;
    res[5] = length;

    return 6 + length;
}
