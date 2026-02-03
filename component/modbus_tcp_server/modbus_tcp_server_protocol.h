#ifndef MODBUS_PROTOCOL_H
#define MODBUS_PROTOCOL_H

#include <stdint.h>

// Modbus 异常码
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION    0x01
#define MODBUS_EXCEPTION_ILLEGAL_ADDRESS     0x02
#define MODBUS_EXCEPTION_ILLEGAL_VALUE       0x03
#define MODBUS_EXCEPTION_DEVICE_FAILURE      0x04

// #define MODBUS_DEBUG

/**
 * @brief 处理一个完整的 Modbus TCP 请求.
 *
 * 这个函数解析传入的 Modbus 请求PDU (Protocol Data Unit)，执行相应的读/写操作，
 * 并构建一个响应PDU。它不处理MBAP头，只处理功能码和数据部分。
 *
 * @param request 指向包含完整 Modbus 请求的缓冲区 (包括 MBAP 头).
 * @param req_len 请求的长度.
 * @param response 指向将要写入 Modbus 响应的缓冲区 (包括 MBAP 头).
 * @return 生成的响应帧的总长度。如果出现错误或无需响应，则返回 0.
 */
int modbus_protocol_handle(uint8_t *request, uint16_t req_len, uint8_t *response);

#endif // MODBUS_PROTOCOL_H
