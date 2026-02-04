#ifndef __USER_LOGIC_H
#define __USER_LOGIC_H
#include "main.h"
// #include "nex_modbus_rtu_client.h"
// #include "com_debug.h"

mb_err_op_t modbus_RxData_logic(uint8_t *Rx_data, uint16_t RxLen);

void modbus_TxData_logic(void);

void timeout_resend_logic(void);
void record_tx_cmd(uint8_t *cmd, uint8_t len);
#endif // __USER_LOGIC_H