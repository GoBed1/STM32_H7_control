#include "modbus_tcp_server_reg.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdbool.h>
// #include "lfs.h"
#include "stm32h7xx_hal_cortex.h"

#define MBTR_E(...) LOG_ERR("MBTR", __VA_ARGS__)
#define MBTR_I(...) LOG_INFO("MBTR", __VA_ARGS__)

#define MODBUS_REGS_FILE "modbus_regs.bin"
#define MODBUS_MUTEX_TIMEOUT 100 // ms

// extern lfs_t lfs_W25Q;
// lfs_t *modbus_lfs;

uint8_t mb_tcp_init_flag = 0;
static osMutexId_t mb_tcp_mutex_id = NULL;
uint16_t holding_regs_database[HOLDING_REGISTERS_SIZE] = {0};
uint16_t input_regs_database[INPUT_REGISTERS_SIZE] = {0};
uint8_t coil_regs_database[COILS_REGISTERS_SIZE] = {0};

#define RETURN_IF_OUT_OF_RANGE(addr, max) \
    do { \
        if ((addr) >= (max)) { \
            return MB_ERR_OUT_OF_RANGE; \
        } \
    } while (0)

/* ============================================================================
 * Private Function Declarations
 * ============================================================================ */
// mb_err_t mb_save_to_flash(void)
// {
//    lfs_file_t file;
//    int err = lfs_file_open(modbus_lfs, &file, MODBUS_REGS_FILE, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
//    if (err != 0) {
//        return MB_ERR_FILE_OPERATION;
//    }

//    lfs_file_write(modbus_lfs, &file, coil_regs_database, sizeof(coil_regs_database));
//    // do not save RTC and Stepper registers that are located at the beginning 6 registers of the array
//    lfs_file_write(modbus_lfs, &file, holding_regs_database + 6,
//                    sizeof(holding_regs_database) - 6 * sizeof(uint16_t));
//    lfs_file_close(modbus_lfs, &file);
//    return MB_OK;
// }
// mb_err_t mb_load_from_flash(void)
// {
//    lfs_file_t file;
//    int err = lfs_file_open(modbus_lfs, &file, MODBUS_REGS_FILE, LFS_O_RDONLY);
//    // if the file is corrupted, create a new file, save the default values to the file
//    if(err == LFS_ERR_CORRUPT){
//        err = lfs_file_open(modbus_lfs, &file, MODBUS_REGS_FILE, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
//        if (err != 0) {
//            return MB_ERR_FILE_OPERATION;
//        }
//        lfs_file_write(modbus_lfs, &file, coil_regs_default, sizeof(coil_regs_default));
//        lfs_file_write(modbus_lfs, &file, holding_regs_default + 6,
//                  sizeof(holding_regs_default) - 6 * sizeof(uint16_t));
//        lfs_file_close(modbus_lfs, &file);
//        return MB_OK;
//    }
//    // other error
//    else if (err != 0) {
//        return MB_ERR_FILE_OPERATION;
//    }

//    lfs_file_read(modbus_lfs, &file, coil_regs_database, sizeof(coil_regs_database));
//    lfs_file_read(modbus_lfs, &file, holding_regs_database + 6,
//                  sizeof(holding_regs_database) - 6 * sizeof(uint16_t));
//    lfs_file_close(modbus_lfs, &file);
//    return MB_OK;
// }

static mb_err_t mb_acquire_mutex(void)
{
    if (osMutexAcquire(mb_tcp_mutex_id, MODBUS_MUTEX_TIMEOUT) != osOK) {
        return MB_ERR_ACQUIRE_MUTEX_TIMEOUT;
    }
    return MB_OK;
}

static void mb_release_mutex(void)
{
    osMutexRelease(mb_tcp_mutex_id);
}

mb_err_t mb_init_reg(void)
{
    if (mb_tcp_init_flag != false) {
        return MB_OK;
    }

    // HACK
//    modbus_lfs = &lfs_W25Q;
//    // Try to load from flash
//    mb_err_t err = mb_load_from_flash();
//    if (err != MB_OK) {
//        err = mb_default_regs(REG_TYPE_ALL);
//        if (err == MB_OK) {
//           err = mb_save_to_flash();
//        }else{
// 		  return MB_ERR_DEFAULT_FAIL;
//        }
//    }

    if (mb_tcp_mutex_id == NULL) {
        const osMutexAttr_t mutex_attr = {
            .name = "ModbusTcpRegMutex",
            .attr_bits = osMutexRecursive,
            .cb_mem = NULL,
            .cb_size = 0U
        };
        mb_tcp_mutex_id = osMutexNew(&mutex_attr);
        if (mb_tcp_mutex_id == NULL) {
            return MB_ERR_ACQUIRE_MUTEX_TIMEOUT;
        }
    }

    mb_err_t err = mb_default_regs(REG_TYPE_ALL);
    if (err != MB_OK) {
        return MB_ERR_DEFAULT_FAIL;
    }

    mb_tcp_init_flag = true;
    return MB_OK;
}

mb_err_t mb_clear_regs(int type)
{
    mb_err_t err = mb_acquire_mutex();
    if (err != MB_OK) return err;

    if (type & REG_TYPE_HOLDING_REGISTER) {
        for (uint16_t i = 0; i < HOLDING_REGISTERS_SIZE; ++i) {
            holding_regs_database[i] = 0;
        }
    }
    if (type & REG_TYPE_INPUT_REGISTER) {
        for (uint16_t i = 0; i < INPUT_REGISTERS_SIZE; ++i) {
            input_regs_database[i] = 0;
        }
    }
    if (type & REG_TYPE_COIL) {
        for (uint16_t i = 0; i < COILS_REGISTERS_SIZE; ++i) {
            coil_regs_database[i] = 0;
        }
    }
    
    mb_release_mutex();

    return MB_OK;
}

mb_err_t mb_default_regs(int type)
{
    mb_err_t err = mb_acquire_mutex();
    if (err != MB_OK) return err;

    if (type & REG_TYPE_HOLDING_REGISTER) {
        for (uint16_t i = 0; i < REG_HOLDING_NUM && i < HOLDING_REGISTERS_SIZE; ++i) {
            holding_regs_database[MB_HOLDING_REGISTERS[i].address] = MB_HOLDING_REGISTERS[i].default_value;
        }
    }
    if (type & REG_TYPE_INPUT_REGISTER) {
        for (uint16_t i = 0; i < REG_INPUT_NUM && i < INPUT_REGISTERS_SIZE; ++i) {
            input_regs_database[MB_INPUT_REGISTERS[i].address] = MB_INPUT_REGISTERS[i].default_value;
        }
    }
    if (type & REG_TYPE_COIL) {
        for (uint16_t i = 0; i < MB_COIL_REG_NUM && i < COILS_REGISTERS_SIZE; ++i) {
            coil_regs_database[MB_COIL_REGISTERS[i].address] = (uint8_t)MB_COIL_REGISTERS[i].default_value;
        }
    }

    mb_release_mutex();

    return MB_OK;
}

// static mb_err_t mb_get_reg(const mb_reg_t *reg, void *value)
// {
//     if (!reg || !value) return MB_ERR_NULL_POINTER;
//     switch (reg->type) {
//         case REG_TYPE_HOLDING_REGISTER:
//             RETURN_IF_OUT_OF_RANGE(reg->address, HOLDING_REGISTERS_SIZE);
//             *(uint16_t*)value = holding_regs_database[reg->address];
//             break;
//         case REG_TYPE_INPUT_REGISTER:
//             RETURN_IF_OUT_OF_RANGE(reg->address, INPUT_REGISTERS_SIZE);
//             *(uint16_t*)value = input_regs_database[reg->address];
//             break;
//         case REG_TYPE_COIL:
//             RETURN_IF_OUT_OF_RANGE(reg->address, COILS_REGISTERS_SIZE);
//             *(uint8_t*)value = coil_regs_database[reg->address];
//             break;
//         default:
//             return MB_ERR_TYPE;
//     }
//     return MB_OK;
// }

// static mb_err_t mb_set_reg(const mb_reg_t *reg, const void *value)
// {
//     if (!reg || !value) return MB_ERR_NULL_POINTER;
//     switch (reg->type) {
//         case REG_TYPE_HOLDING_REGISTER:
//             RETURN_IF_OUT_OF_RANGE(reg->address, HOLDING_REGISTERS_SIZE);
//             holding_regs_database[reg->address] = *(const uint16_t*)value;
//             break;
//         case REG_TYPE_INPUT_REGISTER:
//             RETURN_IF_OUT_OF_RANGE(reg->address, INPUT_REGISTERS_SIZE);
//             input_regs_database[reg->address] = *(const uint16_t*)value;
//             break;
//         case REG_TYPE_COIL:
//             RETURN_IF_OUT_OF_RANGE(reg->address, COILS_REGISTERS_SIZE);
//             coil_regs_database[reg->address] = (*(const uint8_t*)value) ? 1 : 0;
//             break;
//         default:
//             return MB_ERR_TYPE;
//     }
//     return MB_OK;
// }

// mb_err_t mb_get_reg_safe(const mb_reg_t *reg, void *value)
// {
//     if (!reg || !value) return MB_ERR_NULL_POINTER;
//     mb_err_t err = mb_acquire_mutex();
//     if (err != MB_OK) return err;
//     err = mb_get_reg(reg, value);
//     mb_release_mutex();
//     return err;
// }

// mb_err_t mb_set_reg_safe(const mb_reg_t *reg, const void *value)
// {
//     if (!reg || !value) return MB_ERR_NULL_POINTER;
//     mb_err_t err = mb_acquire_mutex();
//     if (err != MB_OK) return err;
//     err = mb_set_reg(reg, value);
//     mb_release_mutex();
//     return err;
// }

mb_err_t mb_set_holding_reg_by_address(uint16_t address, uint16_t value)
{
    RETURN_IF_OUT_OF_RANGE(address, HOLDING_REGISTERS_SIZE);
    mb_err_t err = mb_acquire_mutex();
    if (err != MB_OK) return err;
    holding_regs_database[address] = value;
    mb_release_mutex();
    return MB_OK;
}

mb_err_t mb_get_holding_reg_by_address(uint16_t address, uint16_t *value)
{
    if (!value) return MB_ERR_NULL_POINTER;

    RETURN_IF_OUT_OF_RANGE(address, HOLDING_REGISTERS_SIZE);
    if (!value) return MB_ERR_NULL_POINTER;

    mb_err_t err = mb_acquire_mutex();
    if (err != MB_OK)return err;

    *value = holding_regs_database[address];

    mb_release_mutex();

    return err;
}

mb_err_t mb_set_input_reg_by_address(uint16_t address, uint16_t value)
{
    RETURN_IF_OUT_OF_RANGE(address, INPUT_REGISTERS_SIZE);

    mb_err_t err = mb_acquire_mutex();
    if (err != MB_OK)return err;

    input_regs_database[address] = value;

    mb_release_mutex();

    return err;
}

mb_err_t mb_get_input_reg_by_address(uint16_t address, uint16_t *value)
{
    if (!value) return MB_ERR_NULL_POINTER;

    RETURN_IF_OUT_OF_RANGE(address, INPUT_REGISTERS_SIZE);
    if (!value) return MB_ERR_NULL_POINTER;

    mb_err_t err = mb_acquire_mutex();
    if (err != MB_OK)return err;

    *value = input_regs_database[address];

    mb_release_mutex();

    return err;
}

mb_err_t mb_get_coil_reg_by_address(uint16_t address, uint8_t *value)
{
    if (!value) return MB_ERR_NULL_POINTER;

    RETURN_IF_OUT_OF_RANGE(address, COILS_REGISTERS_SIZE);
    if (!value) return MB_ERR_NULL_POINTER;

    mb_err_t err = mb_acquire_mutex();
    if (err != MB_OK)return err;

    *value = coil_regs_database[address];

    mb_release_mutex();

    return err;
}

mb_err_t mb_set_coil_reg_by_address(uint16_t address, uint8_t value)
{
    RETURN_IF_OUT_OF_RANGE(address, COILS_REGISTERS_SIZE);

    mb_err_t err = mb_acquire_mutex();
    if (err != MB_OK)return err;

    coil_regs_database[address] = value ? 1 : 0;

    mb_release_mutex();

    return err;
}

