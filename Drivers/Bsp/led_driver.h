/**
 * @file    led_driver.h
 * @brief   LED驱动头文件
 * @author  River
 * @date    2025-9-14
 * @version 2.0
 * copyright Copyright (c) 2024 River. All rights reserved.
 * license MIT License, see LICENSE file.
 * note 2025-8-1: v1.0, 初始版本    
 *      2025-9-14: v2.0, 增加C接口实现方式
 *
 */
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// C interface
void register_led(const char* name, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void set_led(const char* name, GPIO_PinState state);
void toggle_led(const char* name);

#ifdef __cplusplus
}
#endif
