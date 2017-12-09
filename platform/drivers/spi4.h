/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file spi4.h
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 *
 ***************************************************************************/

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void spi4_config(void);

int spi4_read(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len);
int spi4_write(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif
