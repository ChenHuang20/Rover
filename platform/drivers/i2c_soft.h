/***************************************************************************
 *
 *  Copyright (c) 2017 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file i2c_soft.h
 *
 * @author zwh <zwh@raaworks.com>
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

void i2c_soft_config(void);

int i2c_soft_read(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len);
int i2c_soft_write(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif
