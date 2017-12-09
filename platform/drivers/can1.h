/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file can1.h
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

extern void can1_config(void);
extern int can1_read(uint8_t *buf, int len);
extern int can1_write(uint8_t *buf, int len);

#ifdef __cplusplus
}
#endif
