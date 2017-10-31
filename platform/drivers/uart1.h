/*      厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file uart1.h
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void uart1_config(void);
extern int uart1_read(uint8_t *buf, int len);

#ifdef __cplusplus
}
#endif
