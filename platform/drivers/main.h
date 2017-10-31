/*      厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file main.h
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void _Error_Handler(char * file, int line);

extern uint64_t time(void);
extern void usleep(uint32_t us);

extern int read(char *path, uint8_t *buf, int len);
extern int write(char *paht, uint8_t *buf, int len);

#ifdef __cplusplus
}
#endif
