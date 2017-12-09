/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file scheduler.h
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 *
 ***************************************************************************/

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint64_t time(void);
extern void usleep(uint32_t us);

void _Error_Handler(char * file, int line);

extern void scheduler(void);

extern int read(char *path, uint8_t *buf, int len);
extern int write(char *paht, uint8_t *buf, int len);

#ifdef __cplusplus
}
#endif
