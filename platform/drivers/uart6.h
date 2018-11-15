/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file uart6.h
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

    typedef struct 
{
	int16_t send_motopwm;
	int16_t send_check;
} send_flag_t;

typedef struct {
    float p;
    float i;
    float d;
}pid_debug_t;

typedef struct {
    int16_t i16_data[4];
    float f_data[4];
}send_data_t;


extern send_flag_t _send_flag;
extern pid_debug_t _pid_debug[18];
extern send_data_t _send;
extern void debug_init(void);
extern void debug_task(void);

extern int16_t Send_Check(void);
extern int16_t Send_int16_Data(void);
extern int16_t Send_float_data(void);
extern void Write_PID(pid_debug_t *pid1,pid_debug_t *pid2,pid_debug_t *pid3);
extern void uart6_config(void);
extern int uart6_read(uint8_t *buf, int len);
extern int uart6_write(uint8_t *buf, int len);

#ifdef __cplusplus
}
#endif
