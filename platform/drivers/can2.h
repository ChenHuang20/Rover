/***************************************************************************
 *
 *                          ���Ŵ�ѧ���ϻ����˶�
 *
 * @2017 all rights reserved
 *
 * @file can2.h
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

extern void can2_config(void);
extern int can2_read(uint8_t *buf, int len);
extern int can2_write(uint8_t *buf, int len);

#ifdef __cplusplus
}
#endif
