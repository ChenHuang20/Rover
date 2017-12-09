/***************************************************************************
 *
 *                          ���Ŵ�ѧ���ϻ����˶�
 *
 * @2017 all rights reserved
 *
 * @file spi5.h
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

void spi5_config(void);

int spi5_read(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len);
int spi5_write(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif
