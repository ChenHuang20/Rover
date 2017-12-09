/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file esc6623.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 *
 ***************************************************************************/

#include "esc6623.h"
#include "scheduler.h"
#include "topics.h"

static char path[3] = { 'c', 1, 0 };

void motor6623_read()
{
//    uint8_t buf[8] = { 0 };

//    if (read(path, buf, sizeof(buf)) == 0) {
//        _gimbal_velocity.timestamp = time();

//        int16_t in[4] = {
//            (int16_t)buf[0] << 8 | buf[1],
//            (int16_t)buf[2] << 8 | buf[3],
//            (int16_t)buf[4] << 8 | buf[5],
//            (int16_t)buf[6] << 8 | buf[7]
//        };

//        float out[4] = {
//            in[0] / 7700.0f,
//            -in[1] / 7700.0f,
//            in[2] / 7700.0f,
//            -in[3] / 7700.0f
//        };

//        for (int i = 0; i < 4; i++) {
//            out[i] = out[i] < -1.0f ? -1.0f : out[i] > 1.0f ? 1.0f : out[i];
//            _wheel_velocity.wheel[i] = out[i];
//        }
//    }
}

void motor6623_write()
{
//    uint8_t buf[12] = { 0 };
//    int16_t out[4];

//    buf[0] = 0x1ff >> 24;
//    buf[1] = 0x1ff >> 16;
//    buf[2] = 0x1ff >> 8;
//    buf[3] = 0x1ff & 0xff;

//    float in[2] = {
//        _gimbal_velocity_setpoint.gimbal[YAW],
//        _gimbal_velocity_setpoint.gimbal[PITCH],
//    };
//    
//    in[PITCH] = _radio.pitch;
//    in[YAW] = _radio.pitch;

//    for (int i = 0; i < 2; i++) {
////        in[i] = in[i] < -1.0f ? -1.0f : in[i] > 1.0f ? 1.0f : in[i];
//        out[i] = in[i]*600;
//    }

//    buf[4] = out[0] >> 8;
//    buf[5] = out[0];
//    buf[6] = out[1] >> 8;
//    buf[7] = out[1];
//    buf[8] = out[2] >> 8;
//    buf[9] = out[2];
//    buf[10] = out[3] >> 8;
//    buf[11] = out[3];

//    write(path, buf, sizeof(buf));
}
